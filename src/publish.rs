use std::time::{Duration, Instant};

use anyhow::Context;
use moq_lite::*;
use url::Url;

use crate::schema::{self, now_ms, shed_thresholds, TRACKS};

/// Write a frame to the correct destination.
/// In single-stream mode, prefixes each frame with a 1-byte track index for demuxing.
fn emit_frame(
    mux_track: &mut Option<TrackProducer>,
    track_producers: &mut [TrackProducer],
    idx: usize,
    payload: Vec<u8>,
) {
    if let Some(ref mut mux) = mux_track {
        let mut buf = Vec::with_capacity(1 + payload.len());
        buf.push(idx as u8);
        buf.extend_from_slice(&payload);
        mux.write_frame(bytes::Bytes::from(buf));
    } else {
        track_producers[idx].write_frame(bytes::Bytes::from(payload));
    }
}

// --- Publisher-side track shedding ---
//
// When QUIC bandwidth is scarce, stop writing low-priority frames entirely.
// QUIC reliable streams queue data instead of dropping it, so without shedding,
// low-priority frames accumulate in Quinn's send buffer and cause unbounded
// latency growth across ALL streams.
//
// The CC estimate inflates when tracks are shed (less data → more spare capacity),
// so we use a sliding-minimum filter: the lowest CC estimate over the last
// BW_WINDOW seconds reflects the true bottleneck, not the transient spare capacity.
//
// Hysteresis prevents oscillation:
//   - Shed when filtered_bw < threshold
//   - Restore when filtered_bw > threshold × RESTORE_HEADROOM AND shed for >= MIN_SHED_DURATION

const RESTORE_HEADROOM: f64 = 1.5;
const MIN_SHED_DURATION: Duration = Duration::from_secs(5);
const BW_WINDOW: Duration = Duration::from_secs(5);

/// Sliding-minimum bandwidth filter.
/// Tracks (timestamp, bw) samples and returns the minimum over the last BW_WINDOW.
struct BwFilter {
    samples: std::collections::VecDeque<(Instant, u64)>,
}

impl BwFilter {
    fn new() -> Self {
        Self {
            samples: std::collections::VecDeque::new(),
        }
    }

    /// Record a new sample and return the minimum over the window.
    fn update(&mut self, bw: u64) -> u64 {
        let now = Instant::now();
        self.samples.push_back((now, bw));
        // Evict samples older than the window
        while let Some(&(t, _)) = self.samples.front() {
            if now.duration_since(t) > BW_WINDOW {
                self.samples.pop_front();
            } else {
                break;
            }
        }
        self.samples.iter().map(|&(_, v)| v).min().unwrap_or(bw)
    }
}

struct ShedState {
    shed: [bool; TRACKS.len()],
    shed_since: [Option<Instant>; TRACKS.len()],
    thresholds: [u64; TRACKS.len()],
    bw_filter: BwFilter,
}

impl ShedState {
    fn new() -> Self {
        Self {
            shed: [false; TRACKS.len()],
            shed_since: [None; TRACKS.len()],
            thresholds: shed_thresholds(),
            bw_filter: BwFilter::new(),
        }
    }

    /// Returns true if track `idx` should emit a frame given the current bandwidth estimate.
    /// Safety (idx 0) is never shed. If no estimate is available, all tracks emit.
    fn should_emit(&mut self, idx: usize, bw: Option<u64>) -> bool {
        if idx == 0 {
            return true;
        } // never shed safety
        let Some(raw_bw) = bw else {
            return true;
        }; // no estimate = emit everything

        let bw = self.bw_filter.update(raw_bw);

        if self.shed[idx] {
            // Currently shed — restore only if filtered bw > threshold×1.5 AND held for >= 5s
            let restore_threshold = (self.thresholds[idx] as f64 * RESTORE_HEADROOM) as u64;
            let held_long_enough = self.shed_since[idx]
                .map(|t| t.elapsed() >= MIN_SHED_DURATION)
                .unwrap_or(true);
            if bw > restore_threshold && held_long_enough {
                self.shed[idx] = false;
                self.shed_since[idx] = None;
                tracing::info!(
                    track = %TRACKS[idx].name,
                    bw_kbps = bw / 1000,
                    raw_bw_kbps = raw_bw / 1000,
                    threshold_kbps = self.thresholds[idx] / 1000,
                    "track restored"
                );
                true
            } else {
                false
            }
        } else {
            // Currently active — shed if filtered bw < threshold
            if bw < self.thresholds[idx] {
                self.shed[idx] = true;
                self.shed_since[idx] = Some(Instant::now());
                tracing::info!(
                    track = %TRACKS[idx].name,
                    bw_kbps = bw / 1000,
                    raw_bw_kbps = raw_bw / 1000,
                    threshold_kbps = self.thresholds[idx] / 1000,
                    "track shed"
                );
                false
            } else {
                true
            }
        }
    }
}

/// Publish all telemetry tracks at their configured rates.
///
/// Connects to a MoQ relay, creates one QUIC stream per track (or a single
/// multiplexed stream in `--single-stream` mode), and loops forever generating
/// synthetic telemetry at IHMC KST-aligned rates.
///
/// When shedding is enabled (default in priority mode), the publisher reads
/// Quinn's congestion controller bandwidth estimate and stops writing frames
/// for low-priority tracks when bandwidth is scarce.
pub async fn run(
    client: moq_native::Client,
    url: &Url,
    broadcast_name: &str,
    no_priority: bool,
    single_stream: bool,
    no_shed: bool,
    log_bandwidth: bool,
) -> anyhow::Result<()> {
    if single_stream {
        tracing::warn!("--single-stream: all tracks multiplexed into one QUIC stream (WebRTC-style HOL blocking)");
    } else if no_priority {
        tracing::warn!("--no-priority: all tracks set to priority 0 (no starvation)");
    }
    tracing::info!(broadcast = %broadcast_name, url = %url, no_priority, single_stream, "publishing telemetry");

    let origin = Origin::produce();
    let mut broadcast = Broadcast::produce();

    // In single-stream mode, all data goes through one MoQ track (one QUIC stream).
    // This reproduces WebRTC's head-of-line blocking: any congestion blocks everything.
    let mut mux_track: Option<TrackProducer> = None;
    let mut track_producers: Vec<TrackProducer> = Vec::new();

    if single_stream {
        let track = broadcast.create_track(Track {
            name: "multiplexed".to_string(),
            priority: 0,
        })?;
        tracing::info!("created single multiplexed track (WebRTC HOL simulation)");
        mux_track = Some(track);
    } else {
        for def in TRACKS {
            let priority = if no_priority { 0 } else { def.priority };
            let track = broadcast.create_track(Track {
                name: def.name.to_string(),
                priority,
            })?;
            tracing::info!(track = %def.name, priority, rate_hz = def.rate_hz, "created track");
            track_producers.push(track);
        }
    }

    origin.publish_broadcast(broadcast_name, broadcast.consume());

    let session = client
        .with_publish(origin.consume())
        .connect(url.clone())
        .await
        .context("failed to connect")?;

    // Bandwidth estimation from Quinn's congestion controller.
    // Returns None if the backend doesn't support estimated_send_rate().
    let bw_consumer = session.send_bandwidth();

    // Optionally log bandwidth estimate every second (--log-bandwidth)
    if log_bandwidth {
        if let Some(ref bw) = bw_consumer {
            let bw = bw.clone();
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_secs(1));
                loop {
                    interval.tick().await;
                    let estimate = bw.peek();
                    tracing::info!(bandwidth_bps = ?estimate, "CC bandwidth estimate");
                }
            });
        } else {
            tracing::warn!("send_bandwidth() returned None — backend doesn't support estimation");
        }
    }

    // Shedding is only meaningful with per-stream priority and multiple streams.
    let shed_enabled = !single_stream && !no_priority && !no_shed;
    if shed_enabled {
        let thresholds = shed_thresholds();
        tracing::info!(
            ?thresholds,
            "track shedding enabled (5s sliding-min filter, restore at threshold×1.5 after 5s)"
        );
    } else if !single_stream && !no_priority && no_shed {
        tracing::info!("track shedding disabled (--no-shed)");
    }

    let mut shed_state = ShedState::new();

    // Track indices (must match TRACKS order in schema.rs):
    // 0: safety/heartbeat      P255  10 Hz
    // 1: control/streaming     P200 167 Hz
    // 2: control/task_status   P200  10 Hz
    // 3: sensors/joints        P150 100 Hz
    // 4: sensors/force_torque  P100 100 Hz
    // 5: sensors/imu           P50  200 Hz
    // 6: perception/pointcloud P10    5 Hz
    // 7: video/camera0         P1    30 Hz
    // 8: video/camera1         P1    30 Hz
    // 9: video/camera2         P1    30 Hz

    let mut heartbeat_interval = tokio::time::interval(std::time::Duration::from_millis(100));   // 10 Hz
    let mut streaming_interval = tokio::time::interval(std::time::Duration::from_millis(6));     // ~167 Hz
    let mut task_status_interval = tokio::time::interval(std::time::Duration::from_millis(100)); // 10 Hz
    let mut joints_interval = tokio::time::interval(std::time::Duration::from_millis(10));       // 100 Hz
    let mut ft_interval = tokio::time::interval(std::time::Duration::from_millis(10));           // 100 Hz
    let mut imu_interval = tokio::time::interval(std::time::Duration::from_millis(5));           // 200 Hz
    let mut pointcloud_interval = tokio::time::interval(std::time::Duration::from_millis(200));  // 5 Hz
    let mut video_interval = tokio::time::interval(std::time::Duration::from_millis(33));        // ~30 Hz
    let mut video1_interval = tokio::time::interval(std::time::Duration::from_millis(33));       // ~30 Hz
    let mut video2_interval = tokio::time::interval(std::time::Duration::from_millis(33));       // ~30 Hz

    let mut heartbeat_seq: u64 = 0;

    let mode = if single_stream {
        "single-stream"
    } else if no_priority {
        "no-priority"
    } else if shed_enabled {
        "priority+shedding"
    } else {
        "priority"
    };
    tracing::info!("publishing started — all {} tracks active (IHMC KST-aligned), mode={mode}", TRACKS.len());

    tokio::select! {
        res = session.closed() => res.context("session closed"),
        _ = async {
            loop {
                // Read bandwidth once per loop iteration (not per arm).
                // If shedding is disabled, bw_estimate stays None → should_emit always returns true.
                let bw_estimate = if shed_enabled {
                    bw_consumer.as_ref().and_then(|c| c.peek())
                } else {
                    None
                };

                tokio::select! {
                    _ = heartbeat_interval.tick() => {
                        // Heartbeat (idx 0) is never shed — always emit
                        let ts = now_ms();
                        heartbeat_seq += 1;
                        let data = schema::generate_heartbeat(ts, heartbeat_seq);
                        let payload = bincode::serialize(&data).expect("fixed-size Heartbeat");
                        emit_frame(&mut mux_track, &mut track_producers, 0, payload);
                    }
                    _ = streaming_interval.tick() => {
                        if shed_state.should_emit(1, bw_estimate) {
                            let ts = now_ms();
                            let data = schema::generate_streaming_command(ts);
                            let payload = bincode::serialize(&data).expect("fixed-size StreamingCommand");
                            emit_frame(&mut mux_track, &mut track_producers, 1, payload);
                        }
                    }
                    _ = task_status_interval.tick() => {
                        if shed_state.should_emit(2, bw_estimate) {
                            let ts = now_ms();
                            let data = schema::generate_task_status(ts);
                            let payload = bincode::serialize(&data).expect("fixed-size TaskStatus");
                            emit_frame(&mut mux_track, &mut track_producers, 2, payload);
                        }
                    }
                    _ = joints_interval.tick() => {
                        if shed_state.should_emit(3, bw_estimate) {
                            let ts = now_ms();
                            let data = schema::generate_joint_state(ts);
                            let payload = bincode::serialize(&data).expect("fixed-size JointState");
                            emit_frame(&mut mux_track, &mut track_producers, 3, payload);
                        }
                    }
                    _ = ft_interval.tick() => {
                        if shed_state.should_emit(4, bw_estimate) {
                            let ts = now_ms();
                            let data = schema::generate_force_torque(ts);
                            let payload = bincode::serialize(&data).expect("fixed-size ForceTorque");
                            emit_frame(&mut mux_track, &mut track_producers, 4, payload);
                        }
                    }
                    _ = imu_interval.tick() => {
                        if shed_state.should_emit(5, bw_estimate) {
                            let ts = now_ms();
                            let data = schema::generate_imu(ts);
                            let payload = bincode::serialize(&data).expect("fixed-size ImuReading");
                            emit_frame(&mut mux_track, &mut track_producers, 5, payload);
                        }
                    }
                    _ = pointcloud_interval.tick() => {
                        if shed_state.should_emit(6, bw_estimate) {
                            let ts = now_ms();
                            let mut cloud = vec![0u8; 50_000];
                            cloud[..8].copy_from_slice(&ts.to_le_bytes());
                            emit_frame(&mut mux_track, &mut track_producers, 6, cloud);
                        }
                    }
                    _ = video_interval.tick() => {
                        if shed_state.should_emit(7, bw_estimate) {
                            let ts = now_ms();
                            let mut frame = vec![0u8; 50_000];
                            frame[..8].copy_from_slice(&ts.to_le_bytes());
                            emit_frame(&mut mux_track, &mut track_producers, 7, frame);
                        }
                    }
                    _ = video1_interval.tick() => {
                        if shed_state.should_emit(8, bw_estimate) {
                            let ts = now_ms();
                            let mut frame = vec![0u8; 50_000];
                            frame[..8].copy_from_slice(&ts.to_le_bytes());
                            emit_frame(&mut mux_track, &mut track_producers, 8, frame);
                        }
                    }
                    _ = video2_interval.tick() => {
                        if shed_state.should_emit(9, bw_estimate) {
                            let ts = now_ms();
                            let mut frame = vec![0u8; 50_000];
                            frame[..8].copy_from_slice(&ts.to_le_bytes());
                            emit_frame(&mut mux_track, &mut track_producers, 9, frame);
                        }
                    }
                }
            }
        } => Ok(()),
    }
}
