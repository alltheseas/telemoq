use anyhow::Context;
use moq_lite::*;
use url::Url;

use crate::schema::{self, now_ms, TRACKS};

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

/// Publish all 8 telemetry tracks at their configured rates.
///
/// Connects to a MoQ relay, creates one QUIC stream per track (or a single
/// multiplexed stream in `--single-stream` mode), and loops forever generating
/// synthetic telemetry at IHMC KST-aligned rates.
pub async fn run(client: moq_native::Client, url: &Url, broadcast_name: &str, no_priority: bool, single_stream: bool) -> anyhow::Result<()> {
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
        });
        tracing::info!("created single multiplexed track (WebRTC HOL simulation)");
        mux_track = Some(track);
    } else {
        for def in TRACKS {
            let priority = if no_priority { 0 } else { def.priority };
            let track = broadcast.create_track(Track {
                name: def.name.to_string(),
                priority,
            });
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

    // Track indices (must match TRACKS order in schema.rs):
    // 0: safety/heartbeat      P0   10 Hz
    // 1: control/streaming     P1  167 Hz
    // 2: control/task_status   P1   10 Hz
    // 3: sensors/joints        P2  100 Hz
    // 4: sensors/force_torque  P3  100 Hz
    // 5: sensors/imu           P5  200 Hz
    // 6: perception/pointcloud P10   5 Hz
    // 7: video/camera0         P20  30 Hz

    let mut heartbeat_interval = tokio::time::interval(std::time::Duration::from_millis(100));   // 10 Hz
    let mut streaming_interval = tokio::time::interval(std::time::Duration::from_millis(6));     // ~167 Hz
    let mut task_status_interval = tokio::time::interval(std::time::Duration::from_millis(100)); // 10 Hz
    let mut joints_interval = tokio::time::interval(std::time::Duration::from_millis(10));       // 100 Hz
    let mut ft_interval = tokio::time::interval(std::time::Duration::from_millis(10));           // 100 Hz
    let mut imu_interval = tokio::time::interval(std::time::Duration::from_millis(5));           // 200 Hz
    let mut pointcloud_interval = tokio::time::interval(std::time::Duration::from_millis(200));  // 5 Hz
    let mut video_interval = tokio::time::interval(std::time::Duration::from_millis(33));        // ~30 Hz

    let mut heartbeat_seq: u64 = 0;

    let mode = if single_stream { "single-stream" } else if no_priority { "no-priority" } else { "priority" };
    tracing::info!("publishing started — all 8 tracks active (IHMC KST-aligned), mode={mode}");

    tokio::select! {
        res = session.closed() => res.context("session closed"),
        _ = async {
            loop {
                tokio::select! {
                    _ = heartbeat_interval.tick() => {
                        let ts = now_ms();
                        heartbeat_seq += 1;
                        let data = schema::generate_heartbeat(ts, heartbeat_seq);
                        let payload = bincode::serialize(&data).expect("fixed-size Heartbeat");
                        emit_frame(&mut mux_track, &mut track_producers, 0, payload);
                    }
                    _ = streaming_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_streaming_command(ts);
                        let payload = bincode::serialize(&data).expect("fixed-size StreamingCommand");
                        emit_frame(&mut mux_track, &mut track_producers, 1, payload);
                    }
                    _ = task_status_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_task_status(ts);
                        let payload = bincode::serialize(&data).expect("fixed-size TaskStatus");
                        emit_frame(&mut mux_track, &mut track_producers, 2, payload);
                    }
                    _ = joints_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_joint_state(ts);
                        let payload = bincode::serialize(&data).expect("fixed-size JointState");
                        emit_frame(&mut mux_track, &mut track_producers, 3, payload);
                    }
                    _ = ft_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_force_torque(ts);
                        let payload = bincode::serialize(&data).expect("fixed-size ForceTorque");
                        emit_frame(&mut mux_track, &mut track_producers, 4, payload);
                    }
                    _ = imu_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_imu(ts);
                        let payload = bincode::serialize(&data).expect("fixed-size ImuReading");
                        emit_frame(&mut mux_track, &mut track_producers, 5, payload);
                    }
                    _ = pointcloud_interval.tick() => {
                        let ts = now_ms();
                        let mut cloud = vec![0u8; 50_000];
                        cloud[..8].copy_from_slice(&ts.to_le_bytes());
                        emit_frame(&mut mux_track, &mut track_producers, 6, cloud);
                    }
                    _ = video_interval.tick() => {
                        let ts = now_ms();
                        let mut frame = vec![0u8; 50_000];
                        frame[..8].copy_from_slice(&ts.to_le_bytes());
                        emit_frame(&mut mux_track, &mut track_producers, 7, frame);
                    }
                }
            }
        } => Ok(()),
    }
}
