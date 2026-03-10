use std::path::Path;

use anyhow::Context;
use moq_lite::*;
use url::Url;

use crate::replay::ReplayReader;
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
        let _ = mux.write_frame(bytes::Bytes::from(buf));
    } else {
        let _ = track_producers[idx].write_frame(bytes::Bytes::from(payload));
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

/// Publish DROID dataset replay through MoQ tracks.
///
/// Reads pre-processed replay data at native 15Hz, stamps wall-clock timestamps,
/// and emits joints + actions + camera JPEG frames. Heartbeat and task_status
/// remain synthetic (DROID dataset doesn't include these).
pub async fn run_replay(
    client: moq_native::Client,
    url: &Url,
    broadcast_name: &str,
    replay_path: &Path,
    all_cameras: bool,
    loop_replay: bool,
    no_priority: bool,
    single_stream: bool,
) -> anyhow::Result<()> {
    let tracks = if all_cameras {
        schema::REPLAY_TRACKS_ALL_CAMERAS
    } else {
        schema::REPLAY_TRACKS
    };

    if single_stream {
        tracing::warn!("--single-stream: all tracks multiplexed into one QUIC stream (WebRTC-style HOL blocking)");
    } else if no_priority {
        tracing::warn!("--no-priority: all tracks set to priority 0 (no starvation)");
    }
    tracing::info!(
        broadcast = %broadcast_name,
        url = %url,
        replay = %replay_path.display(),
        all_cameras,
        "publishing DROID replay"
    );

    let mut reader = ReplayReader::open(replay_path, all_cameras, loop_replay)?;

    let origin = Origin::produce();
    let mut broadcast = Broadcast::produce();

    let mut mux_track: Option<TrackProducer> = None;
    let mut track_producers: Vec<TrackProducer> = Vec::new();

    if single_stream {
        let track = broadcast.create_track(Track {
            name: "multiplexed".to_string(),
            priority: 0,
        })?;
        mux_track = Some(track);
    } else {
        for def in tracks {
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

    // Replay track indices (must match REPLAY_TRACKS / REPLAY_TRACKS_ALL_CAMERAS order):
    // 0: safety/heartbeat      P0   10 Hz  (synthetic)
    // 1: control/streaming     P1   15 Hz  (DROID action)
    // 2: control/task_status   P1   10 Hz  (synthetic)
    // 3: sensors/joints        P2   15 Hz  (DROID state)
    // 4: video/camera0         P20  15 Hz  (DROID wrist camera)
    // 5: video/camera1         P20  15 Hz  (DROID exterior 1, if --all-cameras)
    // 6: video/camera2         P20  15 Hz  (DROID exterior 2, if --all-cameras)
    const IDX_HEARTBEAT: usize = 0;
    const IDX_ACTION: usize = 1;
    const IDX_TASK_STATUS: usize = 2;
    const IDX_JOINTS: usize = 3;
    const IDX_CAMERA0: usize = 4;
    const IDX_CAMERA1: usize = 5;
    const IDX_CAMERA2: usize = 6;

    // 15 Hz replay tick = 66.667ms
    let mut replay_interval = tokio::time::interval(std::time::Duration::from_nanos(66_666_667));
    let mut heartbeat_interval = tokio::time::interval(std::time::Duration::from_millis(100)); // 10 Hz
    let mut task_status_interval = tokio::time::interval(std::time::Duration::from_millis(100)); // 10 Hz

    let mut heartbeat_seq: u64 = 0;

    let mode = if single_stream { "single-stream" } else if no_priority { "no-priority" } else { "priority" };
    tracing::info!("DROID replay started — mode={mode}, fps={}", reader.meta().fps);

    tokio::select! {
        res = session.closed() => res.context("session closed"),
        _ = async {
            loop {
                tokio::select! {
                    _ = replay_interval.tick() => {
                        let step = match reader.next_step() {
                            Some(s) => s,
                            None => {
                                tracing::info!("replay finished");
                                return;
                            }
                        };
                        let ts = now_ms();

                        // Emit joints (DROID observation.state)
                        let joints = schema::DroidJointState {
                            timestamp_ms: ts,
                            positions: step.telemetry.state,
                        };
                        let payload = bincode::serialize(&joints).expect("DroidJointState");
                        emit_frame(&mut mux_track, &mut track_producers, IDX_JOINTS, payload);

                        // Emit action (DROID action as control/streaming)
                        let action = schema::DroidAction {
                            timestamp_ms: ts,
                            commands: step.telemetry.action,
                        };
                        let payload = bincode::serialize(&action).expect("DroidAction");
                        emit_frame(&mut mux_track, &mut track_producers, IDX_ACTION, payload);

                        // Emit camera0 (wrist): 8-byte LE timestamp + raw JPEG
                        if !step.camera0_jpeg.is_empty() {
                            let mut frame = Vec::with_capacity(8 + step.camera0_jpeg.len());
                            frame.extend_from_slice(&ts.to_le_bytes());
                            frame.extend_from_slice(&step.camera0_jpeg);
                            emit_frame(&mut mux_track, &mut track_producers, IDX_CAMERA0, frame);
                        }

                        // Emit camera1 (exterior 1)
                        if let Some(ref jpeg) = step.camera1_jpeg {
                            if !jpeg.is_empty() {
                                let mut frame = Vec::with_capacity(8 + jpeg.len());
                                frame.extend_from_slice(&ts.to_le_bytes());
                                frame.extend_from_slice(jpeg);
                                emit_frame(&mut mux_track, &mut track_producers, IDX_CAMERA1, frame);
                            }
                        }

                        // Emit camera2 (exterior 2)
                        if let Some(ref jpeg) = step.camera2_jpeg {
                            if !jpeg.is_empty() {
                                let mut frame = Vec::with_capacity(8 + jpeg.len());
                                frame.extend_from_slice(&ts.to_le_bytes());
                                frame.extend_from_slice(jpeg);
                                emit_frame(&mut mux_track, &mut track_producers, IDX_CAMERA2, frame);
                            }
                        }
                    }
                    _ = heartbeat_interval.tick() => {
                        let ts = now_ms();
                        heartbeat_seq += 1;
                        let data = schema::generate_heartbeat(ts, heartbeat_seq);
                        let payload = bincode::serialize(&data).expect("Heartbeat");
                        emit_frame(&mut mux_track, &mut track_producers, IDX_HEARTBEAT, payload);
                    }
                    _ = task_status_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_task_status(ts);
                        let payload = bincode::serialize(&data).expect("TaskStatus");
                        emit_frame(&mut mux_track, &mut track_producers, IDX_TASK_STATUS, payload);
                    }
                }
            }
        } => Ok(()),
    }
}
