use anyhow::Context;
use moq_lite::*;
use url::Url;

use crate::schema::{self, TRACKS};

/// Wall-clock timestamp in milliseconds since UNIX epoch.
/// Used instead of elapsed time so subscribers can compute real latency.
fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64
}

pub async fn run(client: moq_native::Client, url: &Url, broadcast_name: &str, no_priority: bool) -> anyhow::Result<()> {
    if no_priority {
        tracing::warn!("--no-priority: all tracks set to priority 0 (no starvation)");
    }
    tracing::info!(broadcast = %broadcast_name, url = %url, no_priority, "publishing telemetry");

    let origin = Origin::produce();
    let mut broadcast = Broadcast::produce();

    // Create a TrackProducer for each telemetry track
    let mut track_producers: Vec<TrackProducer> = Vec::new();
    for def in TRACKS {
        let priority = if no_priority { 0 } else { def.priority };
        let track = broadcast.create_track(Track {
            name: def.name.to_string(),
            priority,
        });
        tracing::info!(track = %def.name, priority, rate_hz = def.rate_hz, "created track");
        track_producers.push(track);
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

    tracing::info!("publishing started — all 8 tracks active (IHMC KST-aligned)");

    tokio::select! {
        res = session.closed() => res.context("session closed"),
        _ = async {
            loop {
                tokio::select! {
                    _ = heartbeat_interval.tick() => {
                        let ts = now_ms();
                        heartbeat_seq += 1;
                        let data = schema::generate_heartbeat(ts, heartbeat_seq);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[0].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = streaming_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_streaming_command(ts);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[1].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = task_status_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_task_status(ts);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[2].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = joints_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_joint_state(ts);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[3].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = ft_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_force_torque(ts);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[4].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = imu_interval.tick() => {
                        let ts = now_ms();
                        let data = schema::generate_imu(ts);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[5].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = pointcloud_interval.tick() => {
                        let ts = now_ms();
                        // 50KB downsampled point cloud with embedded timestamp
                        let mut cloud = vec![0u8; 50_000];
                        cloud[..8].copy_from_slice(&ts.to_le_bytes());
                        track_producers[6].write_frame(bytes::Bytes::from(cloud));
                    }
                    _ = video_interval.tick() => {
                        let ts = now_ms();
                        // 50KB video frame with embedded timestamp
                        let mut frame = vec![0u8; 50_000];
                        frame[..8].copy_from_slice(&ts.to_le_bytes());
                        track_producers[7].write_frame(bytes::Bytes::from(frame));
                    }
                }
            }
        } => Ok(()),
    }
}
