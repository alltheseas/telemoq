use anyhow::Context;
use moq_lite::*;
use url::Url;

use crate::schema::{self, TRACKS};

pub async fn run(client: moq_native::Client, url: &Url, broadcast_name: &str) -> anyhow::Result<()> {
    tracing::info!(broadcast = %broadcast_name, url = %url, "publishing telemetry");

    let origin = Origin::produce();
    let mut broadcast = Broadcast::produce();

    // Create a TrackProducer for each telemetry track
    let mut track_producers: Vec<TrackProducer> = Vec::new();
    for def in TRACKS {
        let track = broadcast.create_track(Track {
            name: def.name.to_string(),
            priority: def.priority,
        });
        tracing::info!(track = %def.name, priority = def.priority, rate_hz = def.rate_hz, "created track");
        track_producers.push(track);
    }

    origin.publish_broadcast(broadcast_name, broadcast.consume());

    let session = client
        .with_publish(origin.consume())
        .connect(url.clone())
        .await
        .context("failed to connect")?;

    let start = tokio::time::Instant::now();

    // Create interval timers for each track rate
    let mut estop_interval = tokio::time::interval(std::time::Duration::from_secs(1));
    let mut commands_interval = tokio::time::interval(std::time::Duration::from_millis(10));
    let mut joints_interval = tokio::time::interval(std::time::Duration::from_millis(10));
    let mut ft_interval = tokio::time::interval(std::time::Duration::from_millis(10));
    let mut imu_interval = tokio::time::interval(std::time::Duration::from_millis(5));
    let mut video_interval = tokio::time::interval(std::time::Duration::from_millis(33));

    tracing::info!("publishing started — all 6 tracks active");

    tokio::select! {
        res = session.closed() => res.context("session closed"),
        _ = async {
            loop {
                tokio::select! {
                    _ = estop_interval.tick() => {
                        let elapsed = start.elapsed().as_millis() as u64;
                        let data = schema::generate_estop(elapsed);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[0].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = commands_interval.tick() => {
                        let elapsed = start.elapsed().as_millis() as u64;
                        let data = schema::generate_command(elapsed);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[1].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = joints_interval.tick() => {
                        let elapsed = start.elapsed().as_millis() as u64;
                        let data = schema::generate_joint_state(elapsed);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[2].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = ft_interval.tick() => {
                        let elapsed = start.elapsed().as_millis() as u64;
                        let data = schema::generate_force_torque(elapsed);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[3].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = imu_interval.tick() => {
                        let elapsed = start.elapsed().as_millis() as u64;
                        let data = schema::generate_imu(elapsed);
                        let payload = bincode::serialize(&data).unwrap();
                        track_producers[4].write_frame(bytes::Bytes::from(payload));
                    }
                    _ = video_interval.tick() => {
                        let elapsed = start.elapsed().as_millis() as u64;
                        // 50KB fake video frame with embedded timestamp
                        let mut frame = vec![0u8; 50_000];
                        // Encode timestamp in first 8 bytes
                        frame[..8].copy_from_slice(&elapsed.to_le_bytes());
                        track_producers[5].write_frame(bytes::Bytes::from(frame));
                    }
                }
            }
        } => Ok(()),
    }
}
