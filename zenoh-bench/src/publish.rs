use zenoh::pubsub::Publisher;
use zenoh::qos::{CongestionControl, Priority};
use zenoh::Session;

use crate::schema::{self, now_ms, TRACKS};

/// Map telemoq priority (u8) to Zenoh Priority enum.
fn map_priority(telemoq_pri: u8) -> Priority {
    match telemoq_pri {
        0 => Priority::RealTime,        // P0: heartbeat
        1 => Priority::InteractiveHigh,  // P1: control
        2 => Priority::InteractiveLow,   // P2: joints
        3 => Priority::DataHigh,         // P3: force/torque
        5 => Priority::Data,             // P5: imu
        10 => Priority::DataLow,         // P10: pointcloud
        20 => Priority::Background,      // P20: video
        _ => Priority::Data,
    }
}

/// Map telemoq priority to congestion control policy.
fn map_congestion_control(telemoq_pri: u8) -> CongestionControl {
    if telemoq_pri <= 5 {
        CongestionControl::Block
    } else {
        CongestionControl::Drop
    }
}

/// Generate payload for track index, returns (payload_bytes, track_index).
fn generate_payload(idx: usize, heartbeat_seq: &mut u64) -> Vec<u8> {
    let ts = now_ms();
    match idx {
        0 => {
            *heartbeat_seq += 1;
            bincode::serialize(&schema::generate_heartbeat(ts, *heartbeat_seq))
                .expect("fixed-size Heartbeat")
        }
        1 => bincode::serialize(&schema::generate_streaming_command(ts))
            .expect("fixed-size StreamingCommand"),
        2 => bincode::serialize(&schema::generate_task_status(ts))
            .expect("fixed-size TaskStatus"),
        3 => bincode::serialize(&schema::generate_joint_state(ts))
            .expect("fixed-size JointState"),
        4 => bincode::serialize(&schema::generate_force_torque(ts))
            .expect("fixed-size ForceTorque"),
        5 => bincode::serialize(&schema::generate_imu(ts))
            .expect("fixed-size ImuReading"),
        6 => {
            let mut cloud = vec![0u8; 50_000];
            cloud[..8].copy_from_slice(&ts.to_le_bytes());
            cloud
        }
        7 => {
            let mut frame = vec![0u8; 50_000];
            frame[..8].copy_from_slice(&ts.to_le_bytes());
            frame
        }
        _ => unreachable!(),
    }
}

pub async fn run(
    session: &Session,
    prefix: &str,
    flat: bool,
    single_key: bool,
    duration_secs: u64,
) -> anyhow::Result<()> {
    if single_key {
        tracing::warn!("--single-key: all tracks multiplexed into one Zenoh key (HOL simulation)");
    } else if flat {
        tracing::warn!("--flat: all tracks at same priority (no starvation)");
    }

    // Declare key expressions as owned strings that live long enough
    let mux_key: String = format!("{prefix}/multiplexed");
    let track_keys: Vec<String> = TRACKS.iter().map(|def| format!("{prefix}/{}", def.name)).collect();

    // Build publishers
    let mut publishers: Vec<Option<Publisher<'_>>> = Vec::with_capacity(TRACKS.len());
    let mut mux_publisher: Option<Publisher<'_>> = None;

    if single_key {
        mux_publisher = Some(
            session
                .declare_publisher(&mux_key)
                .priority(Priority::Data)
                .congestion_control(CongestionControl::Block)
                .await
                .map_err(|e| anyhow::anyhow!("{e}"))?,
        );
        tracing::info!("created single multiplexed key: {mux_key}");
    } else {
        for (i, def) in TRACKS.iter().enumerate() {
            let (priority, cc) = if flat {
                (Priority::Data, CongestionControl::Block)
            } else {
                (map_priority(def.priority), map_congestion_control(def.priority))
            };
            let pub_ = session
                .declare_publisher(&track_keys[i])
                .priority(priority)
                .congestion_control(cc)
                .await
                .map_err(|e| anyhow::anyhow!("{e}"))?;
            publishers.push(Some(pub_));
            tracing::info!(track = %def.name, ?priority, rate_hz = def.rate_hz, "created publisher");
        }
    }

    // Intervals — identical to telemoq
    let mut heartbeat_interval = tokio::time::interval(std::time::Duration::from_millis(100));   // 10 Hz
    let mut streaming_interval = tokio::time::interval(std::time::Duration::from_millis(6));     // ~167 Hz
    let mut task_status_interval = tokio::time::interval(std::time::Duration::from_millis(100)); // 10 Hz
    let mut joints_interval = tokio::time::interval(std::time::Duration::from_millis(10));       // 100 Hz
    let mut ft_interval = tokio::time::interval(std::time::Duration::from_millis(10));           // 100 Hz
    let mut imu_interval = tokio::time::interval(std::time::Duration::from_millis(5));           // 200 Hz
    let mut pointcloud_interval = tokio::time::interval(std::time::Duration::from_millis(200));  // 5 Hz
    let mut video_interval = tokio::time::interval(std::time::Duration::from_millis(33));        // ~30 Hz

    let mut heartbeat_seq: u64 = 0;
    let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(duration_secs);

    let mode = if single_key { "single-key" } else if flat { "flat" } else { "priority" };
    tracing::info!("publishing started — 8 tracks, mode={mode}, duration={duration_secs}s");

    loop {
        if tokio::time::Instant::now() >= deadline {
            tracing::info!("publish duration reached ({duration_secs}s), stopping");
            break;
        }

        // Macro to reduce repetition for each track
        macro_rules! emit {
            ($idx:expr) => {{
                let payload = generate_payload($idx, &mut heartbeat_seq);
                if let Some(ref mux) = mux_publisher {
                    let mut buf = Vec::with_capacity(1 + payload.len());
                    buf.push($idx as u8);
                    buf.extend_from_slice(&payload);
                    let _ = mux.put(buf).await;
                } else if let Some(ref pub_) = publishers[$idx] {
                    let _ = pub_.put(payload).await;
                }
            }};
        }

        tokio::select! {
            _ = heartbeat_interval.tick() => { emit!(0); }
            _ = streaming_interval.tick() => { emit!(1); }
            _ = task_status_interval.tick() => { emit!(2); }
            _ = joints_interval.tick() => { emit!(3); }
            _ = ft_interval.tick() => { emit!(4); }
            _ = imu_interval.tick() => { emit!(5); }
            _ = pointcloud_interval.tick() => { emit!(6); }
            _ = video_interval.tick() => { emit!(7); }
        }
    }

    Ok(())
}
