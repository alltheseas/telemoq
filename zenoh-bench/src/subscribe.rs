use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use zenoh::Session;

use crate::schema::{self, TRACKS};

struct TrackStats {
    recv_count: AtomicU64,
    recv_bytes: AtomicU64,
    latency_sum_ms: AtomicU64,
    latency_count: AtomicU64,
    max_gap_ms: AtomicU64,
    last_recv_ms: AtomicU64,
}

impl TrackStats {
    fn new() -> Self {
        Self {
            recv_count: AtomicU64::new(0),
            recv_bytes: AtomicU64::new(0),
            latency_sum_ms: AtomicU64::new(0),
            latency_count: AtomicU64::new(0),
            max_gap_ms: AtomicU64::new(0),
            last_recv_ms: AtomicU64::new(0),
        }
    }
}

fn is_raw_blob_track(name: &str) -> bool {
    matches!(name, "perception/pointcloud" | "video/camera0")
}

fn process_payload(stats: &TrackStats, payload: &[u8], track_name: &str) {
    let recv_time = schema::now_ms();
    stats.recv_count.fetch_add(1, Ordering::Relaxed);
    stats.recv_bytes.fetch_add(payload.len() as u64, Ordering::Relaxed);

    // Inter-arrival gap
    let prev = stats.last_recv_ms.swap(recv_time, Ordering::Relaxed);
    if prev > 0 {
        let gap = recv_time.saturating_sub(prev);
        let mut current = stats.max_gap_ms.load(Ordering::Relaxed);
        while gap > current {
            match stats.max_gap_ms.compare_exchange_weak(
                current, gap, Ordering::Relaxed, Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(v) => current = v,
            }
        }
    }

    // Extract timestamp and compute latency
    if payload.len() >= 8 {
        let ts = if is_raw_blob_track(track_name) {
            u64::from_le_bytes(payload[..8].try_into().unwrap())
        } else {
            bincode::deserialize::<u64>(&payload[..8]).unwrap_or(0)
        };
        if ts > 0 && recv_time > ts {
            let latency = recv_time - ts;
            stats.latency_sum_ms.fetch_add(latency, Ordering::Relaxed);
            stats.latency_count.fetch_add(1, Ordering::Relaxed);
        }
    }
}

pub async fn run(
    session: &Session,
    prefix: &str,
    flat: bool,
    single_key: bool,
    duration_secs: u64,
) -> anyhow::Result<()> {
    let stats: Vec<Arc<TrackStats>> = TRACKS.iter().map(|_| Arc::new(TrackStats::new())).collect();
    let start = tokio::time::Instant::now();

    // Subscribe
    if single_key {
        let key = format!("{prefix}/multiplexed");
        let subscriber = session.declare_subscriber(&key).await.map_err(|e| anyhow::anyhow!("{e}"))?;
        let all_stats: Vec<Arc<TrackStats>> = stats.iter().map(Arc::clone).collect();
        tokio::spawn(async move {
            loop {
                match subscriber.recv_async().await {
                    Ok(sample) => {
                        let payload = sample.payload().to_bytes();
                        if payload.is_empty() { continue; }
                        let idx = payload[0] as usize;
                        if idx >= TRACKS.len() { continue; }
                        process_payload(&all_stats[idx], &payload[1..], TRACKS[idx].name);
                    }
                    Err(_) => break,
                }
            }
        });
    } else {
        for (i, def) in TRACKS.iter().enumerate() {
            let key = format!("{prefix}/{}", def.name);
            let subscriber = session.declare_subscriber(&key).await.map_err(|e| anyhow::anyhow!("{e}"))?;
            let track_stats = stats[i].clone();
            let track_name = def.name;
            tokio::spawn(async move {
                loop {
                    match subscriber.recv_async().await {
                        Ok(sample) => {
                            let payload = sample.payload().to_bytes();
                            process_payload(&track_stats, &payload, track_name);
                        }
                        Err(_) => break,
                    }
                }
            });
        }
    }

    // CSV header — identical format to telemoq
    let mode = if single_key { "single-key" } else if flat { "flat" } else { "priority" };
    println!("elapsed_s,track,priority,recv_per_s,expected_per_s,pct,bytes_per_s,avg_latency_ms,max_gap_ms,mode");

    let mut interval = tokio::time::interval(std::time::Duration::from_secs(1));
    interval.tick().await; // skip first tick

    let deadline = start + std::time::Duration::from_secs(duration_secs);

    loop {
        interval.tick().await;
        let elapsed = start.elapsed().as_secs();

        if tokio::time::Instant::now() >= deadline {
            break;
        }

        for (i, def) in TRACKS.iter().enumerate() {
            let count = stats[i].recv_count.swap(0, Ordering::Relaxed);
            let bytes = stats[i].recv_bytes.swap(0, Ordering::Relaxed);
            let lat_sum = stats[i].latency_sum_ms.swap(0, Ordering::Relaxed);
            let lat_count = stats[i].latency_count.swap(0, Ordering::Relaxed);
            let gap = stats[i].max_gap_ms.swap(0, Ordering::Relaxed);
            let effective_pri = if flat { 0 } else { def.priority };
            let pct = if def.rate_hz > 0 {
                count * 100 / def.rate_hz as u64
            } else {
                0
            };
            let avg_lat = if lat_count > 0 { lat_sum / lat_count } else { 0 };
            println!(
                "{elapsed},{},{},{count},{},{pct},{bytes},{avg_lat},{gap},{mode}",
                def.name, effective_pri, def.rate_hz
            );
        }
    }

    Ok(())
}
