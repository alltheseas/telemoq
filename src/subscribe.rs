use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use anyhow::Context;
use moq_lite::*;
use url::Url;

use crate::schema::{self, TRACKS};

struct TrackStats {
    recv_count: AtomicU64,
    recv_bytes: AtomicU64,
    latency_sum_ms: AtomicU64,
    latency_count: AtomicU64,
    /// Max inter-arrival gap in ms (reset each display tick)
    max_gap_ms: AtomicU64,
    last_payload: std::sync::Mutex<Option<bytes::Bytes>>,
}

impl TrackStats {
    fn new() -> Self {
        Self {
            recv_count: AtomicU64::new(0),
            recv_bytes: AtomicU64::new(0),
            latency_sum_ms: AtomicU64::new(0),
            latency_count: AtomicU64::new(0),
            max_gap_ms: AtomicU64::new(0),
            last_payload: std::sync::Mutex::new(None),
        }
    }
}

/// Subscribe to all telemetry tracks and display live stats.
///
/// Connects to a MoQ relay, waits for a broadcast to appear, then subscribes
/// to each track (or a single multiplexed track in `--single-stream` mode).
/// Prints a live terminal dashboard or CSV output every second.
pub async fn run(
    client: moq_native::Client,
    url: &Url,
    broadcast_name: &str,
    csv: bool,
    no_priority: bool,
    single_stream: bool,
) -> anyhow::Result<()> {
    tracing::info!(broadcast = %broadcast_name, url = %url, "subscribing to telemetry");

    let origin = Origin::produce();

    let session = client
        .with_consume(origin.clone())
        .connect(url.clone())
        .await
        .context("failed to connect")?;

    let path: Path<'_> = broadcast_name.into();
    let mut origin_consumer = origin
        .consume_only(&[path])
        .context("not allowed to consume broadcast")?;

    // Wait for the broadcast to appear
    tracing::info!("waiting for broadcast '{broadcast_name}' to come online...");

    // Create stats counters for each track
    let stats: Vec<Arc<TrackStats>> = TRACKS.iter().map(|_| Arc::new(TrackStats::new())).collect();

    let start = tokio::time::Instant::now();

    // Spawn the display task
    let display_stats = stats.clone();
    let display_broadcast = broadcast_name.to_string();
    let display_url = url.to_string();
    let display_handle = tokio::spawn(async move {
        display_loop(&display_stats, &display_broadcast, &display_url, start, csv, no_priority, single_stream).await;
    });

    // Main event loop: wait for broadcasts, subscribe to tracks
    loop {
        tokio::select! {
            Some(announce) = origin_consumer.announced() => {
                match announce {
                    (path, Some(broadcast)) => {
                        tracing::info!(broadcast = %path, "broadcast online — subscribing to tracks");

                        if single_stream {
                            // Single-stream mode: subscribe to one "multiplexed" track, demux by prefix byte
                            let track_info = Track {
                                name: "multiplexed".to_string(),
                                priority: 0,
                            };
                            let consumer = broadcast.subscribe_track(&track_info)?;
                            let all_stats: Vec<Arc<TrackStats>> = stats.iter().map(Arc::clone).collect();
                            tokio::spawn(async move {
                                if let Err(e) = read_muxed_track(consumer, &all_stats).await {
                                    tracing::warn!(error = %e, "multiplexed track reader ended");
                                }
                            });
                        } else {
                            // Normal mode: subscribe to each track individually
                            for (i, def) in TRACKS.iter().enumerate() {
                                let track_info = Track {
                                    name: def.name.to_string(),
                                    priority: def.priority,
                                };
                                let consumer = broadcast.subscribe_track(&track_info)?;
                                let track_stats = stats[i].clone();
                                let track_name = def.name;

                                tokio::spawn(async move {
                                    if let Err(e) = read_track(consumer, track_stats, track_name).await {
                                        tracing::warn!(track = %track_name, error = %e, "track reader ended");
                                    }
                                });
                            }
                        }
                    }
                    (path, None) => {
                        tracing::warn!(broadcast = %path, "broadcast offline — waiting...");
                    }
                }
            }
            res = session.closed() => {
                display_handle.abort();
                return res.context("session closed");
            }
        }
    }
}

/// Tracks that use raw byte blobs with LE u64 timestamp in first 8 bytes
/// (not bincode-serialized structs)
fn is_raw_blob_track(name: &str) -> bool {
    matches!(name, "video/camera0" | "perception/pointcloud")
}

async fn read_track(
    mut consumer: TrackConsumer,
    stats: Arc<TrackStats>,
    track_name: &str,
) -> anyhow::Result<()> {
    let mut last_recv_time: Option<u64> = None;

    while let Some(mut group) = consumer.next_group().await? {
        while let Some(frame) = group.read_frame().await? {
            let recv_time = schema::now_ms();
            stats.recv_count.fetch_add(1, Ordering::Relaxed);
            stats.recv_bytes.fetch_add(frame.len() as u64, Ordering::Relaxed);

            // Track inter-arrival gap
            if let Some(prev) = last_recv_time {
                let gap = recv_time.saturating_sub(prev);
                // Atomic max: keep retrying until we succeed or see a larger value
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
            last_recv_time = Some(recv_time);

            // Extract wall-clock timestamp and compute latency
            if frame.len() >= 8 {
                let ts = if is_raw_blob_track(track_name) {
                    u64::from_le_bytes(frame[..8].try_into().unwrap())
                } else {
                    bincode::deserialize::<u64>(&frame[..8]).unwrap_or(0)
                };

                if ts > 0 && recv_time > ts {
                    let latency = recv_time - ts;
                    stats.latency_sum_ms.fetch_add(latency, Ordering::Relaxed);
                    stats.latency_count.fetch_add(1, Ordering::Relaxed);
                }
            }

            *stats.last_payload.lock().unwrap() = Some(frame);
        }
    }

    Ok(())
}

/// Read from the single multiplexed track, demux by 1-byte track index prefix.
async fn read_muxed_track(
    mut consumer: TrackConsumer,
    stats: &[Arc<TrackStats>],
) -> anyhow::Result<()> {
    let mut last_recv_times: Vec<Option<u64>> = vec![None; TRACKS.len()];

    while let Some(mut group) = consumer.next_group().await? {
        while let Some(frame) = group.read_frame().await? {
            if frame.is_empty() {
                continue;
            }

            let track_idx = frame[0] as usize;
            if track_idx >= stats.len() || track_idx >= TRACKS.len() {
                continue;
            }

            let payload = frame.slice(1..); // strip the 1-byte prefix
            let recv_time = schema::now_ms();
            let stat = &stats[track_idx];

            stat.recv_count.fetch_add(1, Ordering::Relaxed);
            stat.recv_bytes.fetch_add(payload.len() as u64, Ordering::Relaxed);

            // Track inter-arrival gap per original track
            if let Some(prev) = last_recv_times[track_idx] {
                let gap = recv_time.saturating_sub(prev);
                let mut current = stat.max_gap_ms.load(Ordering::Relaxed);
                while gap > current {
                    match stat.max_gap_ms.compare_exchange_weak(
                        current, gap, Ordering::Relaxed, Ordering::Relaxed,
                    ) {
                        Ok(_) => break,
                        Err(v) => current = v,
                    }
                }
            }
            last_recv_times[track_idx] = Some(recv_time);

            // Extract timestamp and compute latency
            if payload.len() >= 8 {
                let track_name = TRACKS[track_idx].name;
                let ts = if is_raw_blob_track(track_name) {
                    u64::from_le_bytes(payload[..8].try_into().unwrap())
                } else {
                    bincode::deserialize::<u64>(&payload[..8]).unwrap_or(0)
                };

                if ts > 0 && recv_time > ts {
                    let latency = recv_time - ts;
                    stat.latency_sum_ms.fetch_add(latency, Ordering::Relaxed);
                    stat.latency_count.fetch_add(1, Ordering::Relaxed);
                }
            }

            *stat.last_payload.lock().unwrap() = Some(payload);
        }
    }

    Ok(())
}

async fn display_loop(
    stats: &[Arc<TrackStats>],
    broadcast_name: &str,
    url: &str,
    start: tokio::time::Instant,
    csv: bool,
    no_priority: bool,
    single_stream: bool,
) {
    let mode = if single_stream { "single-stream" } else if no_priority { "no-priority" } else { "priority" };
    if csv {
        println!("elapsed_s,track,priority,recv_per_s,expected_per_s,pct,bytes_per_s,avg_latency_ms,max_gap_ms,mode");
    }

    let mut interval = tokio::time::interval(std::time::Duration::from_secs(1));
    interval.tick().await; // skip first tick

    loop {
        interval.tick().await;
        let elapsed = start.elapsed().as_secs();

        if csv {
            for (i, def) in TRACKS.iter().enumerate() {
                let count = stats[i].recv_count.swap(0, Ordering::Relaxed);
                let bytes = stats[i].recv_bytes.swap(0, Ordering::Relaxed);
                let lat_sum = stats[i].latency_sum_ms.swap(0, Ordering::Relaxed);
                let lat_count = stats[i].latency_count.swap(0, Ordering::Relaxed);
                let gap = stats[i].max_gap_ms.swap(0, Ordering::Relaxed);
                let effective_pri = if no_priority { 0 } else { def.priority };
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
            continue;
        }

        // ANSI terminal display
        print!("\x1B[H\x1B[J"); // cursor home + clear
        let mode_display = if single_stream {
            "\x1B[31m SINGLE-STREAM \x1B[0m"
        } else if no_priority {
            "\x1B[33m NO-PRIORITY \x1B[0m"
        } else {
            "\x1B[32m PRIORITY \x1B[0m"
        };
        println!(
            "  telemoq | {} | relay: {} | uptime: {}s | mode: {}",
            broadcast_name, url, elapsed, mode_display
        );
        println!("  ╔══════════════════════╤═════╤═════════╤══════════╤════════════╤═════════╤═══════╤═══════════════════════╗");
        println!("  ║ Track                │ Pri │ Recv/s  │ Expected │   Status   │ Latency │  Gap  │ Throughput            ║");
        println!("  ╟──────────────────────┼─────┼─────────┼──────────┼────────────┼─────────┼───────┼───────────────────────╢");

        for (i, def) in TRACKS.iter().enumerate() {
            let count = stats[i].recv_count.swap(0, Ordering::Relaxed);
            let bytes = stats[i].recv_bytes.swap(0, Ordering::Relaxed);
            let lat_sum = stats[i].latency_sum_ms.swap(0, Ordering::Relaxed);
            let lat_count = stats[i].latency_count.swap(0, Ordering::Relaxed);
            let gap = stats[i].max_gap_ms.swap(0, Ordering::Relaxed);
            let expected = def.rate_hz as u64;
            let pct = if expected > 0 {
                (count * 100).checked_div(expected).unwrap_or(0)
            } else {
                100
            };

            let status = if pct >= 80 { "\x1B[32m  OK  \x1B[0m" } else { "\x1B[31mDEGRAD\x1B[0m" };

            let avg_lat = if lat_count > 0 { lat_sum / lat_count } else { 0 };
            let lat_str = format!("{:>4}ms", avg_lat);
            // Color latency: green <10ms, yellow 10-50ms, red >50ms
            let lat_colored = if avg_lat < 10 {
                format!("\x1B[32m{lat_str}\x1B[0m")
            } else if avg_lat < 50 {
                format!("\x1B[33m{lat_str}\x1B[0m")
            } else {
                format!("\x1B[31m{lat_str}\x1B[0m")
            };

            // Color gap: green if within 12ms extrapolation window, red if exceeding
            let gap_str = format!("{:>3}ms", gap);
            let gap_colored = if gap <= 12 {
                format!("\x1B[32m{gap_str}\x1B[0m")
            } else {
                format!("\x1B[31m{gap_str}\x1B[0m")
            };

            // Bar chart (20 chars wide)
            let bar_len = std::cmp::min((pct as usize) / 5, 20);
            let bar: String = "█".repeat(bar_len) + &"░".repeat(20 - bar_len);

            let throughput = format_bytes(bytes);

            let effective_pri = if no_priority { 0 } else { def.priority };
            println!(
                "  ║ {:<20} │ P{:<2} │ {:>5}/s │ {:>6}/s │   {}   │ {} │ {} │ {} {:>9} ║",
                def.label, effective_pri, count, expected, status, lat_colored, gap_colored, bar, throughput
            );

            // Show latest data for interesting tracks
            if let Some(payload) = stats[i].last_payload.lock().unwrap().as_ref() {
                if def.name == "safety/heartbeat" && payload.len() >= 17 {
                    if let Ok(hb) = bincode::deserialize::<crate::schema::Heartbeat>(payload) {
                        let estop = if hb.estop_engaged { "\x1B[31mENGAGED\x1B[0m" } else { "ok" };
                        println!(
                            "  ║   └─ seq: {} estop: {} {:>65} ║",
                            hb.sequence, estop, ""
                        );
                    }
                }
                if def.name == "control/task_status" && payload.len() >= 14 {
                    if let Ok(ts) = bincode::deserialize::<crate::schema::TaskStatus>(payload) {
                        let state_str = match ts.state {
                            0 => "idle",
                            1 => "executing",
                            2 => "complete",
                            3 => "failed",
                            _ => "?",
                        };
                        println!(
                            "  ║   └─ task #{}: {} ({}%) {:>58} ║",
                            ts.task_id, state_str, ts.progress_pct, ""
                        );
                    }
                }
                if def.name == "sensors/joints" && payload.len() > 8 {
                    if let Ok(js) = bincode::deserialize::<crate::schema::JointState>(payload) {
                        let angles: Vec<String> =
                            js.positions.iter().map(|p| format!("{:.2}", p)).collect();
                        println!(
                            "  ║   └─ joints: [{}] rad {:>30} ║",
                            angles.join(", "),
                            ""
                        );
                    }
                }
                if def.name == "sensors/force_torque" && payload.len() > 8 {
                    if let Ok(ft) = bincode::deserialize::<crate::schema::ForceTorque>(payload) {
                        let contact = if ft.force[2].abs() > 3.0 { "YES" } else { "no" };
                        println!(
                            "  ║   └─ F=[{:.1}, {:.1}, {:.1}]N  weld contact: {} {:>39} ║",
                            ft.force[0], ft.force[1], ft.force[2], contact, ""
                        );
                    }
                }
            }
        }

        println!("  ╟──────────────────────┴─────┴─────────┴──────────┴────────────┴─────────┴───────┴───────────────────────╢");
        println!("  ║  Latency = wall-clock delta (pub→relay→sub).  Gap = max inter-arrival time (12ms = IHMC KST window).  ║");
        println!("  ║                                                                                                        ║");
        println!("  ║  IHMC DRC lesson: 9,600 bps was enough for control. Video is expendable.                               ║");
        println!("  ║  TCP/WebRTC: One lost video packet  →  ALL streams freeze (HOL blocking)                               ║");
        println!("  ║  MoQ/QUIC:   Video degrades         →  Robot stays under control (priority streams)                    ║");
        println!("  ╚════════════════════════════════════════════════════════════════════════════════════════════════════════╝");
    }
}

fn format_bytes(bytes: u64) -> String {
    if bytes >= 1_000_000 {
        format!("{:.1} MB/s", bytes as f64 / 1_000_000.0)
    } else if bytes >= 1_000 {
        format!("{:.1} KB/s", bytes as f64 / 1_000.0)
    } else {
        format!("{} B/s", bytes)
    }
}
