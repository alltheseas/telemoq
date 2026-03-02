use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use anyhow::Context;
use moq_lite::*;
use url::Url;

use crate::schema::TRACKS;

struct TrackStats {
    recv_count: AtomicU64,
    recv_bytes: AtomicU64,
    _latency_sum_ms: AtomicU64,
    _latency_count: AtomicU64,
    last_payload: std::sync::Mutex<Option<bytes::Bytes>>,
}

impl TrackStats {
    fn new() -> Self {
        Self {
            recv_count: AtomicU64::new(0),
            recv_bytes: AtomicU64::new(0),
            _latency_sum_ms: AtomicU64::new(0),
            _latency_count: AtomicU64::new(0),
            last_payload: std::sync::Mutex::new(None),
        }
    }
}

pub async fn run(
    client: moq_native::Client,
    url: &Url,
    broadcast_name: &str,
    csv: bool,
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
        display_loop(&display_stats, &display_broadcast, &display_url, start, csv).await;
    });

    // Main event loop: wait for broadcasts, subscribe to tracks
    loop {
        tokio::select! {
            Some(announce) = origin_consumer.announced() => {
                match announce {
                    (path, Some(broadcast)) => {
                        tracing::info!(broadcast = %path, "broadcast online — subscribing to tracks");

                        // Subscribe to all tracks
                        for (i, def) in TRACKS.iter().enumerate() {
                            let track_info = Track {
                                name: def.name.to_string(),
                                priority: def.priority,
                            };
                            let consumer = broadcast.subscribe_track(&track_info);
                            let track_stats = stats[i].clone();
                            let track_name = def.name;

                            tokio::spawn(async move {
                                if let Err(e) = read_track(consumer, track_stats, track_name).await {
                                    tracing::warn!(track = %track_name, error = %e, "track reader ended");
                                }
                            });
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

async fn read_track(
    mut consumer: TrackConsumer,
    stats: Arc<TrackStats>,
    track_name: &str,
) -> anyhow::Result<()> {
    let now_ms = || {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64
    };

    while let Some(mut group) = consumer.next_group().await? {
        while let Some(frame) = group.read_frame().await? {
            stats.recv_count.fetch_add(1, Ordering::Relaxed);
            stats.recv_bytes.fetch_add(frame.len() as u64, Ordering::Relaxed);

            // Try to extract timestamp from bincode-serialized payload
            // All our structs start with timestamp_ms: u64
            if frame.len() >= 8 {
                let ts = if track_name == "video/camera0" {
                    // Video: timestamp is raw LE u64 in first 8 bytes
                    u64::from_le_bytes(frame[..8].try_into().unwrap())
                } else {
                    // Bincode: first 8 bytes are the u64 timestamp
                    bincode::deserialize::<u64>(&frame[..8]).unwrap_or(0)
                };

                if ts > 0 {
                    // ts is elapsed_ms from publisher start, not wall clock
                    // For latency, we use current wall time - publisher wall time
                    // Since both are on localhost, we just note the timestamp
                    let current = now_ms();
                    if current > ts {
                        // This won't be useful for elapsed-based timestamps
                        // but works for wall-clock timestamps
                    }
                }
            }

            *stats.last_payload.lock().unwrap() = Some(frame);
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
) {
    if csv {
        println!("elapsed_s,track,priority,recv_per_s,expected_per_s,pct,bytes_per_s");
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
                let pct = if def.rate_hz > 0 {
                    count * 100 / def.rate_hz as u64
                } else {
                    0
                };
                println!(
                    "{elapsed},{},{},{count},{},{pct},{bytes}",
                    def.name, def.priority, def.rate_hz
                );
            }
            continue;
        }

        // ANSI terminal display
        print!("\x1B[H\x1B[J"); // cursor home + clear
        println!(
            "  telemoq | {} | relay: {} | uptime: {}s",
            broadcast_name, url, elapsed
        );
        println!("  ╔══════════════════════╤═════╤═════════╤══════════╤════════════╤═══════════════════════╗");
        println!("  ║ Track                │ Pri │ Recv/s  │ Expected │   Status   │ Throughput            ║");
        println!("  ╟──────────────────────┼─────┼─────────┼──────────┼────────────┼───────────────────────╢");

        for (i, def) in TRACKS.iter().enumerate() {
            let count = stats[i].recv_count.swap(0, Ordering::Relaxed);
            let bytes = stats[i].recv_bytes.swap(0, Ordering::Relaxed);
            let expected = def.rate_hz as u64;
            let pct = if expected > 0 {
                (count * 100).checked_div(expected).unwrap_or(0)
            } else {
                100
            };

            let status = if pct >= 80 { "\x1B[32m  OK  \x1B[0m" } else { "\x1B[31mDEGRAD\x1B[0m" };

            // Bar chart (20 chars wide)
            let bar_len = std::cmp::min((pct as usize) / 5, 20);
            let bar: String = "█".repeat(bar_len) + &"░".repeat(20 - bar_len);

            let throughput = format_bytes(bytes);

            println!(
                "  ║ {:<20} │ P{:<2} │ {:>5}/s │ {:>6}/s │   {}   │ {} {:>9} ║",
                def.label, def.priority, count, expected, status, bar, throughput
            );

            // Show latest data for interesting tracks
            if let Some(payload) = stats[i].last_payload.lock().unwrap().as_ref() {
                if def.name == "sensors/joints" && payload.len() > 8 {
                    if let Ok(js) = bincode::deserialize::<crate::schema::JointState>(payload) {
                        let angles: Vec<String> =
                            js.positions.iter().map(|p| format!("{:.2}", p)).collect();
                        println!(
                            "  ║   └─ joints: [{}] rad {:>14} ║",
                            angles.join(", "),
                            ""
                        );
                    }
                }
                if def.name == "sensors/force_torque" && payload.len() > 8 {
                    if let Ok(ft) = bincode::deserialize::<crate::schema::ForceTorque>(payload) {
                        let contact = if ft.force[2].abs() > 3.0 { "YES" } else { "no" };
                        println!(
                            "  ║   └─ F=[{:.1}, {:.1}, {:.1}]N  weld contact: {} {:>23} ║",
                            ft.force[0], ft.force[1], ft.force[2], contact, ""
                        );
                    }
                }
            }
        }

        println!("  ╟──────────────────────┴─────┴─────────┴──────────┴────────────┴───────────────────────╢");
        println!("  ║                                                                                      ║");
        println!("  ║  TCP/WebRTC: One lost video packet  →  ALL streams freeze (HOL blocking)             ║");
        println!("  ║  MoQ/QUIC:   Video degrades         →  Robot stays under control (priority streams)  ║");
        println!("  ║                                                                                      ║");
        println!("  ╚══════════════════════════════════════════════════════════════════════════════════════╝");
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
