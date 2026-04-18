//! telemoq — MoQ transport for robotics teleoperation.
//!
//! "Build highly optimized, low-latency, reliable data streaming systems
//!  over unreliable transports in real-world conditions."
//!
//! On-robot: ROS2/DDS handles 1kHz inter-process comms. LCM for lightweight messaging.
//! WAN link: MoQ/QUIC with per-stream priority. This is the gap telemoq fills.
//! Alternative to WebRTC (Polymath, Transitive, Viam) and DDS+SRT (IHMC) for the
//! operator↔robot link over WiFi and cellular.

use std::time::{Duration, Instant};

use anyhow::Context;
use clap::Parser;
use url::Url;

mod publish;
mod replay;
mod schema;
mod subscribe;

#[derive(Parser, Clone)]
pub struct Config {
    /// Connect to the given URL starting with https://
    #[arg(long)]
    pub url: Url,

    /// The broadcast name (robot identifier)
    #[arg(long, default_value = "robot-1")]
    pub broadcast: String,

    /// The MoQ client configuration.
    #[command(flatten)]
    pub client: moq_native::ClientConfig,

    /// The log configuration.
    #[command(flatten)]
    pub log: moq_native::Log,

    /// Publish or subscribe mode
    #[command(subcommand)]
    pub role: Role,

    /// Enable CSV logging to stdout (subscriber only)
    #[arg(long)]
    pub csv: bool,

    /// Disable per-stream priority (all tracks get equal priority 0).
    /// Benchmark mode: simulates a transport without priority scheduling (like DDS).
    #[arg(long)]
    pub no_priority: bool,

    /// Multiplex all tracks into a single QUIC stream.
    /// Benchmark mode: simulates WebRTC-style head-of-line blocking.
    #[arg(long)]
    pub single_stream: bool,

    /// Disable publisher-side track shedding (publisher only).
    /// When shedding is active, low-priority tracks are dropped when bandwidth is scarce.
    #[arg(long)]
    pub no_shed: bool,

    /// Log QUIC congestion controller bandwidth estimate every second (publisher only).
    #[arg(long)]
    pub log_bandwidth: bool,

    /// Replay DROID dataset from the given directory instead of synthetic data (publisher only).
    /// Directory must contain meta.json and episode_NNN/ subdirectories.
    #[arg(long)]
    pub replay: Option<String>,

    /// Publish all 3 cameras (default: camera0 only). Only used with --replay.
    #[arg(long)]
    pub all_cameras: bool,

    /// Disable auto-reconnect on connection loss.
    /// By default, telemoq reconnects with jittered exponential backoff.
    #[arg(long)]
    pub no_reconnect: bool,
}

#[derive(Parser, Clone)]
pub enum Role {
    Publish,
    Subscribe,
}

/// Run a single publish or subscribe session. Returns when the session ends.
async fn run_session(config: &Config) -> anyhow::Result<()> {
    let client = config.client.clone().init()?;

    match config.role {
        Role::Publish => {
            if let Some(ref replay_path) = config.replay {
                publish::run_replay(
                    client,
                    &config.url,
                    &config.broadcast,
                    replay_path,
                    config.all_cameras,
                    config.no_priority,
                    config.single_stream,
                    config.no_shed,
                    config.log_bandwidth,
                )
                .await
                .context("replay publisher error")
            } else {
                publish::run(client, &config.url, &config.broadcast, config.no_priority, config.single_stream, config.no_shed, config.log_bandwidth)
                    .await
                    .context("publisher error")
            }
        }
        Role::Subscribe => {
            subscribe::run(client, &config.url, &config.broadcast, config.csv, config.no_priority, config.single_stream)
                .await
                .context("subscriber error")
        }
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let config = Config::parse();
    config.log.init();

    if config.no_reconnect {
        return run_session(&config).await;
    }

    // Reconnect loop with jittered exponential backoff.
    // Defends against WiFi/cellular dropouts that kill the QUIC connection.
    let mut attempt: u32 = 0;
    let mut rapid_failures: u32 = 0;
    let mut delay = Duration::from_secs(1);
    let max_delay = Duration::from_secs(30);
    // Sessions lasting < 2s are "rapid failures" — likely misconfiguration, not WiFi dropout
    let rapid_failure_threshold = Duration::from_secs(2);
    let max_rapid_failures: u32 = 3;

    loop {
        attempt += 1;
        let session_start = Instant::now();

        tracing::info!(attempt, "connecting to {}", config.url);

        match run_session(&config).await {
            Ok(()) => {
                tracing::info!(
                    attempt,
                    session_duration_s = session_start.elapsed().as_secs(),
                    "session ended cleanly"
                );
                break Ok(());
            }
            Err(e) => {
                let session_duration = session_start.elapsed();
                let rapid = session_duration < rapid_failure_threshold;

                tracing::warn!(
                    attempt,
                    session_duration_ms = session_duration.as_millis() as u64,
                    error = %e,
                    rapid_failure = rapid,
                    "session lost — will reconnect"
                );

                if rapid {
                    rapid_failures += 1;
                } else {
                    rapid_failures = 0;
                }

                // Misconfiguration guard: if sessions keep dying immediately, stop spinning
                if rapid_failures >= max_rapid_failures {
                    tracing::error!(
                        attempt,
                        rapid_failures,
                        "aborting: {} consecutive rapid failures (< {}s each) — likely misconfiguration (wrong URL, bad cert, relay down)",
                        rapid_failures,
                        rapid_failure_threshold.as_secs()
                    );
                    break Err(e.context("too many rapid failures — check URL and relay"));
                }

                // Reset backoff after a session that ran long enough to be "real"
                if !rapid {
                    delay = Duration::from_secs(1);
                }

                // Jitter: delay + random(0..delay/2) prevents thundering herd
                let jitter = Duration::from_millis(
                    rand::random::<u64>() % (delay.as_millis() as u64 / 2).max(1)
                );
                let sleep_duration = delay + jitter;

                tracing::info!(
                    delay_ms = sleep_duration.as_millis() as u64,
                    next_attempt = attempt + 1,
                    "backing off before reconnect"
                );

                tokio::time::sleep(sleep_duration).await;

                // Exponential backoff (capped)
                delay = (delay * 2).min(max_delay);
            }
        }
    }
}
