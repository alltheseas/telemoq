//! zenoh-teleop-bench — Zenoh priority starvation benchmark for robotics teleoperation.
//!
//! Apples-to-apples comparison with telemoq (alltheseas/telemoq).
//! Same 8 tracks, same payloads, same rates, same CSV output format.
//! Congestion is intrinsic: aggregate ~3 MB/s publish rate overwhelms transport on localhost.

use clap::Parser;

mod publish;
mod schema;
mod subscribe;

#[derive(Parser, Clone)]
pub struct Config {
    /// Publish or subscribe mode
    #[command(subcommand)]
    pub role: Role,

    /// Key expression prefix (like broadcast name in telemoq)
    #[arg(long, default_value = "robot-1")]
    pub prefix: String,

    /// Zenoh router endpoint to connect to (empty = peer mode)
    #[arg(long)]
    pub connect: Option<String>,

    /// Path to Zenoh JSON5 config file (overrides --connect)
    #[arg(long)]
    pub config: Option<String>,

    /// Disable per-key priority (all keys at same priority).
    /// Benchmark mode: simulates DDS-like flat priority.
    #[arg(long)]
    pub flat: bool,

    /// Multiplex all tracks into a single Zenoh key.
    /// Benchmark mode: simulates WebRTC-style HOL blocking.
    #[arg(long)]
    pub single_key: bool,

    /// Benchmark duration in seconds
    #[arg(long, default_value = "63")]
    pub duration: u64,
}

#[derive(Parser, Clone)]
pub enum Role {
    Publish,
    Subscribe,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"))
        )
        .with_writer(std::io::stderr)
        .init();

    let config = Config::parse();

    // Build Zenoh config
    let zenoh_config = if let Some(ref path) = config.config {
        zenoh::Config::from_file(path).map_err(|e| anyhow::anyhow!("bad config file: {e}"))?
    } else {
        let mut cfg = zenoh::Config::default();
        let endpoint = config.connect.as_deref().unwrap_or("tcp/127.0.0.1:7447");
        cfg.insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
            .map_err(|e| anyhow::anyhow!("bad endpoint config: {e}"))?;
        // Force client mode so all traffic routes through zenohd (not peer-to-peer)
        cfg.insert_json5("mode", "\"client\"")
            .map_err(|e| anyhow::anyhow!("bad mode config: {e}"))?;
        // Disable multicast scouting to prevent peer discovery
        cfg.insert_json5("scouting/multicast/enabled", "false")
            .map_err(|e| anyhow::anyhow!("bad scouting config: {e}"))?;
        cfg
    };

    let session = zenoh::open(zenoh_config).await.map_err(|e| anyhow::anyhow!("{e}"))?;
    tracing::info!("Zenoh session opened");

    // Small delay to let routing stabilize
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    match config.role {
        Role::Publish => {
            publish::run(&session, &config.prefix, config.flat, config.single_key, config.duration).await
        }
        Role::Subscribe => {
            subscribe::run(&session, &config.prefix, config.flat, config.single_key, config.duration).await
        }
    }
}
