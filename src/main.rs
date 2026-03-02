//! telemoq — MoQ transport for robotics teleoperation.
//!
//! "Build highly optimized, low-latency, reliable data streaming systems
//!  over unreliable transports in real-world conditions."
//!
//! On-robot: ROS2/DDS handles 1kHz inter-process comms. LCM for lightweight messaging.
//! WAN link: MoQ/QUIC with per-stream priority. This is the gap telemoq fills.
//! Alternative to WebRTC (Polymath, Transitive, Viam) and DDS+SRT (IHMC) for the
//! operator↔robot link over WiFi and cellular.

use anyhow::Context;
use clap::Parser;
use url::Url;

mod publish;
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
}

#[derive(Parser, Clone)]
pub enum Role {
    Publish,
    Subscribe,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let config = Config::parse();
    config.log.init();

    let client = config.client.init()?;

    match config.role {
        Role::Publish => publish::run(client, &config.url, &config.broadcast, config.no_priority, config.single_stream)
            .await
            .context("publisher error"),
        Role::Subscribe => {
            subscribe::run(client, &config.url, &config.broadcast, config.csv, config.no_priority, config.single_stream)
                .await
                .context("subscriber error")
        }
    }
}
