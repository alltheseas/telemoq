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
        Role::Publish => publish::run(client, &config.url, &config.broadcast)
            .await
            .context("publisher error"),
        Role::Subscribe => subscribe::run(client, &config.url, &config.broadcast, config.csv)
            .await
            .context("subscriber error"),
    }
}
