# telemoq

MoQ transport for robotics teleoperation. Demonstrates priority-based stream starvation under packet loss: high-priority control telemetry stays rock-steady while low-priority video degrades gracefully.

Built on [moq-dev/moq](https://github.com/moq-dev/moq) (Media over QUIC).

## The Demo

Under 15% packet loss, the subscriber output tells a story:

```
  telemoq | robot-1 | relay: https://localhost:4443 | uptime: 12s
  ╔══════════════════════╤═════╤═════════╤══════════╤════════════╤═══════════════════════╗
  ║ Track                │ Pri │ Recv/s  │ Expected │   Status   │ Throughput            ║
  ╟──────────────────────┼─────┼─────────┼──────────┼────────────┼───────────────────────╢
  ║ control/estop        │ P0  │     1/s │      1/s │     OK     │ ████████████████████   10 B/s ║
  ║ control/commands     │ P1  │    98/s │    100/s │     OK     │ ████████████████████  5.5 KB/s ║
  ║ sensors/joints       │ P2  │    96/s │    100/s │     OK     │ ████████████████████ 11.5 KB/s ║
  ║   └─ joints: [-0.12, 0.45, -0.89, 1.23, -0.56, 0.78, 0.01] rad                   ║
  ║ sensors/force_torque │ P3  │    93/s │    100/s │     OK     │ ████████████████████  5.2 KB/s ║
  ║   └─ F=[10.2, 2.0, -5.3]N  weld contact: YES                                      ║
  ║ sensors/imu          │ P5  │   160/s │    200/s │   DEGRAD   │ ████████████████░░░░ 14.1 KB/s ║
  ║ video/camera0        │ P20 │     5/s │     30/s │   DEGRAD   │ ███░░░░░░░░░░░░░░░░░  0.3 MB/s ║
  ╟──────────────────────┴─────┴─────────┴──────────┴────────────┴───────────────────────╢
  ║                                                                                      ║
  ║  TCP/WebRTC: One lost video packet  →  ALL streams freeze (HOL blocking)             ║
  ║  MoQ/QUIC:   Video degrades         →  Robot stays under control (priority streams)  ║
  ║                                                                                      ║
  ╚══════════════════════════════════════════════════════════════════════════════════════╝
```

High-priority control: rock steady. Low-priority video: starved first. The robot never loses control.

## How It Works

Each track maps to a separate QUIC stream with a different priority. Under congestion, quinn's scheduler (a `BinaryHeap` sorted by priority) sends high-priority streams first, starving lower-priority ones. This is not a hack -- it's the core MoQ design principle.

| Track | Priority | Rate | Payload |
|-------|----------|------|---------|
| `control/estop` | P0 | 1 Hz | 1 byte |
| `control/commands` | P1 | 100 Hz | ~56 bytes |
| `sensors/joints` | P2 | 100 Hz | ~120 bytes |
| `sensors/force_torque` | P3 | 100 Hz | ~56 bytes |
| `sensors/imu` | P5 | 200 Hz | ~88 bytes |
| `video/camera0` | P20 | 30 fps | ~50 KB |

No video encoding -- frames are 50KB payloads published at 30fps. This isolates the transport layer behavior. Real H.264 encoding is available in [lumina-video](https://github.com/AcrossTheCloud/lumina-video) and adds ~20ms encode latency.

## Why Not WebRTC?

WebRTC uses DTLS/SCTP over a single ordered byte stream. When a video keyframe is lost, **all streams stall** waiting for retransmission (head-of-line blocking). WebRTC's GCC congestion control degrades everything uniformly -- it can't prioritize control over video.

MoQ/QUIC gives each track an independent stream with explicit priority. Under loss, the scheduler drops video before control. The robot stays under operator command even in poor network conditions.

## Setup

### Prerequisites

Clone [moq-dev/moq](https://github.com/moq-dev/moq) alongside this repo:

```
develop/
├── lumina-video/moq/    # moq-dev/moq repo
└── telemoq/             # this repo
```

Rust 1.85+ required (edition 2024).

### Build

```bash
cargo build
```

### Run

**Terminal 1: Start the relay**

```bash
cd ../lumina-video/moq && cargo run --bin moq-relay -- --listen [::]:4443
```

Or use the dev config if available:

```bash
cd ../lumina-video/moq && cargo run --bin moq-cli -- relay --config dev/relay.toml
```

**Terminal 2: Publisher**

```bash
cargo run -- --url https://localhost:4443 --tls-disable-verify --broadcast robot-1 publish
```

**Terminal 3: Subscriber**

```bash
cargo run -- --url https://localhost:4443 --tls-disable-verify --broadcast robot-1 subscribe
```

### Packet Loss Testing (macOS)

macOS doesn't have `tc netem`. Use `dnctl` + `pfctl`:

```bash
# Add 15% packet loss on loopback
sudo dnctl pipe 1 config plr 0.15
echo "dummynet in proto udp from any to any port 4443 pipe 1" | sudo pfctl -f -
sudo pfctl -e

# Remove
sudo pfctl -d
sudo dnctl flush
```

Or use Apple's Network Link Conditioner (Xcode > Additional Tools).

### CSV Logging

```bash
cargo run -- --url https://localhost:4443 --tls-disable-verify --broadcast robot-1 --csv subscribe > telemoq.csv
```

Plot with:

```python
import pandas as pd
df = pd.read_csv("telemoq.csv")
df.pivot(columns="track", values="recv_per_s").plot(title="telemoq: priority starvation under packet loss")
```

## Architecture

- **Single binary** with `publish` / `subscribe` subcommands
- **moq-lite**: Origin/Broadcast/Track/Group/Frame hierarchy
- **moq-native**: QUIC connection management (quinn backend)
- Publisher creates one `TrackProducer` per telemetry stream, each with a different priority
- Subscriber waits for broadcast announcement, subscribes to all tracks, displays live stats
- Under congestion, quinn's BBR congestion controller + priority scheduler deliver high-priority tracks first

### Production Notes

- This demo uses a single publisher for both robot-to-operator (sensors, video) and operator-to-robot (commands) streams. In production, each endpoint runs its own publisher; bidirectional flow happens through the relay.
- Video frames are synthetic 50KB blobs. Swap in real H.264/H.265 encoding for production.
- For multi-robot fleet observation, multiple subscribers connect to the same relay and subscribe to different broadcast names.

## License

MIT OR Apache-2.0
