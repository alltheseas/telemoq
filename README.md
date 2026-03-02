# telemoq

> *"Build highly optimized, low-latency, reliable data streaming systems over unreliable transports in real-world conditions."*
>
> *"Build optimized, reliable data streaming protocols functioning over WiFi and cellular networks."*

MoQ (Media over QUIC) transport for robotics teleoperation. Demonstrates priority-based stream starvation under packet loss: high-priority control telemetry stays rock-steady while low-priority video degrades gracefully.

Built on [moq-dev/moq](https://github.com/moq-dev/moq) (Media over QUIC).

### The Protocol Stack

| Layer | Protocol | Role |
|-------|----------|------|
| **On-robot** | ROS2 / DDS (RTPS) | 1kHz inter-process communication between controllers, sensors, actuators |
| **On-robot** | LCM | Lightweight messaging for real-time control loops |
| **WAN teleop** | **MoQ / QUIC** | Operator ↔ robot over WiFi and cellular. Priority-aware. *This is what telemoq demonstrates.* |

The industry default for WAN teleop is WebRTC -- used by [Polymath/LiveKit](https://livekit.io/customers/polymath), [Transitive Robotics](https://transitiverobotics.com/caps/transitive-robotics/webrtc-video/), and [Viam (gRPC+WebRTC)](https://www.viam.com/post/real-time-robot-control-grpc-webrtc). WebRTC's trade-off: all streams share a single ordered channel, so a lost video packet can stall control data (head-of-line blocking).

IHMC uses DDS + SRT (two separate protocols) for control and video respectively. MoQ unifies both in a single QUIC connection with priority scheduling.

## The Demo

Under 15% packet loss, the subscriber output tells a story:

```
  telemoq | robot-1 | relay: https://localhost:4443 | uptime: 12s
  ╔══════════════════════╤═════╤═════════╤══════════╤════════════╤═════════╤═══════╤═══════════════════════╗
  ║ Track                │ Pri │ Recv/s  │ Expected │   Status   │ Latency │  Gap  │ Throughput            ║
  ╟──────────────────────┼─────┼─────────┼──────────┼────────────┼─────────┼───────┼───────────────────────╢
  ║ safety/heartbeat     │ P0  │    10/s │     10/s │     OK     │    1ms  │   8ms │ ████████████████████    170 B/s ║
  ║   └─ seq: 120 estop: ok                                                                              ║
  ║ control/streaming    │ P1  │   165/s │    167/s │     OK     │    1ms  │   6ms │ ████████████████████ 26.4 KB/s ║
  ║ control/task_ack     │ P1  │    10/s │     10/s │     OK     │    1ms  │   9ms │ ████████████████████    140 B/s ║
  ║   └─ task #1: executing (45%)                                                                        ║
  ║ sensors/joints       │ P2  │    98/s │    100/s │     OK     │    2ms  │  10ms │ ████████████████████ 11.8 KB/s ║
  ║   └─ joints: [-0.12, 0.45, -0.89, 1.23, -0.56, 0.78, 0.01] rad                                     ║
  ║ sensors/force_torque │ P3  │    96/s │    100/s │     OK     │    2ms  │  10ms │ ████████████████████  5.4 KB/s ║
  ║   └─ F=[10.2, 2.0, -5.3]N  weld contact: YES                                                        ║
  ║ sensors/imu          │ P5  │   160/s │    200/s │   DEGRAD   │    5ms  │  15ms │ ████████████████░░░░ 14.1 KB/s ║
  ║ perception/ptcloud   │ P10 │     4/s │      5/s │     OK     │   12ms  │  45ms │ ████████████████████  0.2 MB/s ║
  ║ video/camera0        │ P20 │     5/s │     30/s │   DEGRAD   │  340ms  │ 200ms │ ███░░░░░░░░░░░░░░░░░  0.3 MB/s ║
  ╟──────────────────────┴─────┴─────────┴──────────┴────────────┴─────────┴───────┴───────────────────────╢
  ║  Latency = wall-clock delta (pub→relay→sub).  Gap = max inter-arrival time (12ms = IHMC KST window). ║
  ║                                                                                                       ║
  ║  IHMC DRC lesson: 9,600 bps was enough for control. Video is expendable.                              ║
  ║  TCP/WebRTC: One lost video packet  →  ALL streams freeze (HOL blocking)                              ║
  ║  MoQ/QUIC:   Video degrades         →  Robot stays under control (priority streams)                   ║
  ╚═══════════════════════════════════════════════════════════════════════════════════════════════════════╝
```

High-priority control: rock steady. Low-priority video: starved first. The robot never loses control.

## How It Works

Each track maps to a separate QUIC stream with a different priority. Under congestion, quinn's scheduler (a `BinaryHeap` sorted by priority) sends high-priority streams first, starving lower-priority ones. This is not a hack -- it's the core MoQ design principle.

Track hierarchy follows [IHMC's DRC/KST architecture](https://github.com/ihmcrobotics/ihmc-open-robotics-software) -- the robot's onboard 1kHz QP controller handles low-level execution; the teleop link carries high-level SE3 task goals, not raw joint commands.

| Track | Priority | Rate | Payload | Description |
|-------|----------|------|---------|-------------|
| `safety/heartbeat` | P0 | 10 Hz | 17 bytes | Watchdog liveness. Absence = safe stop. |
| `control/streaming` | P1 | 167 Hz | ~160 bytes | SE3 pose targets (IHMC KST pattern) |
| `control/task_status` | P1 | 10 Hz | 14 bytes | Task acknowledgment (robot → operator) |
| `sensors/joints` | P2 | 100 Hz | ~120 bytes | 7-DOF joint state (Franka Panda) |
| `sensors/force_torque` | P3 | 100 Hz | ~56 bytes | Contact detection for weld monitoring |
| `sensors/imu` | P5 | 200 Hz | ~88 bytes | Orientation/acceleration |
| `perception/pointcloud` | P10 | 5 Hz | ~50 KB | Downsampled 3D point cloud |
| `video/camera0` | P20 | 30 fps | ~50 KB | Camera feed (lowest priority) |

No video encoding -- frames are 50KB payloads published at 30fps. This isolates the transport layer behavior. Real H.264 encoding is available in [lumina-video](https://github.com/AcrossTheCloud/lumina-video) and adds ~20ms encode latency.

### Latency Measurement

The publisher embeds wall-clock timestamps (`SystemTime::now()`) in every message. The subscriber computes per-track latency as `recv_time - publish_time`. On localhost with synchronized clocks, this gives sub-millisecond accuracy.

The subscriber also tracks **max inter-arrival gap** per track. The IHMC KST pipeline uses a 12ms forward extrapolation window (`stream_integration_duration`). When inter-arrival gap exceeds 12ms, the robot controller must extrapolate beyond observed data -- gaps shown in red in the terminal display.

## Why Not WebRTC?

WebRTC is the default in robotics teleoperation:
- **[Polymath Robotics](https://livekit.io/customers/polymath)** uses LiveKit/WebRTC to teleoperate dozers in Ontario from Las Vegas
- **[Transitive Robotics](https://transitiverobotics.com/caps/transitive-robotics/webrtc-video/)** provides WebRTC P2P video streaming for ROS robots ($20/robot/month)
- **[Viam](https://www.viam.com/post/real-time-robot-control-grpc-webrtc)** wraps gRPC over WebRTC for distributed robot communication

WebRTC's trade-off: DTLS/SCTP uses a single ordered byte stream. When a video keyframe is lost, all streams wait for retransmission (head-of-line blocking). GCC congestion control degrades everything uniformly -- it doesn't distinguish control from video.

MoQ/QUIC gives each track an independent stream with explicit priority. Under loss, the scheduler degrades video before control. The robot stays under operator command even in poor network conditions -- exactly the "reliable data streaming over unreliable transports" that industrial teleop requires.

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
cd ../lumina-video/moq && cargo run --bin moq-relay -- --listen '[::]:4443' --tls-generate localhost --auth-public ''
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

CSV columns: `elapsed_s,track,priority,recv_per_s,expected_per_s,pct,bytes_per_s,avg_latency_ms,max_gap_ms`

Plot with:

```python
import pandas as pd
df = pd.read_csv("telemoq.csv")
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
df.pivot(columns="track", values="recv_per_s").plot(ax=ax1, title="Recv rate (priority starvation)")
df.pivot(columns="track", values="avg_latency_ms").plot(ax=ax2, title="Latency (priority queuing)")
```

## Benchmark: Priority vs No-Priority

The `--no-priority` flag disables per-stream priority scheduling, setting all 8 tracks to equal priority 0. Same binary, same relay, same connection, same payloads — one variable: priority scheduling.

This produces an apples-to-apples comparison: MoQ with priority (the default) vs MoQ without priority (equivalent to a transport that treats all streams equally, like DDS topics sharing a bus or WebRTC's uniform congestion response).

```bash
# Terminal 1: relay
cd ../lumina-video/moq && cargo run --bin moq-relay -- --listen '[::]:4443' --tls-generate localhost --auth-public ''

# Run 1: With priority (default)
# Terminal 2: publisher
cargo run -- --url https://localhost:4443 --tls-disable-verify --broadcast robot-1 publish
# Terminal 3: subscriber
cargo run -- --url https://localhost:4443 --tls-disable-verify --broadcast robot-1 --csv subscribe > with-priority.csv

# Enable 15% packet loss, wait 60s, ctrl-C both.

# Run 2: Without priority
# Terminal 2: publisher
cargo run -- --url https://localhost:4443 --tls-disable-verify --broadcast robot-1 --no-priority publish
# Terminal 3: subscriber
cargo run -- --url https://localhost:4443 --tls-disable-verify --broadcast robot-1 --no-priority --csv subscribe > no-priority.csv
```

**Expected results under congestion:**
- `with-priority.csv`: P0-P1 tracks at ~100% delivery, video degraded (priority starvation working)
- `no-priority.csv`: ALL tracks degraded equally (no prioritization)

See `telemoq-benchmark.html` for a visual comparison chart.

## Architecture

- **Single binary** with `publish` / `subscribe` subcommands
- **moq-lite**: Origin/Broadcast/Track/Group/Frame hierarchy
- **moq-native**: QUIC connection management (quinn backend)
- Publisher creates one `TrackProducer` per telemetry stream, each with a different priority
- Subscriber waits for broadcast announcement, subscribes to all tracks, displays live stats
- Under congestion, quinn's BBR congestion controller + priority scheduler deliver high-priority tracks first

### Congestion Control Caveat (BBR)

Quinn uses **BBR** (Bottleneck Bandwidth and RTT) congestion control. BBR is designed for congestion-induced loss, not random packet loss:

- **Bandwidth constraint** (what this demo tests): BBR adapts well. Priority starvation works as expected -- high-priority streams get bandwidth first.
- **Random packet loss >5%** (real-world RF environments): BBR can misinterpret random loss as congestion and collapse throughput for *all* streams, regardless of priority. This is a known limitation ([BBR FAQ](https://github.com/google/bbr/blob/master/Documentation/bbr-faq.md)).

For production in high-loss environments (shipyards at 15% loss per [NIST TN 1982](https://doi.org/10.6028/NIST.TN.1982)), consider:
- **Loss-tolerant congestion control** (e.g., BBRv2, COPA)
- **FEC (Forward Error Correction)** at the QUIC layer
- **Redundant encoding** for safety-critical tracks (heartbeat, e-stop)

### QUIC Connection Migration

QUIC supports connection migration -- the ability to maintain a session across network changes (e.g., WiFi → cellular, or robot moving between access points). This is valuable for mobile robots in shipyards where:

- The robot moves between welding bays with different APs
- RF conditions change as the robot enters/exits steel enclosures
- The operator's XR headset roams between networks

Unlike TCP (which binds to a 4-tuple and dies on network change) or WebRTC (which requires ICE restart), QUIC connections survive network transitions transparently using connection IDs. The teleop session continues without interruption.

**Note**: Connection migration requires relay support. The current moq-relay does not yet implement migration, but the protocol supports it.

### Production Notes

- This demo uses a single publisher for both robot-to-operator (sensors, video) and operator-to-robot (commands) streams. In production, each endpoint runs its own publisher; bidirectional flow happens through the relay.
- Video frames are synthetic 50KB blobs. Swap in real H.264/H.265 encoding for production.
- For multi-robot fleet observation, multiple subscribers connect to the same relay and subscribe to different broadcast names.
- IHMC uses DDS + SRT (two separate protocols) for control and video respectively. MoQ unifies both in a single QUIC connection with priority scheduling.

## License

MIT OR Apache-2.0
