# telemoq

> *"Build optimized, reliable data streaming protocols functioning over WiFi and cellular networks."*

**[Live Demo: MoQ vs WebRTC vs DDS](https://alltheseas.github.io/telemoq/telemoq-vs-webrtc.html)** | **[Benchmark Results](https://alltheseas.github.io/telemoq/telemoq-benchmark.html)**

MoQ (Media over QUIC) transport for robotics teleoperation. Under congestion, high-priority control stays rock-steady while low-priority video degrades gracefully.

Built on [moq-dev/moq](https://github.com/moq-dev/moq). MoQ is an [IETF standards-track protocol](https://datatracker.ietf.org/doc/draft-ietf-moq-transport/) over [QUIC (RFC 9000)](https://www.rfc-editor.org/rfc/rfc9000).

### The Protocol Stack

| Layer | Protocol | Role |
|-------|----------|------|
| **On-robot** | ROS2 / DDS, LCM | 1kHz IPC between controllers, sensors, actuators |
| **WAN teleop** | **MoQ / QUIC** | Operator ↔ robot over WiFi/cellular. Priority-aware. *This is what telemoq demonstrates.* |

The industry default for WAN teleop is WebRTC ([Polymath/LiveKit](https://livekit.io/customers/polymath), [Transitive](https://transitiverobotics.com/caps/transitive-robotics/webrtc-video/), [Viam](https://www.viam.com/post/real-time-robot-control-grpc-webrtc)). WebRTC's trade-off: all streams share one ordered channel, so a lost video packet stalls control data (head-of-line blocking). MoQ gives each track an independent QUIC stream with explicit priority.

## The Demo

Under congestion (bandwidth-limited, not artificial packet loss), the subscriber output tells the story:

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

Each track maps to a separate QUIC stream with a different priority. Under congestion, quinn's priority scheduler sends high-priority streams first, starving lower-priority ones. Track hierarchy follows [IHMC's KST architecture](https://github.com/ihmcrobotics/ihmc-open-robotics-software) -- the teleop link carries SE3 task goals, not raw joint commands.

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

No video encoding -- frames are 50KB payloads at 30fps to isolate transport behavior. Real H.264 via [lumina-video](https://github.com/AcrossTheCloud/lumina-video) adds ~20ms encode latency.

### Latency Measurement

Publisher embeds `SystemTime::now()` timestamps; subscriber computes `recv_time - publish_time` per track. Also tracks **max inter-arrival gap** vs. IHMC's 12ms KST extrapolation window.

## Setup

### Prerequisites

Clone [moq-dev/moq](https://github.com/moq-dev/moq) alongside this repo:

```
develop/
├── lumina-video/moq/    # moq-dev/moq repo
└── telemoq/             # this repo
```

Rust 1.85+ required (edition 2021).

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

### CSV Logging

```bash
cargo run -- --url https://localhost:4443 --tls-disable-verify --broadcast robot-1 --csv subscribe > telemoq.csv
```

CSV columns: `elapsed_s,track,priority,recv_per_s,expected_per_s,pct,bytes_per_s,avg_latency_ms,max_gap_ms,mode`

## Benchmark: Three Modes, One Binary

Three benchmark modes isolate the effect of priority scheduling and stream multiplexing:

| Flag | Simulates | Behavior |
|------|-----------|----------|
| *(default)* | **MoQ** | Per-stream priority scheduling |
| `--no-priority` | **DDS** | All tracks equal priority 0, no starvation |
| `--single-stream` | **WebRTC** | All tracks multiplexed into one QUIC stream (HOL blocking) |

Same binary, same relay, same payloads. Only the transport behavior changes. Add `--no-priority` or `--single-stream` flags to publisher and subscriber.

**Measured results under congestion (~63s runs):**
- **Priority (MoQ):** P0-P5 control: 22.2 msgs/s. Video/pointcloud starved to zero.
- **No-priority (DDS-like):** Control: 5.3 msgs/s (76% loss). Degradation unpredictable.
- **Single-stream (WebRTC-like):** Control: 0.2 msgs/s (99% loss). Near-total blackout.

See [benchmark results](https://alltheseas.github.io/telemoq/telemoq-benchmark.html) for visual comparison.

## Architecture

- **Single binary** with `publish` / `subscribe` subcommands
- Publisher creates one `TrackProducer` per stream with a different priority
- Under congestion, quinn's BBR + priority scheduler deliver high-priority tracks first

### Caveats

- **BBR vs. random loss:** BBR handles bandwidth congestion well (what this demo tests) but can collapse under >5% random packet loss. Production in high-loss environments needs FEC or loss-tolerant CC. See [BBR FAQ](https://github.com/google/bbr/blob/master/Documentation/bbr-faq.md).
- **Connection migration:** QUIC supports session migration across network changes (WiFi → cellular) via connection IDs. Not yet implemented in moq-relay.
- **Single publisher:** Demo uses one publisher for both directions. Production uses two publishers through the relay.

## License

MIT OR Apache-2.0
