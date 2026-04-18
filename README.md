# telemoq – Priority-aware transport for robot teleoperation

**Problem** – In today's teleop stacks (WebRTC, DDS) *all* streams share a single ordered channel. When a video packet is lost, the whole link freezes (head-of-line blocking) and the robot can become uncontrollable.

**Solution** – **telemoq** builds on the IETF-standard **MoQ (Media over QUIC)** protocol. Each logical track (heartbeat, control, joint state, video, point cloud…) lives on its **own QUIC stream** with an explicit priority. Under bandwidth congestion the scheduler delivers high-priority control data first, while lower-priority video is gracefully degraded.

**Impact** – In a realistic 300 kbps congested link the control loop stays **teady (≈ 1 ms latency, 0% packet loss)**, whereas WebRTC drops to **≈ 0.2 msgs/s** for the same control stream. Video simply starves – the robot never loses control.

> "Control data must never freeze." – IHMC DRC Lesson: 9,600 bps is enough for control; everything else is expendable.

[▶️ Live demo – MoQ vs WebRTC vs DDS](https://alltheseas.github.io/telemoq/telemoq-vs-webrtc.html) | [📹 Walk-through video (2 min)](./demotelemoq.mp4) | [Benchmark Results](https://alltheseas.github.io/telemoq/telemoq-benchmark.html)

---

Built on [moq-dev/moq](https://github.com/moq-dev/moq). MoQ is an [IETF standards-track protocol](https://datatracker.ietf.org/doc/draft-ietf-moq-transport/) over [QUIC (RFC 9000)](https://www.rfc-editor.org/rfc/rfc9000).

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
cd ../lumina-video/moq && cargo run --release --bin moq-relay -- \
  --tls-generate localhost --auth-public "" \
  --server-bind '[::]:4443' --web-http-listen 0.0.0.0:4443
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

## Fleet Demo: DROID Dataset Replay

Replay real [DROID](https://droid-dataset.github.io/) robot manipulation data (Franka Panda, 3 cameras, 7-DOF joints) as a multi-robot fleet through the browser viewer.

### Prerequisites

The `sample_data/` directory must exist with converted DROID episodes (see `scripts/convert_droid.py`). The included sample has 3 episodes, 546 frames, 3 cameras per frame.

### Run (3-robot fleet)

**Terminal 1: Relay**

```bash
cd ../lumina-video/moq && cargo run --release --bin moq-relay -- \
  --tls-generate localhost --auth-public "" \
  --server-bind '[::]:4443' --web-http-listen 0.0.0.0:4443
```

**Terminals 2-4: Publishers** (one per robot)

```bash
cargo run --release -- --url https://localhost:4443 --tls-disable-verify \
  --broadcast robot-1 --replay sample_data --all-cameras --no-shed publish

cargo run --release -- --url https://localhost:4443 --tls-disable-verify \
  --broadcast robot-2 --replay sample_data --all-cameras --no-shed publish

cargo run --release -- --url https://localhost:4443 --tls-disable-verify \
  --broadcast robot-3 --replay sample_data --all-cameras --no-shed publish
```

**Browser: Fleet Viewer**

Open `viewer.html` in **Chromium** (not Safari — Safari's WebTransport with self-signed certs is broken). Set Robots to `3`, click Connect.

The viewer shows:
- Live camera grid with per-robot wrist/exterior camera switching
- Real 7-DOF Franka joint positions
- Per-track stats (heartbeat, joints, cameras)
- Heartbeat watchdog (green/red dot)

Each publisher replays the same dataset but episodes are offset in time, so each robot shows different frames.

### Remote Publisher (over WiFi)

To publish from a remote machine (e.g., an on-robot computer):

```bash
# Copy source and data to the remote machine
scp src/*.rs user@robot:~/telemoq/src/
scp Cargo.toml user@robot:~/telemoq/
scp -r sample_data user@robot:~/telemoq/

# Build and run on the remote machine
ssh user@robot "cd ~/telemoq && cargo build --release"
ssh user@robot "cd ~/telemoq && ./target/release/telemoq \
  --url https://<relay-ip>:4443 --tls-disable-verify \
  --broadcast robot-1 --replay sample_data --all-cameras --no-shed publish"
```

### Fleet Stress Test (N robots)

```bash
./fleet-test-pub.sh 10 https://<relay-ip>:4443 --replay sample_data --all-cameras
```

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
