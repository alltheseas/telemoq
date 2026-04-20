# Ethernet Benchmark Methodology

## Date: 2026-04-20

## Hardware
- **Publisher**: System76 Lemur Pro (b@10.0.0.2), Ubuntu (Pop!_OS)
- **Relay + Subscriber**: Mac Mini M2 (10.0.0.1), macOS
- **Link**: Direct Ethernet cable, 1000baseT full-duplex
- **Ethernet interfaces**: Mac `en0`, Lemur `enxf8e43b407b77` (USB adapter)
- **Baseline RTT**: ~2.2ms (ping)

## Software
- telemoq branch: `fix/moq-lite-api-result` (commit 9ccf703)
- moq-relay: from `lumina-video/moq` (release build)
- Relay flags: `--server-bind '[::]:4443' --tls-generate 'localhost,10.0.0.1' --auth-public '/'`
- Publisher flags: `--url 'https://10.0.0.1:4443' --tls-disable-verify --broadcast robot-1 --log-bandwidth --no-reconnect publish`
- Subscriber flags: `--url 'https://10.0.0.1:4443' --tls-disable-verify --broadcast robot-1 --csv subscribe`

## Traffic shaping
- Applied on **publisher side** (Lemur) using Linux `tc tbf`
- Command: `sudo tc qdisc add dev enxf8e43b407b77 root tbf rate <RATE> burst 32kbit latency 50ms`
- Removed between tests: `sudo tc qdisc del dev enxf8e43b407b77 root`

## Test matrix
| File | Bandwidth cap | Duration | Notes |
|------|--------------|----------|-------|
| ethernet-baseline.csv | None (1Gbps) | ~34s | All tracks delivered at full rate |
| ethernet-50mbps.csv | 50 Mbps | ~56s | All tracks fit (demand ~38.5Mbps) |
| ethernet-10mbps.csv | 10 Mbps | ~44s | Video shed, all control 100% |
| ethernet-5mbps.csv | 5 Mbps | ~44s | Video shed, all control 100% |

## Track configuration (from schema.rs)
| Track | Priority | Rate | Payload | Bitrate |
|-------|----------|------|---------|---------|
| safety/heartbeat | 255 | 10 Hz | 17 B | 1.4 Kbps |
| control/streaming | 200 | 167 Hz | 157 B | 210 Kbps |
| control/task_status | 200 | 10 Hz | 14 B | 1.1 Kbps |
| sensors/joints | 150 | 100 Hz | 120 B | 96 Kbps |
| sensors/force_torque | 100 | 100 Hz | 56 B | 45 Kbps |
| sensors/imu | 50 | 200 Hz | 88 B | 141 Kbps |
| perception/pointcloud | 10 | 5 Hz | 50 KB | 2 Mbps |
| video/camera{0,1,2} | 1 | 30 Hz | 50 KB | 12 Mbps each |

Total demand: ~38.5 Mbps (control: ~0.5 Mbps, pointcloud: 2 Mbps, video: 36 Mbps)

## Results summary
| Bandwidth | Heartbeat latency | Control delivery | Video delivery |
|-----------|------------------|-----------------|----------------|
| Baseline (1Gbps) | 0-1ms | 100% | 100% (30fps) |
| 50 Mbps | 4-16ms | 100% | 100% (30fps) |
| 10 Mbps | 3-30ms | 100% | 0% (shed) |
| 5 Mbps | 2-32ms | 100% | 0% (shed) |

## Key findings
1. **Deterministic**: Unlike WiFi benchmarks, Ethernet + tc results are stable and reproducible
2. **Priority scheduling works**: At 10 and 5 Mbps, video is cleanly shed while all control tracks maintain 100% delivery
3. **No shedding oscillation at 5Mbps**: Video stays shed (no restore attempts), unlike WiFi where CC estimate noise caused oscillation
4. **Latency scales with congestion**: Heartbeat latency increases from <1ms to ~15ms average as bandwidth tightens, but stays well within safety thresholds
