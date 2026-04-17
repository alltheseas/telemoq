use serde::{Deserialize, Serialize};

/// Wall-clock timestamp in milliseconds since UNIX epoch.
/// Used by both publisher and subscriber to compute end-to-end latency.
pub fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock before UNIX epoch")
        .as_millis() as u64
}

// telemoq: MoQ transport for the WAN teleop link.
//
// Protocol context:
//   On-robot:  ROS2/DDS (RTPS) for 1kHz inter-process comms, LCM for lightweight messaging.
//   WAN link:  MoQ/QUIC — this crate. Priority-aware streaming over WiFi and cellular.
//   Industry:  Polymath, Transitive, Viam all use WebRTC (HOL blocking under loss).
//              IHMC uses DDS + SRT (two protocols, no cross-protocol priority).
//   telemoq:   Single QUIC connection, per-stream priority. Video degrades gracefully.

/// Track definition: name, priority (higher = higher in MoQ), publish rate Hz, expected payload bytes.
pub struct TrackDef {
    pub name: &'static str,
    pub priority: u8,
    pub rate_hz: u32,
    pub payload_bytes: usize,
    pub label: &'static str,
}

impl TrackDef {
    /// Nominal bitrate in bits per second: rate_hz × payload_bytes × 8.
    pub const fn bitrate_bps(&self) -> u64 {
        self.rate_hz as u64 * self.payload_bytes as u64 * 8
    }
}

/// Minimum bandwidth (bps) to keep each track active.
/// For track `i`, this is the cumulative bitrate of all tracks with priority >= TRACKS[i].priority.
/// Index 0 (safety/heartbeat) returns 0 — never shed.
pub fn shed_thresholds() -> [u64; TRACKS.len()] {
    let mut thresholds = [0u64; TRACKS.len()];
    let mut i = 1; // skip index 0: never shed safety
    while i < TRACKS.len() {
        let mut cumulative = 0u64;
        let mut j = 0;
        while j < TRACKS.len() {
            if TRACKS[j].priority >= TRACKS[i].priority {
                cumulative += TRACKS[j].bitrate_bps();
            }
            j += 1;
        }
        thresholds[i] = cumulative;
        i += 1;
    }
    thresholds
}

// IHMC-informed track hierarchy (validated against DRC & KST architecture):
//
// MoQ priority: higher value = scheduled first by QUIC stream scheduler.
//
// P255 — Safety: watchdog heartbeat. Absence = connection lost → robot safe mode.
// P200 — Control: SE3 pose targets (KST streaming at 167Hz) + task acknowledgments.
// P150 — Sensors/joints: 100Hz joint state for operator situational awareness.
// P100 — Sensors/force_torque: 100Hz contact detection for weld monitoring.
// P50  — Sensors/imu: 200Hz orientation/accel. First to degrade under congestion.
// P10  — Perception: downsampled point cloud for 3D world model.
// P1   — Video: camera feed. Lowest priority — starved first under congestion.
//
// Key insight from IHMC DRC: 9,600 bps was sufficient for control.
// The robot's onboard 1kHz QP controller handles low-level execution.
// The teleop link carries high-level task goals, not raw joint commands.

pub const TRACKS: &[TrackDef] = &[
    TrackDef { name: "safety/heartbeat",      priority: 255, rate_hz: 10,  payload_bytes: 17,     label: "safety/heartbeat" },
    TrackDef { name: "control/streaming",     priority: 200, rate_hz: 167, payload_bytes: 160,    label: "control/streaming" },
    TrackDef { name: "control/task_status",   priority: 200, rate_hz: 10,  payload_bytes: 14,     label: "control/task_ack" },
    TrackDef { name: "sensors/joints",        priority: 150, rate_hz: 100, payload_bytes: 120,    label: "sensors/joints" },
    TrackDef { name: "sensors/force_torque",  priority: 100, rate_hz: 100, payload_bytes: 56,     label: "sensors/force_torque" },
    TrackDef { name: "sensors/imu",           priority: 50,  rate_hz: 200, payload_bytes: 88,     label: "sensors/imu" },
    TrackDef { name: "perception/pointcloud", priority: 10,  rate_hz: 5,   payload_bytes: 50_000, label: "perception/ptcloud" },
    TrackDef { name: "video/camera0",         priority: 1,   rate_hz: 30,  payload_bytes: 50_000, label: "video/camera0" },
    TrackDef { name: "video/camera1",         priority: 1,   rate_hz: 30,  payload_bytes: 50_000, label: "video/camera1" },
    TrackDef { name: "video/camera2",         priority: 1,   rate_hz: 30,  payload_bytes: 50_000, label: "video/camera2" },
];

// --- Data types ---

/// Watchdog heartbeat (P0). Absence triggers robot safe mode.
/// Replaces binary e-stop toggle with continuous liveness signal.
/// IHMC pattern: if no heartbeat within threshold → hold position → weight decay → safe stop.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Heartbeat {
    pub timestamp_ms: u64,
    pub sequence: u64,      // monotonic counter — gaps indicate packet loss
    pub estop_engaged: bool,
}

/// SE3 streaming command (P1). Matches IHMC KST WholeBodyStreamingMessage pattern.
/// Per-body-part presence flags — only actively controlled parts send data.
/// stream_integration_duration_ms: forward extrapolation window (IHMC default: 12ms).
/// On timeout: controller extrapolates trajectory, then holds position, then weight decay.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct StreamingCommand {
    pub timestamp_ms: u64,
    pub stream_integration_duration_ms: u16,
    // Left hand SE3 (position + orientation)
    pub has_left_hand: bool,
    pub left_hand_pos: [f64; 3],  // xyz meters
    pub left_hand_quat: [f64; 4], // wxyz
    // Right hand SE3
    pub has_right_hand: bool,
    pub right_hand_pos: [f64; 3],
    pub right_hand_quat: [f64; 4],
    // Chest SO3 (orientation only)
    pub has_chest: bool,
    pub chest_quat: [f64; 4],
}

/// Task status acknowledgment (P1, robot→operator).
/// Coactive Design: operator must observe task progress (observability principle).
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TaskStatus {
    pub timestamp_ms: u64,
    pub task_id: u32,
    pub progress_pct: u8, // 0-100
    pub state: u8,        // 0=idle, 1=executing, 2=complete, 3=failed
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct JointState {
    pub timestamp_ms: u64,
    pub positions: [f64; 7],  // 7-DOF arm (Franka Panda)
    pub velocities: [f64; 7],
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ImuReading {
    pub timestamp_ms: u64,
    pub accel: [f64; 3],     // m/s^2
    pub gyro: [f64; 3],      // rad/s
    pub orientation: [f64; 4], // quaternion (w, x, y, z)
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ForceTorque {
    pub timestamp_ms: u64,
    pub force: [f64; 3],   // N
    pub torque: [f64; 3],  // Nm
}

// --- Generators: deterministic sinusoidal, physically plausible ---

/// Franka Panda joint limits (rad): [q_min, q_max] per joint
const JOINT_LIMITS: [(f64, f64); 7] = [
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
];

/// Generate a watchdog heartbeat with the given timestamp and sequence number.
pub fn generate_heartbeat(elapsed_ms: u64, sequence: u64) -> Heartbeat {
    Heartbeat {
        timestamp_ms: elapsed_ms,
        sequence,
        estop_engaged: false,
    }
}

/// Generate an SE3 streaming command with sinusoidal hand/chest motion.
pub fn generate_streaming_command(elapsed_ms: u64) -> StreamingCommand {
    let t = elapsed_ms as f64 / 1000.0;
    StreamingCommand {
        timestamp_ms: elapsed_ms,
        stream_integration_duration_ms: 12, // IHMC KST default
        has_left_hand: true,
        left_hand_pos: [
            0.45 + 0.12 * (0.3 * t).sin(),   // forward reach
            0.22 + 0.08 * (0.2 * t).cos(),    // lateral
            0.95 + 0.15 * (0.15 * t).sin(),   // height
        ],
        left_hand_quat: {
            let angle = 0.25 * (0.2 * t).sin();
            let half = angle / 2.0;
            [half.cos(), half.sin(), 0.0, 0.0]
        },
        has_right_hand: true,
        right_hand_pos: [
            0.50 + 0.10 * (0.25 * t + 1.0).sin(),  // welding torch reach
            -0.20 + 0.06 * (0.3 * t + 0.5).cos(),   // lateral
            0.85 + 0.10 * (0.2 * t + 0.7).sin(),     // height
        ],
        right_hand_quat: {
            let angle = 0.2 * (0.15 * t + 0.5).sin();
            let half = angle / 2.0;
            [half.cos(), 0.0, half.sin(), 0.0]
        },
        has_chest: true,
        chest_quat: {
            let angle = 0.08 * (0.1 * t).sin(); // subtle body sway
            let half = angle / 2.0;
            [half.cos(), 0.0, 0.0, half.sin()]
        },
    }
}

/// Generate a task status cycling through idle → executing → complete every 10s.
pub fn generate_task_status(elapsed_ms: u64) -> TaskStatus {
    // Simulate repeating 10-second task cycle: idle → executing → complete
    let cycle_ms: u64 = 10_000;
    let phase = (elapsed_ms % cycle_ms) as f64 / cycle_ms as f64;
    let (state, progress) = if phase < 0.1 {
        (0u8, 0u8) // idle
    } else if phase < 0.9 {
        (1u8, ((phase - 0.1) / 0.8 * 100.0) as u8) // executing
    } else {
        (2u8, 100u8) // complete
    };
    TaskStatus {
        timestamp_ms: elapsed_ms,
        task_id: (elapsed_ms / cycle_ms) as u32,
        progress_pct: progress,
        state,
    }
}

/// Generate 7-DOF joint state with sinusoidal motion within Franka Panda limits.
pub fn generate_joint_state(elapsed_ms: u64) -> JointState {
    let t = elapsed_ms as f64 / 1000.0;
    let mut positions = [0.0f64; 7];
    let mut velocities = [0.0f64; 7];
    for i in 0..7 {
        let (lo, hi) = JOINT_LIMITS[i];
        let mid = (lo + hi) / 2.0;
        let amp = (hi - lo) / 4.0;
        let freq = 0.1 + 0.05 * i as f64;
        positions[i] = mid + amp * (2.0 * std::f64::consts::PI * freq * t).sin();
        velocities[i] = amp * 2.0 * std::f64::consts::PI * freq
            * (2.0 * std::f64::consts::PI * freq * t).cos();
    }
    JointState { timestamp_ms: elapsed_ms, positions, velocities }
}

/// Generate IMU reading with gravity-dominated accel and small gyro perturbations.
pub fn generate_imu(elapsed_ms: u64) -> ImuReading {
    let t = elapsed_ms as f64 / 1000.0;
    ImuReading {
        timestamp_ms: elapsed_ms,
        accel: [
            0.5 * (0.3 * t).sin(),
            0.3 * (0.5 * t).cos(),
            -9.81 + 0.1 * (0.2 * t).sin(),
        ],
        gyro: [
            0.02 * (0.4 * t).sin(),
            0.01 * (0.6 * t).cos(),
            0.03 * (0.2 * t).sin(),
        ],
        orientation: {
            let angle = 0.05 * (0.1 * t).sin();
            let half = angle / 2.0;
            [half.cos(), half.sin(), 0.0, 0.0]
        },
    }
}

/// Generate force/torque sensor reading simulating weld contact forces.
pub fn generate_force_torque(elapsed_ms: u64) -> ForceTorque {
    let t = elapsed_ms as f64 / 1000.0;
    ForceTorque {
        timestamp_ms: elapsed_ms,
        force: [
            10.0 + 5.0 * (0.5 * t).sin(),
            2.0 * (0.3 * t).cos(),
            -5.0 + 3.0 * (0.4 * t).sin(),
        ],
        torque: [
            0.5 * (0.2 * t).sin(),
            0.3 * (0.4 * t).cos(),
            1.0 + 0.5 * (0.6 * t).sin(),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heartbeat_preserves_fields() {
        let hb = generate_heartbeat(12345, 42);
        assert_eq!(hb.timestamp_ms, 12345);
        assert_eq!(hb.sequence, 42);
        assert!(!hb.estop_engaged);
    }

    #[test]
    fn heartbeat_serializes_to_expected_size() {
        let hb = generate_heartbeat(0, 0);
        let bytes = bincode::serialize(&hb).expect("serialize heartbeat");
        // u64 + u64 + bool = 17 bytes (bincode)
        assert_eq!(bytes.len(), 17);
    }

    #[test]
    fn streaming_command_quaternions_are_unit() {
        for ms in [0, 1000, 5000, 10_000] {
            let cmd = generate_streaming_command(ms);
            let norm = |q: &[f64; 4]| (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
            assert!((norm(&cmd.left_hand_quat) - 1.0).abs() < 1e-10, "left hand quat not unit at t={ms}");
            assert!((norm(&cmd.right_hand_quat) - 1.0).abs() < 1e-10, "right hand quat not unit at t={ms}");
            assert!((norm(&cmd.chest_quat) - 1.0).abs() < 1e-10, "chest quat not unit at t={ms}");
        }
    }

    #[test]
    fn streaming_command_has_all_parts() {
        let cmd = generate_streaming_command(0);
        assert!(cmd.has_left_hand);
        assert!(cmd.has_right_hand);
        assert!(cmd.has_chest);
        assert_eq!(cmd.stream_integration_duration_ms, 12);
    }

    #[test]
    fn task_status_cycles_through_states() {
        let idle = generate_task_status(500);       // 5% of 10s cycle
        assert_eq!(idle.state, 0);
        assert_eq!(idle.progress_pct, 0);

        let mid = generate_task_status(5000);       // 50% of cycle
        assert_eq!(mid.state, 1);
        assert!(mid.progress_pct > 0 && mid.progress_pct < 100);

        let done = generate_task_status(9500);      // 95% of cycle
        assert_eq!(done.state, 2);
        assert_eq!(done.progress_pct, 100);
    }

    #[test]
    fn joint_state_within_limits() {
        for ms in [0, 1000, 5000, 10_000, 60_000] {
            let js = generate_joint_state(ms);
            assert_eq!(js.timestamp_ms, ms);
            for (i, &pos) in js.positions.iter().enumerate() {
                let (lo, hi) = JOINT_LIMITS[i];
                assert!(pos >= lo && pos <= hi, "joint {i} out of range at t={ms}: {pos}");
            }
        }
    }

    #[test]
    fn imu_has_gravity() {
        let imu = generate_imu(0);
        // Z accel should be near -9.81 (gravity)
        assert!((imu.accel[2] + 9.81).abs() < 0.5, "z accel should be near -9.81");
    }

    #[test]
    fn force_torque_has_reasonable_magnitudes() {
        let ft = generate_force_torque(0);
        // X force centered around 10N (weld contact)
        assert!(ft.force[0] > 0.0, "x force should be positive");
        // Torque magnitudes should be < 5 Nm
        for &t in &ft.torque {
            assert!(t.abs() < 5.0, "torque magnitude unreasonable: {t}");
        }
    }

    #[test]
    fn all_generators_bincode_roundtrip() {
        let hb = generate_heartbeat(1000, 1);
        let hb2: Heartbeat = bincode::deserialize(&bincode::serialize(&hb).unwrap()).unwrap();
        assert_eq!(hb.sequence, hb2.sequence);

        let cmd = generate_streaming_command(1000);
        let cmd2: StreamingCommand = bincode::deserialize(&bincode::serialize(&cmd).unwrap()).unwrap();
        assert_eq!(cmd.timestamp_ms, cmd2.timestamp_ms);

        let ts = generate_task_status(1000);
        let ts2: TaskStatus = bincode::deserialize(&bincode::serialize(&ts).unwrap()).unwrap();
        assert_eq!(ts.task_id, ts2.task_id);

        let js = generate_joint_state(1000);
        let js2: JointState = bincode::deserialize(&bincode::serialize(&js).unwrap()).unwrap();
        assert_eq!(js.positions, js2.positions);

        let imu = generate_imu(1000);
        let imu2: ImuReading = bincode::deserialize(&bincode::serialize(&imu).unwrap()).unwrap();
        assert_eq!(imu.accel, imu2.accel);

        let ft = generate_force_torque(1000);
        let ft2: ForceTorque = bincode::deserialize(&bincode::serialize(&ft).unwrap()).unwrap();
        assert_eq!(ft.force, ft2.force);
    }

    #[test]
    fn track_definitions_are_sorted_by_priority() {
        // MoQ: higher value = higher priority. TRACKS listed highest-first (descending).
        for window in TRACKS.windows(2) {
            assert!(
                window[0].priority >= window[1].priority,
                "TRACKS not sorted by priority (descending): {} (P{}) < {} (P{})",
                window[0].name, window[0].priority, window[1].name, window[1].priority,
            );
        }
    }

    #[test]
    fn track_count_matches_expected() {
        assert_eq!(TRACKS.len(), 10, "expected 10 tracks (IHMC KST hierarchy + 3 cameras)");
    }

    #[test]
    fn bitrate_bps_matches_expected() {
        // safety/heartbeat: 10 Hz × 17 B × 8 = 1,360 bps
        assert_eq!(TRACKS[0].bitrate_bps(), 1_360);
        // control/streaming: 167 Hz × 160 B × 8 = 213,760 bps
        assert_eq!(TRACKS[1].bitrate_bps(), 213_760);
        // video/camera0: 30 Hz × 50,000 B × 8 = 12,000,000 bps
        assert_eq!(TRACKS[7].bitrate_bps(), 12_000_000);
        // video/camera1 and camera2 same as camera0
        assert_eq!(TRACKS[8].bitrate_bps(), 12_000_000);
        assert_eq!(TRACKS[9].bitrate_bps(), 12_000_000);
    }

    #[test]
    fn shed_thresholds_are_correct() {
        let t = shed_thresholds();
        // Index 0 (safety) = 0 — never shed
        assert_eq!(t[0], 0);
        // Index 1 (control/streaming P200): cumulative of P>=200 tracks
        // = heartbeat(1360) + streaming(213760) + task_status(11200) = 226,320
        let control_cumulative = TRACKS[0].bitrate_bps() + TRACKS[1].bitrate_bps() + TRACKS[2].bitrate_bps();
        assert_eq!(t[1], control_cumulative);
        assert_eq!(t[2], control_cumulative); // same priority as streaming
        // Last video track (P1): cumulative of ALL tracks
        let total: u64 = TRACKS.iter().map(|t| t.bitrate_bps()).sum();
        assert_eq!(t[TRACKS.len() - 1], total);
        // All video tracks (same P1) should have same threshold
        assert_eq!(t[7], t[8]);
        assert_eq!(t[8], t[9]);
        // Thresholds are monotonically non-decreasing (lower priority = higher threshold)
        for i in 1..t.len() {
            assert!(t[i] >= t[i - 1], "threshold[{i}]={} < threshold[{}]={}", t[i], i - 1, t[i - 1]);
        }
    }
}
