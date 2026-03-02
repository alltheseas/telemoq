use serde::{Deserialize, Serialize};

// telemoq: MoQ transport for the WAN teleop link.
//
// Protocol context:
//   On-robot:  ROS2/DDS (RTPS) for 1kHz inter-process comms, LCM for lightweight messaging.
//   WAN link:  MoQ/QUIC — this crate. Priority-aware streaming over WiFi and cellular.
//   Industry:  Polymath, Transitive, Viam all use WebRTC (HOL blocking under loss).
//              IHMC uses DDS + SRT (two protocols, no cross-protocol priority).
//   telemoq:   Single QUIC connection, per-stream priority. Video degrades gracefully.

/// Track definition: name, priority (lower = higher), publish rate Hz, expected payload bytes.
pub struct TrackDef {
    pub name: &'static str,
    pub priority: u8,
    pub rate_hz: u32,
    #[allow(dead_code)]
    pub payload_bytes: usize,
    pub label: &'static str,
}

// IHMC-informed track hierarchy (validated against DRC & KST architecture):
//
// P0  — Safety: watchdog heartbeat. Absence = connection lost → robot safe mode.
// P1  — Control: SE3 pose targets (KST streaming at 167Hz) + task acknowledgments.
// P2  — Sensors/joints: 100Hz joint state for operator situational awareness.
// P3  — Sensors/force_torque: 100Hz contact detection for weld monitoring.
// P5  — Sensors/imu: 200Hz orientation/accel. First to degrade under congestion.
// P10 — Perception: downsampled point cloud for 3D world model.
// P20 — Video: camera feed. Lowest priority — starved first under congestion.
//
// Key insight from IHMC DRC: 9,600 bps was sufficient for control.
// The robot's onboard 1kHz QP controller handles low-level execution.
// The teleop link carries high-level task goals, not raw joint commands.

pub const TRACKS: &[TrackDef] = &[
    TrackDef { name: "safety/heartbeat",      priority: 0,  rate_hz: 10,  payload_bytes: 17,     label: "safety/heartbeat" },
    TrackDef { name: "control/streaming",     priority: 1,  rate_hz: 167, payload_bytes: 160,    label: "control/streaming" },
    TrackDef { name: "control/task_status",   priority: 1,  rate_hz: 10,  payload_bytes: 14,     label: "control/task_ack" },
    TrackDef { name: "sensors/joints",        priority: 2,  rate_hz: 100, payload_bytes: 120,    label: "sensors/joints" },
    TrackDef { name: "sensors/force_torque",  priority: 3,  rate_hz: 100, payload_bytes: 56,     label: "sensors/force_torque" },
    TrackDef { name: "sensors/imu",           priority: 5,  rate_hz: 200, payload_bytes: 88,     label: "sensors/imu" },
    TrackDef { name: "perception/pointcloud", priority: 10, rate_hz: 5,   payload_bytes: 50_000, label: "perception/ptcloud" },
    TrackDef { name: "video/camera0",         priority: 20, rate_hz: 30,  payload_bytes: 50_000, label: "video/camera0" },
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

pub fn generate_heartbeat(elapsed_ms: u64, sequence: u64) -> Heartbeat {
    Heartbeat {
        timestamp_ms: elapsed_ms,
        sequence,
        estop_engaged: false,
    }
}

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
