use serde::{Deserialize, Serialize};

/// Track definition: name, priority (lower = higher), publish rate Hz, expected payload bytes.
pub struct TrackDef {
    pub name: &'static str,
    pub priority: u8,
    pub rate_hz: u32,
    #[allow(dead_code)]
    pub payload_bytes: usize,
    pub label: &'static str,
}

pub const TRACKS: &[TrackDef] = &[
    TrackDef { name: "control/estop",        priority: 0,  rate_hz: 1,   payload_bytes: 1,      label: "control/estop" },
    TrackDef { name: "control/commands",      priority: 1,  rate_hz: 100, payload_bytes: 56,     label: "control/commands" },
    TrackDef { name: "sensors/joints",        priority: 2,  rate_hz: 100, payload_bytes: 120,    label: "sensors/joints" },
    TrackDef { name: "sensors/force_torque",  priority: 3,  rate_hz: 100, payload_bytes: 56,     label: "sensors/force_torque" },
    TrackDef { name: "sensors/imu",           priority: 5,  rate_hz: 200, payload_bytes: 88,     label: "sensors/imu" },
    TrackDef { name: "video/camera0",         priority: 20, rate_hz: 30,  payload_bytes: 50_000, label: "video/camera0" },
];

// --- Data types ---

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct JointState {
    pub timestamp_ms: u64,
    pub positions: [f64; 7],  // 7-DOF (Franka Panda)
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

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ControlCommand {
    pub timestamp_ms: u64,
    pub target_positions: [f64; 7],
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct EStop {
    pub timestamp_ms: u64,
    pub engaged: bool,
}

// --- Generators: deterministic sinusoidal, physically plausible for Franka Panda ---

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
            // Small oscillation around identity quaternion
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

pub fn generate_command(elapsed_ms: u64) -> ControlCommand {
    let t = elapsed_ms as f64 / 1000.0;
    let mut target_positions = [0.0f64; 7];
    for i in 0..7 {
        let (lo, hi) = JOINT_LIMITS[i];
        let mid = (lo + hi) / 2.0;
        let amp = (hi - lo) / 4.0;
        let freq = 0.1 + 0.05 * i as f64;
        // Command leads actual joint state by 50ms
        target_positions[i] = mid + amp * (2.0 * std::f64::consts::PI * freq * (t + 0.05)).sin();
    }
    ControlCommand { timestamp_ms: elapsed_ms, target_positions }
}

pub fn generate_estop(elapsed_ms: u64) -> EStop {
    EStop { timestamp_ms: elapsed_ms, engaged: false }
}
