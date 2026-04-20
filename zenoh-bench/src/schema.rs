use serde::{Deserialize, Serialize};

/// Wall-clock timestamp in milliseconds since UNIX epoch.
/// Used by both publisher and subscriber to compute end-to-end latency.
pub fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock before UNIX epoch")
        .as_millis() as u64
}

/// Track definition: name, priority (lower = higher), publish rate Hz, expected payload bytes.
pub struct TrackDef {
    pub name: &'static str,
    pub priority: u8,
    pub rate_hz: u32,
    #[allow(dead_code)]
    pub payload_bytes: usize,
    pub label: &'static str,
}

// IHMC-informed track hierarchy — identical to telemoq for apples-to-apples comparison.
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

// --- Data types (identical to telemoq) ---

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Heartbeat {
    pub timestamp_ms: u64,
    pub sequence: u64,
    pub estop_engaged: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct StreamingCommand {
    pub timestamp_ms: u64,
    pub stream_integration_duration_ms: u16,
    pub has_left_hand: bool,
    pub left_hand_pos: [f64; 3],
    pub left_hand_quat: [f64; 4],
    pub has_right_hand: bool,
    pub right_hand_pos: [f64; 3],
    pub right_hand_quat: [f64; 4],
    pub has_chest: bool,
    pub chest_quat: [f64; 4],
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TaskStatus {
    pub timestamp_ms: u64,
    pub task_id: u32,
    pub progress_pct: u8,
    pub state: u8,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct JointState {
    pub timestamp_ms: u64,
    pub positions: [f64; 7],
    pub velocities: [f64; 7],
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ImuReading {
    pub timestamp_ms: u64,
    pub accel: [f64; 3],
    pub gyro: [f64; 3],
    pub orientation: [f64; 4],
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ForceTorque {
    pub timestamp_ms: u64,
    pub force: [f64; 3],
    pub torque: [f64; 3],
}

// --- Generators (identical to telemoq) ---

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
        stream_integration_duration_ms: 12,
        has_left_hand: true,
        left_hand_pos: [
            0.45 + 0.12 * (0.3 * t).sin(),
            0.22 + 0.08 * (0.2 * t).cos(),
            0.95 + 0.15 * (0.15 * t).sin(),
        ],
        left_hand_quat: {
            let angle = 0.25 * (0.2 * t).sin();
            let half = angle / 2.0;
            [half.cos(), half.sin(), 0.0, 0.0]
        },
        has_right_hand: true,
        right_hand_pos: [
            0.50 + 0.10 * (0.25 * t + 1.0).sin(),
            -0.20 + 0.06 * (0.3 * t + 0.5).cos(),
            0.85 + 0.10 * (0.2 * t + 0.7).sin(),
        ],
        right_hand_quat: {
            let angle = 0.2 * (0.15 * t + 0.5).sin();
            let half = angle / 2.0;
            [half.cos(), 0.0, half.sin(), 0.0]
        },
        has_chest: true,
        chest_quat: {
            let angle = 0.08 * (0.1 * t).sin();
            let half = angle / 2.0;
            [half.cos(), 0.0, 0.0, half.sin()]
        },
    }
}

pub fn generate_task_status(elapsed_ms: u64) -> TaskStatus {
    let cycle_ms: u64 = 10_000;
    let phase = (elapsed_ms % cycle_ms) as f64 / cycle_ms as f64;
    let (state, progress) = if phase < 0.1 {
        (0u8, 0u8)
    } else if phase < 0.9 {
        (1u8, ((phase - 0.1) / 0.8 * 100.0) as u8)
    } else {
        (2u8, 100u8)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heartbeat_serializes_to_expected_size() {
        let hb = generate_heartbeat(0, 0);
        let bytes = bincode::serialize(&hb).expect("serialize heartbeat");
        assert_eq!(bytes.len(), 17);
    }

    #[test]
    fn track_count_matches_telemoq() {
        assert_eq!(TRACKS.len(), 8);
    }

    #[test]
    fn joint_state_within_limits() {
        for ms in [0, 1000, 5000, 10_000, 60_000] {
            let js = generate_joint_state(ms);
            for (i, &pos) in js.positions.iter().enumerate() {
                let (lo, hi) = JOINT_LIMITS[i];
                assert!(pos >= lo && pos <= hi, "joint {i} out of range at t={ms}: {pos}");
            }
        }
    }
}
