//! DROID dataset replay reader.
//!
//! Reads the flat replay format produced by `scripts/convert_droid.py`:
//!   meta.json, episode_NNN/telemetry.bin, episode_NNN/cameraN/*.jpg

use std::path::{Path, PathBuf};

use serde::Deserialize;

/// Metadata from meta.json.
#[derive(Deserialize, Debug)]
pub struct ReplayMeta {
    pub fps: u32,
    pub episodes: usize,
    pub total_frames: usize,
    pub dof: u32,
}

/// A single telemetry frame from the dataset.
#[derive(Debug, Clone)]
pub struct TelemetryFrame {
    pub timestamp: f32,
    pub state: [f32; 7],
    pub action: [f32; 7],
}

/// A single replay step: synchronized telemetry + camera frames.
pub struct ReplayStep {
    pub telemetry: TelemetryFrame,
    pub camera0_jpeg: Vec<u8>,
    pub camera1_jpeg: Option<Vec<u8>>,
    pub camera2_jpeg: Option<Vec<u8>>,
}

/// Reads frames from a single episode directory.
struct EpisodeReader {
    episode_dir: PathBuf,
    telemetry: Vec<TelemetryFrame>,
    current: usize,
    all_cameras: bool,
}

impl EpisodeReader {
    fn open(episode_dir: &Path, all_cameras: bool) -> anyhow::Result<Self> {
        let telemetry_path = episode_dir.join("telemetry.bin");
        let data = std::fs::read(&telemetry_path)?;
        let telemetry = parse_telemetry(&data);
        if telemetry.is_empty() {
            anyhow::bail!("empty telemetry in {}", episode_dir.display());
        }
        Ok(Self {
            episode_dir: episode_dir.to_path_buf(),
            telemetry,
            current: 0,
            all_cameras,
        })
    }

    fn next_step(&mut self) -> Option<ReplayStep> {
        if self.current >= self.telemetry.len() {
            return None;
        }
        let telem = self.telemetry[self.current].clone();
        let frame_idx = self.current;
        self.current += 1;

        let cam0 = self.read_jpeg(0, frame_idx);
        let cam1 = if self.all_cameras { Some(self.read_jpeg(1, frame_idx)) } else { None };
        let cam2 = if self.all_cameras { Some(self.read_jpeg(2, frame_idx)) } else { None };

        Some(ReplayStep {
            telemetry: telem,
            camera0_jpeg: cam0,
            camera1_jpeg: cam1,
            camera2_jpeg: cam2,
        })
    }

    fn read_jpeg(&self, camera_idx: usize, frame_idx: usize) -> Vec<u8> {
        let path = self.episode_dir
            .join(format!("camera{camera_idx}"))
            .join(format!("{frame_idx:06}.jpg"));
        std::fs::read(&path).unwrap_or_default()
    }

    fn frame_count(&self) -> usize {
        self.telemetry.len()
    }
}

/// Top-level replay state that iterates through episodes.
pub struct ReplayReader {
    #[allow(dead_code)]
    base_dir: PathBuf,
    meta: ReplayMeta,
    episode_dirs: Vec<PathBuf>,
    current_episode_idx: usize,
    current_reader: Option<EpisodeReader>,
    all_cameras: bool,
    pub loop_episodes: bool,
}

impl ReplayReader {
    /// Open a replay directory and prepare to iterate.
    pub fn open(base_dir: &Path, all_cameras: bool, loop_episodes: bool) -> anyhow::Result<Self> {
        let meta_path = base_dir.join("meta.json");
        let meta_str = std::fs::read_to_string(&meta_path)
            .map_err(|e| anyhow::anyhow!("failed to read {}: {e}", meta_path.display()))?;
        let meta: ReplayMeta = serde_json::from_str(&meta_str)?;

        // Discover episode directories (sorted)
        let mut episode_dirs: Vec<PathBuf> = Vec::new();
        for i in 0..meta.episodes {
            let dir = base_dir.join(format!("episode_{i:03}"));
            if dir.exists() {
                episode_dirs.push(dir);
            }
        }
        if episode_dirs.is_empty() {
            anyhow::bail!("no episode directories found in {}", base_dir.display());
        }

        tracing::info!(
            episodes = episode_dirs.len(),
            total_frames = meta.total_frames,
            fps = meta.fps,
            all_cameras,
            "replay data loaded"
        );

        let mut reader = Self {
            base_dir: base_dir.to_path_buf(),
            meta,
            episode_dirs,
            current_episode_idx: 0,
            current_reader: None,
            all_cameras,
            loop_episodes,
        };
        reader.load_episode(0)?;
        Ok(reader)
    }

    fn load_episode(&mut self, idx: usize) -> anyhow::Result<()> {
        if idx >= self.episode_dirs.len() {
            return Ok(());
        }
        let reader = EpisodeReader::open(&self.episode_dirs[idx], self.all_cameras)?;
        tracing::info!(
            episode = idx,
            frames = reader.frame_count(),
            path = %self.episode_dirs[idx].display(),
            "loaded episode"
        );
        self.current_episode_idx = idx;
        self.current_reader = Some(reader);
        Ok(())
    }

    /// Get the next replay step, advancing episodes as needed.
    pub fn next_step(&mut self) -> Option<ReplayStep> {
        loop {
            if let Some(ref mut reader) = self.current_reader {
                if let Some(step) = reader.next_step() {
                    return Some(step);
                }
            }
            // Current episode exhausted — advance
            let next = self.current_episode_idx + 1;
            if next >= self.episode_dirs.len() {
                if self.loop_episodes {
                    self.load_episode(0).ok()?;
                } else {
                    return None;
                }
            } else {
                self.load_episode(next).ok()?;
            }
        }
    }

    pub fn meta(&self) -> &ReplayMeta {
        &self.meta
    }
}

/// Parse packed telemetry binary: N records x 60 bytes each.
/// Layout per record (little-endian): f32 timestamp + 7xf32 state + 7xf32 action
fn parse_telemetry(data: &[u8]) -> Vec<TelemetryFrame> {
    const RECORD_SIZE: usize = 60; // 15 x f32 = 60 bytes
    data.chunks_exact(RECORD_SIZE)
        .map(|chunk| {
            let mut offset = 0;
            let read_f32 = |o: &mut usize| -> f32 {
                let val = f32::from_le_bytes(chunk[*o..*o + 4].try_into().unwrap());
                *o += 4;
                val
            };
            let timestamp = read_f32(&mut offset);
            let mut state = [0.0f32; 7];
            for s in &mut state {
                *s = read_f32(&mut offset);
            }
            let mut action = [0.0f32; 7];
            for a in &mut action {
                *a = read_f32(&mut offset);
            }
            TelemetryFrame { timestamp, state, action }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_telemetry_roundtrip() {
        let mut data = Vec::new();
        for i in 0..2 {
            let t = i as f32 * 0.0667;
            data.extend_from_slice(&t.to_le_bytes());
            for j in 0..7 {
                let v = (i * 7 + j) as f32 * 0.1;
                data.extend_from_slice(&v.to_le_bytes());
            }
            for j in 0..7 {
                let v = (i * 7 + j) as f32 * -0.05;
                data.extend_from_slice(&v.to_le_bytes());
            }
        }
        assert_eq!(data.len(), 120);
        let frames = parse_telemetry(&data);
        assert_eq!(frames.len(), 2);
        assert!((frames[0].timestamp - 0.0).abs() < 1e-6);
        assert!((frames[1].timestamp - 0.0667).abs() < 1e-4);
    }

    #[test]
    fn parse_telemetry_empty() {
        assert!(parse_telemetry(&[]).is_empty());
    }

    #[test]
    fn parse_telemetry_partial_record_ignored() {
        assert!(parse_telemetry(&vec![0u8; 59]).is_empty());
    }
}
