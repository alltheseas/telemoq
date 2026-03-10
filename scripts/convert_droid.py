#!/usr/bin/env python3
"""Convert lerobot/droid_100 to flat replay format for telemoq.

Output structure:
  <output_dir>/
    meta.json
    episode_000/
      telemetry.bin    # N x 60 bytes: f32 timestamp + 7xf32 state + 7xf32 action
      camera0/000000.jpg, 000001.jpg, ...
      camera1/000000.jpg, ...
      camera2/000000.jpg, ...

Usage:
  pip install -r requirements.txt
  python convert_droid.py --output ../droid_replay
  python convert_droid.py --output ../droid_replay --episodes 3  # quick sample
"""

import argparse
import json
import struct
import sys
from pathlib import Path

import cv2
import numpy as np


def convert_episode(ds, episode_idx: int, output_dir: Path, camera_keys: list[str]):
    """Convert a single episode to flat replay format."""
    ep_dir = output_dir / f"episode_{episode_idx:03d}"

    # Get frames for this episode
    from_idx = ds.episode_data_index["from"][episode_idx].item()
    to_idx = ds.episode_data_index["to"][episode_idx].item()
    n_frames = to_idx - from_idx

    # Write telemetry.bin
    ep_dir.mkdir(parents=True, exist_ok=True)
    telemetry_path = ep_dir / "telemetry.bin"
    with open(telemetry_path, "wb") as f:
        for i in range(from_idx, to_idx):
            item = ds[i]
            timestamp = item["timestamp"].item() if "timestamp" in item else (i - from_idx) / 15.0
            state = item["observation.state"].numpy().astype(np.float32)
            action = item["action"].numpy().astype(np.float32)
            # Pad/truncate to 7 DOF
            state_7 = np.zeros(7, dtype=np.float32)
            action_7 = np.zeros(7, dtype=np.float32)
            state_7[: min(len(state), 7)] = state[: min(len(state), 7)]
            action_7[: min(len(action), 7)] = action[: min(len(action), 7)]
            # Pack: 1 f32 timestamp + 7 f32 state + 7 f32 action = 60 bytes
            f.write(struct.pack("<f", timestamp))
            f.write(state_7.tobytes())
            f.write(action_7.tobytes())

    # Write camera frames as JPEG
    for cam_idx, cam_key in enumerate(camera_keys):
        cam_dir = ep_dir / f"camera{cam_idx}"
        cam_dir.mkdir(parents=True, exist_ok=True)
        for i in range(from_idx, to_idx):
            item = ds[i]
            if cam_key not in item:
                break
            frame_idx = i - from_idx
            img = item[cam_key]
            # Convert PIL/tensor to numpy
            if hasattr(img, "numpy"):
                img = img.numpy()
            if hasattr(img, "convert"):
                img = np.array(img)
            # Ensure uint8 RGB
            if img.dtype == np.float32 or img.dtype == np.float64:
                img = (img * 255).clip(0, 255).astype(np.uint8)
            # Convert channels-first (C,H,W) to channels-last (H,W,C) if needed
            if img.ndim == 3 and img.shape[0] in (1, 3):
                img = np.transpose(img, (1, 2, 0))
            # Convert RGB to BGR for OpenCV
            if img.ndim == 3 and img.shape[2] == 3:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            out_path = cam_dir / f"{frame_idx:06d}.jpg"
            cv2.imwrite(str(out_path), img, [cv2.IMWRITE_JPEG_QUALITY, 85])

    return n_frames


def main():
    parser = argparse.ArgumentParser(description="Convert lerobot/droid_100 to telemoq replay format")
    parser.add_argument("--output", type=str, default="droid_replay", help="Output directory")
    parser.add_argument("--dataset", type=str, default="lerobot/droid_100", help="HuggingFace dataset ID")
    parser.add_argument("--episodes", type=int, default=None, help="Number of episodes to convert (default: all)")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading dataset {args.dataset}...")
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(args.dataset)

    # Discover camera keys
    camera_keys = [k for k in ds.meta.camera_keys] if hasattr(ds.meta, "camera_keys") else []
    if not camera_keys:
        # Fallback: look for observation.images.* or observation.image* keys
        camera_keys = sorted([k for k in ds.features if "image" in k.lower()])
    print(f"Camera keys: {camera_keys}")

    n_episodes = ds.meta.total_episodes if hasattr(ds.meta, "total_episodes") else len(ds.episode_data_index["from"])
    if args.episodes is not None:
        n_episodes = min(args.episodes, n_episodes)

    total_frames = 0
    for ep_idx in range(n_episodes):
        n = convert_episode(ds, ep_idx, output_dir, camera_keys)
        total_frames += n
        print(f"  Episode {ep_idx:03d}: {n} frames")

    # Write meta.json
    meta = {
        "fps": 15,
        "episodes": n_episodes,
        "total_frames": total_frames,
        "dof": 7,
        "dataset": args.dataset,
        "camera_keys": camera_keys,
    }
    with open(output_dir / "meta.json", "w") as f:
        json.dump(meta, f, indent=2)

    print(f"\nDone: {n_episodes} episodes, {total_frames} frames -> {output_dir}")


if __name__ == "__main__":
    main()
