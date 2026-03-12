#!/usr/bin/env python3
"""Convert lerobot/droid_100 to flat replay format for telemoq.

Downloads parquet telemetry and AV1 video from HuggingFace, extracts
JPEG frames per camera, writes packed binary telemetry.

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
  python convert_droid.py --output ../sample_data --episodes 3
  python convert_droid.py --output ../droid_replay              # all 100 episodes
"""

import argparse
import json
import struct
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq
from huggingface_hub import hf_hub_download

CAMERA_KEYS = [
    "observation.images.wrist_image_left",
    "observation.images.exterior_image_1_left",
    "observation.images.exterior_image_2_left",
]


def main():
    parser = argparse.ArgumentParser(description="Convert lerobot/droid_100 to telemoq replay format")
    parser.add_argument("--output", type=str, default="droid_replay", help="Output directory")
    parser.add_argument("--dataset", type=str, default="lerobot/droid_100", help="HuggingFace dataset ID")
    parser.add_argument("--episodes", type=int, default=None, help="Number of episodes to convert (default: all)")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    repo = args.dataset

    # Download parquet data
    print("Downloading parquet telemetry...")
    parquet_path = hf_hub_download(repo, "data/chunk-000/file-000.parquet", repo_type="dataset")

    print("Reading telemetry...")
    table = pq.read_table(parquet_path)
    ep_indices = table.column("episode_index").to_pylist()
    timestamps = table.column("timestamp").to_pylist()
    states = table.column("observation.state").to_pylist()
    actions = table.column("action").to_pylist()

    # Build episode boundary index
    episode_ranges = {}
    for i, ep_idx in enumerate(ep_indices):
        if ep_idx not in episode_ranges:
            episode_ranges[ep_idx] = [i, i]
        episode_ranges[ep_idx][1] = i + 1

    n_episodes = len(episode_ranges)
    if args.episodes is not None:
        n_episodes = min(args.episodes, n_episodes)
    print(f"Found {len(episode_ranges)} episodes, converting {n_episodes}")

    # Download video files
    video_paths = {}
    for cam_key in CAMERA_KEYS:
        vpath = f"videos/{cam_key}/chunk-000/file-000.mp4"
        print(f"Downloading {vpath}...")
        video_paths[cam_key] = hf_hub_download(repo, vpath, repo_type="dataset")

    total_frames = 0

    for ep_idx in range(n_episodes):
        from_row, to_row = episode_ranges[ep_idx]
        n_frames = to_row - from_row
        ep_dir = output_dir / f"episode_{ep_idx:03d}"
        ep_dir.mkdir(parents=True, exist_ok=True)

        # Write telemetry.bin (60 bytes per frame)
        tel_path = ep_dir / "telemetry.bin"
        with open(tel_path, "wb") as f:
            for row_idx in range(from_row, to_row):
                ts = float(timestamps[row_idx])
                state = np.array(states[row_idx], dtype=np.float32)
                action = np.array(actions[row_idx], dtype=np.float32)
                s7 = np.zeros(7, dtype=np.float32)
                a7 = np.zeros(7, dtype=np.float32)
                s7[: min(len(state), 7)] = state[: min(len(state), 7)]
                a7[: min(len(action), 7)] = action[: min(len(action), 7)]
                f.write(struct.pack("<f", ts))
                f.write(s7.tobytes())
                f.write(a7.tobytes())

        # Extract video frames as JPEG
        for cam_idx, cam_key in enumerate(CAMERA_KEYS):
            cam_dir = ep_dir / f"camera{cam_idx}"
            cam_dir.mkdir(parents=True, exist_ok=True)

            cap = cv2.VideoCapture(video_paths[cam_key])
            if not cap.isOpened():
                print(f"  WARNING: Could not open video for {cam_key}")
                continue

            cap.set(cv2.CAP_PROP_POS_FRAMES, from_row)
            for frame_i in range(n_frames):
                ret, frame = cap.read()
                if not ret:
                    print(f"  WARNING: Failed to read frame {frame_i} from {cam_key}")
                    break
                out_path = cam_dir / f"{frame_i:06d}.jpg"
                cv2.imwrite(str(out_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            cap.release()

        total_frames += n_frames
        print(f"  Episode {ep_idx:03d}: {n_frames} frames")

    # Write meta.json
    meta = {
        "fps": 15,
        "episodes": n_episodes,
        "total_frames": total_frames,
        "dof": 7,
        "dataset": repo,
        "camera_keys": CAMERA_KEYS,
    }
    with open(output_dir / "meta.json", "w") as f:
        json.dump(meta, f, indent=2)

    print(f"\nDone: {n_episodes} episodes, {total_frames} frames -> {output_dir}")


if __name__ == "__main__":
    main()
