#!/usr/bin/env python3
"""Convert an ArmLab dataset export into an on-disk LeRobotDataset directory.

The browser app (Episode tab -> "Download dataset") can only produce JSON: a
static page has no parquet writer and no video encoder, so it exports the
same info/tasks/episodes/frame-table content a LeRobotDataset needs, just not
in LeRobotDataset's on-disk layout. This script does the rest of the
conversion offline, one time, outside the browser.

Usage:
    python scripts/lerobot_export.py armlab-dataset-1234567890.json ./out/my_dataset

Requires pandas and a parquet engine (pyarrow or fastparquet):
    pip install pandas pyarrow

Output layout (matches LeRobotDataset v2.x's directory shape):
    out/my_dataset/
      meta/info.json
      meta/tasks.jsonl
      meta/episodes.jsonl
      data/chunk-000/episode_000000.parquet
      data/chunk-000/episode_000001.parquet
      ...

This does not include camera frames or video: ArmLab is a browser geometry
simulation with no camera pipeline (see the project README), so datasets it
exports are state/action-only. Check the resulting directory against your
installed `lerobot` version's `LeRobotDatasetMetadata` before training -
the on-disk schema has moved between lerobot releases, and this script
targets the commonly-used v2.x shape rather than tracking every revision.
"""
import argparse
import json
import sys
from pathlib import Path

try:
    import pandas as pd
except ImportError:
    print("This script needs pandas and a parquet engine: pip install pandas pyarrow", file=sys.stderr)
    raise


def convert(bundle_path: Path, out_dir: Path, chunk_size: int = 1000) -> None:
    bundle = json.loads(bundle_path.read_text())
    for key in ("info", "tasks", "episodes", "frames"):
        if key not in bundle:
            raise ValueError(f"{bundle_path} is missing '{key}' - is this an armlab-dataset export?")

    meta_dir = out_dir / "meta"
    data_dir = out_dir / "data" / "chunk-000"
    meta_dir.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)

    info = dict(bundle["info"])
    info["chunks_size"] = chunk_size
    info["data_path"] = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2))

    with (meta_dir / "tasks.jsonl").open("w") as f:
        for task in bundle["tasks"]:
            f.write(json.dumps(task) + "\n")

    with (meta_dir / "episodes.jsonl").open("w") as f:
        for episode in bundle["episodes"]:
            f.write(json.dumps(episode) + "\n")

    frames = pd.DataFrame(bundle["frames"])
    for episode in bundle["episodes"]:
        episode_index = episode["episode_index"]
        episode_frames = frames[frames["episode_index"] == episode_index]
        out_path = data_dir / f"episode_{episode_index:06d}.parquet"
        episode_frames.to_parquet(out_path, index=False)

    print(f"Wrote {len(bundle['episodes'])} episode(s), {len(bundle['frames'])} frame(s) to {out_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bundle", type=Path, help="armlab-dataset-*.json downloaded from the app's Episode tab")
    parser.add_argument("out_dir", type=Path, help="Directory to create the LeRobotDataset layout in")
    parser.add_argument("--chunk-size", type=int, default=1000, help="Episodes per data chunk directory (default: 1000)")
    args = parser.parse_args()

    if args.out_dir.exists() and any(args.out_dir.iterdir()):
        parser.error(f"{args.out_dir} already exists and is not empty - pick an empty or new directory")

    convert(args.bundle, args.out_dir, args.chunk_size)


if __name__ == "__main__":
    main()
