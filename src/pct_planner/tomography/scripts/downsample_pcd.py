#!/usr/bin/env python3
"""Prepare a FAST-LIO PCD for PCT without machine-specific paths."""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def xyz_triplet(value):
    try:
        result = tuple(float(item) for item in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected x,y,z") from exc
    if len(result) != 3 or not np.isfinite(result).all():
        raise argparse.ArgumentTypeError("expected three finite values: x,y,z")
    return result


def parse_args():
    parser = argparse.ArgumentParser(
        description="Crop, denoise and voxel-downsample a PCD for PCT tomography."
    )
    parser.add_argument("input", type=Path, help="FAST-LIO scans.pcd or another input PCD")
    parser.add_argument("output", type=Path, help="output PCD; existing files require --force")
    parser.add_argument("--voxel-size", type=float, default=0.05, help="voxel size in metres")
    parser.add_argument("--min-bound", type=xyz_triplet, help="optional crop minimum x,y,z")
    parser.add_argument("--max-bound", type=xyz_triplet, help="optional crop maximum x,y,z")
    parser.add_argument("--remove-outliers", action="store_true",
                        help="apply statistical outlier removal after cropping")
    parser.add_argument("--neighbors", type=int, default=30, help="outlier-filter neighbours")
    parser.add_argument("--std-ratio", type=float, default=2.0, help="outlier-filter std ratio")
    parser.add_argument("--visualize", action="store_true", help="open an Open3D result window")
    parser.add_argument("--force", action="store_true", help="overwrite the output file")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.voxel_size <= 0.0:
        raise SystemExit("--voxel-size must be positive")
    if args.neighbors < 2 or args.std_ratio <= 0.0:
        raise SystemExit("--neighbors must be >= 2 and --std-ratio must be positive")
    if bool(args.min_bound) != bool(args.max_bound):
        raise SystemExit("--min-bound and --max-bound must be supplied together")
    if not args.input.is_file():
        raise SystemExit(f"input PCD does not exist: {args.input}")
    if args.output.exists() and not args.force:
        raise SystemExit(f"output already exists: {args.output} (use --force to overwrite)")

    cloud = o3d.io.read_point_cloud(str(args.input))
    original_count = len(cloud.points)
    if original_count == 0:
        raise SystemExit(f"input PCD contains no points: {args.input}")

    if args.min_bound:
        min_bound = np.asarray(args.min_bound, dtype=float)
        max_bound = np.asarray(args.max_bound, dtype=float)
        if np.any(min_bound >= max_bound):
            raise SystemExit("every --min-bound value must be smaller than --max-bound")
        cloud = cloud.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound))

    if args.remove_outliers and len(cloud.points):
        cloud, _ = cloud.remove_statistical_outlier(
            nb_neighbors=args.neighbors, std_ratio=args.std_ratio
        )

    cloud = cloud.voxel_down_sample(voxel_size=args.voxel_size)
    if not len(cloud.points):
        raise SystemExit("all points were removed; check crop bounds and filter settings")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), cloud, write_ascii=False):
        raise SystemExit(f"failed to write: {args.output}")

    reduction = 100.0 * (1.0 - len(cloud.points) / original_count)
    print(f"input points : {original_count:,}")
    print(f"output points: {len(cloud.points):,} ({reduction:.1f}% reduction)")
    print(f"saved        : {args.output.resolve()}")

    if args.visualize:
        o3d.visualization.draw_geometries([cloud], window_name="PCT input cloud")


if __name__ == "__main__":
    main()
