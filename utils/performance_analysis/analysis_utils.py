"""Shared utilities for performance analysis scripts."""

import os
from typing import Dict, Optional, Tuple

import numpy as np
from bagfile_loader import BagfileLoader, BagfileRecord


def load_waypoints_from_file(path: str) -> np.ndarray:
    """Load waypoint file (space-separated x y [speed]). Returns Nx2 array of (x, y)."""
    points = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 2:
                points.append((float(parts[0]), float(parts[1])))
    wp = np.array(points)
    print(f"Loaded {len(wp)} waypoints from {os.path.basename(path)}")
    return wp


def resolve_waypoints(
    loader: BagfileLoader, waypoints_path: Optional[str]
) -> Optional[np.ndarray]:
    """Return waypoints from --waypoints-path if given, else from the bag."""
    if waypoints_path:
        return load_waypoints_from_file(waypoints_path)
    if loader.waypoints is not None:
        print(f"Loaded {len(loader.waypoints)} waypoints from bag /waypoints topic")
        return loader.waypoints
    return None


def trim_records(
    records: Dict[float, BagfileRecord],
    start: Optional[float],
    end: Optional[float],
) -> Tuple[Dict[float, BagfileRecord], bool]:
    """
    Trim records to a [start, end] window (relative seconds from bag start).

    Returns (trimmed_records, was_trimmed).
    """
    if start is None and end is None:
        return records, False

    if end is not None and start is not None and end <= start:
        raise ValueError(
            f"End time ({end}s) must be greater than start time ({start}s)"
        )

    bag_t0 = min(records.keys())
    abs_start = bag_t0 + start if start is not None else -float("inf")
    abs_end = bag_t0 + end if end is not None else float("inf")
    trimmed = {t: r for t, r in records.items() if abs_start <= t <= abs_end}
    print(
        f"Trimmed to [{start or 0:.1f}s, {end or '...'!s}s]: " f"{len(trimmed)} records"
    )
    return trimmed, True


def add_time_window_args(parser) -> None:
    """Add --start and --end arguments to an argparse parser."""
    parser.add_argument(
        "--start",
        type=float,
        default=None,
        help="Start time in seconds from the beginning of the bag (discard earlier data)",
    )
    parser.add_argument(
        "--end",
        type=float,
        default=None,
        help="End time in seconds from the beginning of the bag (discard later data)",
    )
