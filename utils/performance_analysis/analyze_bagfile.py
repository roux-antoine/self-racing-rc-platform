#!/usr/bin/env python3
"""
Performance analysis tool for autonomous driving bag files.

Computes path-tracking metrics (cross-track error, heading error) and
steering/lateral-control metrics (saturation, jitter, cmd vs feedback)
from a recorded ROS bag file, given a waypoint file for the driven track.

Usage:
    python analyze_bagfile.py \
        --bag-path /path/to/bagfile.bag \
        --waypoints-path /path/to/waypoints.txt
"""

import argparse
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import rosbag

try:
    import tf

    TF_AVAILABLE = True
except ImportError:
    from geometry_utils_pkg import ros_geometry_utils

    TF_AVAILABLE = False


# ---------------------------------------------------------------------------
# Constants (mirror vehicle_models_constants.py)
# ---------------------------------------------------------------------------
STEERING_IDLE_PWM = 90
STEERING_MIN_PWM = 64
STEERING_MAX_PWM = 116
SATURATION_MARGIN = 2  # PWM units from limit to count as "saturated"


# ---------------------------------------------------------------------------
# Data container
# ---------------------------------------------------------------------------
class Record:
    """One time-aligned snapshot of all relevant topics."""

    __slots__ = [
        "t",
        "x",
        "y",
        "yaw",
        "speed",
        "steering_cmd",
        "steering_fbk",
        "throttle_cmd",
        "fix_type",
        "target_curvature",
        "target_speed",
    ]

    def __init__(self) -> None:
        self.t: float = 0.0
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0
        self.speed: float = 0.0
        self.steering_cmd: float = 0.0
        self.steering_fbk: float = 0.0
        self.throttle_cmd: float = 0.0
        self.fix_type: str = ""
        self.target_curvature: Optional[float] = None
        self.target_speed: Optional[float] = None


# ---------------------------------------------------------------------------
# Bag file reading
# ---------------------------------------------------------------------------
TOPICS = [
    "/current_pose",
    "/current_velocity",
    "/arduino_logging",
    "/gps_info",
    "/target_curvature",
    "/target_velocity",
]

ALIGN_TOLERANCE = 0.25  # seconds


def _find_closest(timestamps: List[float], target: float) -> Optional[int]:
    """Return index of closest timestamp >= target, or None if too far."""
    for i, ts in enumerate(timestamps):
        if ts >= target:
            if (ts - target) <= ALIGN_TOLERANCE:
                return i
            return None
    return None


def load_bag(bag_path: str) -> List[Record]:
    """Read a bag file and return time-aligned Records."""

    # Accumulate per-topic
    pose_ts, xs, ys, yaws = [], [], [], []
    vel_ts, speeds = [], []
    ard_ts, steer_cmds, steer_fbks, throttle_cmds = [], [], [], []
    gps_ts, fix_types = [], []
    tc_ts, target_curvatures = [], []
    tv_ts, target_speeds = [], []

    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=TOPICS):
            if topic == "/current_pose":
                quat = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
                if TF_AVAILABLE:
                    _, _, yaw = tf.transformations.euler_from_quaternion(quat)
                else:
                    _, _, yaw = ros_geometry_utils.euler_from_quaternion(quat)
                pose_ts.append(t.to_sec())
                xs.append(msg.pose.position.x)
                ys.append(msg.pose.position.y)
                yaws.append(yaw)

            elif topic == "/current_velocity":
                vel_ts.append(t.to_sec())
                speeds.append(msg.twist.linear.x)

            elif topic == "/arduino_logging":
                ard_ts.append(t.to_sec())
                steer_cmds.append(msg.steering_cmd_final)
                steer_fbks.append(msg.steering_fbk)
                throttle_cmds.append(msg.throttle_cmd_final)

            elif topic == "/gps_info":
                gps_ts.append(t.to_sec())
                fix_types.append(msg.mode_indicator)

            elif topic == "/target_curvature":
                tc_ts.append(t.to_sec())
                target_curvatures.append(msg.data)

            elif topic == "/target_velocity":
                tv_ts.append(t.to_sec())
                target_speeds.append(msg.twist.linear.x)

    has_target_curvature = len(tc_ts) > 0
    has_target_speed = len(tv_ts) > 0

    # Align everything to GPS timestamps
    records: List[Record] = []
    for gi, gps_time in enumerate(gps_ts):
        pi = _find_closest(pose_ts, gps_time)
        vi = _find_closest(vel_ts, gps_time)
        ai = _find_closest(ard_ts, gps_time)
        if pi is None or vi is None or ai is None:
            continue

        r = Record()
        r.t = gps_time
        r.x = xs[pi]
        r.y = ys[pi]
        r.yaw = yaws[pi]
        r.speed = speeds[vi]
        r.steering_cmd = steer_cmds[ai]
        r.steering_fbk = steer_fbks[ai]
        r.throttle_cmd = throttle_cmds[ai]
        r.fix_type = fix_types[gi]

        if has_target_curvature:
            tci = _find_closest(tc_ts, gps_time)
            if tci is not None:
                r.target_curvature = target_curvatures[tci]

        if has_target_speed:
            tvi = _find_closest(tv_ts, gps_time)
            if tvi is not None:
                r.target_speed = target_speeds[tvi]

        records.append(r)

    print(f"Loaded {len(records)} aligned records from {os.path.basename(bag_path)}")
    if has_target_curvature:
        n = sum(1 for r in records if r.target_curvature is not None)
        print(f"  /target_curvature present in {n}/{len(records)} records")
    else:
        print("  /target_curvature not found in bag")
    if has_target_speed:
        n = sum(1 for r in records if r.target_speed is not None)
        print(f"  /target_velocity present in {n}/{len(records)} records")
    else:
        print("  /target_velocity not found in bag")

    return records


# ---------------------------------------------------------------------------
# Waypoint loading
# ---------------------------------------------------------------------------
def load_waypoints(path: str) -> np.ndarray:
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


# ---------------------------------------------------------------------------
# Metrics computation
# ---------------------------------------------------------------------------
def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def compute_path_metrics(
    records: List[Record], waypoints: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    For each record, compute:
      - signed cross-track error (positive = left of path)
      - heading error (radians, wrapped)
      - index of nearest segment

    Returns (cte_array, heading_err_array, seg_idx_array).
    """
    n_segs = len(waypoints) - 1
    # Precompute segment vectors and lengths
    seg_starts = waypoints[:-1]  # (N-1, 2)
    seg_ends = waypoints[1:]
    seg_vecs = seg_ends - seg_starts  # (N-1, 2)
    seg_lens_sq = np.sum(seg_vecs**2, axis=1)  # (N-1,)
    seg_lens_sq[seg_lens_sq == 0] = 1e-12  # avoid div-by-zero for degenerate segments
    seg_angles = np.arctan2(seg_vecs[:, 1], seg_vecs[:, 0])  # tangent angle per segment

    cte = np.zeros(len(records))
    heading_err = np.zeros(len(records))
    seg_idx = np.zeros(len(records), dtype=int)

    for i, r in enumerate(records):
        pt = np.array([r.x, r.y])

        # Project onto each segment: t = dot(pt - start, seg_vec) / |seg_vec|^2
        diffs = pt - seg_starts  # (N-1, 2)
        t_params = np.sum(diffs * seg_vecs, axis=1) / seg_lens_sq
        t_clamped = np.clip(t_params, 0.0, 1.0)

        # Closest point on each segment
        closest = seg_starts + t_clamped[:, np.newaxis] * seg_vecs  # (N-1, 2)
        dists_sq = np.sum((pt - closest) ** 2, axis=1)

        best = np.argmin(dists_sq)
        seg_idx[i] = best

        # Signed CTE: cross product of segment direction with (pt - closest_on_seg)
        # Positive = point is to the left of the path direction
        dx = r.x - closest[best, 0]
        dy = r.y - closest[best, 1]
        seg_dir = seg_vecs[best]
        cross = seg_dir[0] * dy - seg_dir[1] * dx
        cte[i] = math.copysign(math.sqrt(dists_sq[best]), cross)

        # Heading error
        heading_err[i] = _wrap_angle(r.yaw - seg_angles[best])

    return cte, heading_err, seg_idx


def compute_steering_metrics(
    records: List[Record],
) -> Tuple[np.ndarray, np.ndarray, float, float, float]:
    """
    Compute steering-related metrics.

    Returns:
      - times (relative, seconds)
      - steering_rate (d(cmd)/dt, per timestep — length N-1)
      - sat_left_pct (% time near STEERING_MIN_PWM)
      - sat_right_pct (% time near STEERING_MAX_PWM)
      - sat_total_pct
    """
    times = np.array([r.t for r in records])
    times = times - times[0]  # relative
    cmds = np.array([r.steering_cmd for r in records])

    # Rate of change
    dt = np.diff(times)
    dt[dt == 0] = 1e-6
    steering_rate = np.diff(cmds) / dt

    # Saturation
    n = len(cmds)
    sat_left = np.sum(cmds <= STEERING_MIN_PWM + SATURATION_MARGIN) / n * 100
    sat_right = np.sum(cmds >= STEERING_MAX_PWM - SATURATION_MARGIN) / n * 100
    sat_total = sat_left + sat_right

    return times, steering_rate, sat_left, sat_right, sat_total


# ---------------------------------------------------------------------------
# Summary statistics (printed to console)
# ---------------------------------------------------------------------------
def print_summary(
    records: List[Record],
    cte: np.ndarray,
    heading_err: np.ndarray,
    sat_left: float,
    sat_right: float,
    sat_total: float,
    steering_rate: np.ndarray,
) -> None:
    abs_cte = np.abs(cte)
    rms_cte = np.sqrt(np.mean(cte**2))
    he_deg = np.degrees(heading_err)

    print("\n===== PERFORMANCE SUMMARY =====")
    print(f"  Records analysed:       {len(records)}")
    print(f"  Duration:               {records[-1].t - records[0].t:.1f} s")

    print("\n-- Path Tracking --")
    print(f"  Cross-track error (mean):  {np.mean(abs_cte):.3f} m")
    print(f"  Cross-track error (RMS):   {rms_cte:.3f} m")
    print(f"  Cross-track error (max):   {np.max(abs_cte):.3f} m")
    pct_over = np.sum(abs_cte > 0.5) / len(abs_cte) * 100
    print(f"  Time with |CTE| > 0.5 m:  {pct_over:.1f}%")
    print(f"  Heading error (mean):      {np.mean(np.abs(he_deg)):.1f} deg")
    print(f"  Heading error (max):       {np.max(np.abs(he_deg)):.1f} deg")

    # Speed tracking
    target_speeds = [r.target_speed for r in records if r.target_speed is not None]
    if target_speeds:
        actual = [r.speed for r in records if r.target_speed is not None]
        errs = np.array(actual) - np.array(target_speeds)
        print(f"\n-- Speed Tracking --")
        print(f"  Speed error (mean):        {np.mean(errs):.3f} m/s")
        print(f"  Speed error (RMS):         {np.sqrt(np.mean(errs**2)):.3f} m/s")

    print(f"\n-- Steering / Lateral Control --")
    print(f"  Saturation left:           {sat_left:.1f}%")
    print(f"  Saturation right:          {sat_right:.1f}%")
    print(f"  Saturation total:          {sat_total:.1f}%")
    print(f"  Steering rate (mean |dCmd/dt|): {np.mean(np.abs(steering_rate)):.1f} PWM/s")
    print(f"  Steering rate (max |dCmd/dt|):  {np.max(np.abs(steering_rate)):.1f} PWM/s")

    # GPS quality breakdown
    fix_counts: Dict[str, int] = {}
    for r in records:
        fix_counts[r.fix_type] = fix_counts.get(r.fix_type, 0) + 1
    print(f"\n-- GPS Fix Quality --")
    for ft, count in sorted(fix_counts.items(), key=lambda x: -x[1]):
        label = {"F": "RTK Fixed", "R": "RTK Float", "D": "Differential", "A": "Autonomous"}.get(ft, ft)
        print(f"  {label} ({ft}): {count / len(records) * 100:.1f}%")

    print("================================\n")


# ---------------------------------------------------------------------------
# Plotly visualizations
# ---------------------------------------------------------------------------
FIX_TYPE_COLORS = {
    "F": "green",
    "R": "gold",
    "D": "orange",
    "A": "red",
    "N": "gray",
}


def build_figure(
    records: List[Record],
    waypoints: np.ndarray,
    cte: np.ndarray,
    heading_err: np.ndarray,
    steering_rate: np.ndarray,
) -> go.Figure:
    """Build the multi-panel Plotly figure."""

    has_target_speed = any(r.target_speed is not None for r in records)
    has_target_curv = any(r.target_curvature is not None for r in records)

    # Number of rows: trajectory, CTE, heading, speed, steering cmd/fbk, steering rate, GPS fix
    n_rows = 7
    row_heights = [0.30, 0.12, 0.10, 0.12, 0.14, 0.12, 0.04]
    subtitles = [
        "Trajectory (colored by cross-track error)",
        "Cross-Track Error (m)",
        "Heading Error (deg)",
        "Speed: Actual vs Target (m/s)",
        "Steering: Command vs Feedback (PWM)",
        "Steering Command Rate of Change (PWM/s)",
        "GPS Fix Type",
    ]

    fig = make_subplots(
        rows=n_rows,
        cols=1,
        subplot_titles=subtitles,
        row_heights=row_heights,
        vertical_spacing=0.035,
        specs=[[{"secondary_y": False}]] * n_rows,
    )

    times = np.array([r.t for r in records])
    t0 = times[0]
    t_rel = times - t0

    # --- Row 1: Trajectory ---
    # Waypoints as background
    fig.add_trace(
        go.Scatter(
            x=waypoints[:, 0],
            y=waypoints[:, 1],
            mode="lines",
            line={"color": "lightgray", "width": 4},
            name="Waypoints",
            showlegend=True,
        ),
        row=1,
        col=1,
    )
    # Actual path colored by CTE
    xs = [r.x for r in records]
    ys = [r.y for r in records]
    abs_cte = np.abs(cte)
    fig.add_trace(
        go.Scatter(
            x=xs,
            y=ys,
            mode="markers",
            marker={
                "color": abs_cte,
                "colorscale": "RdYlGn_r",
                "size": 4,
                "cmin": 0,
                "cmax": max(1.0, np.percentile(abs_cte, 95)),
                "colorbar": {"title": "|CTE| (m)", "len": 0.25, "y": 0.88},
            },
            name="Actual path",
            showlegend=True,
        ),
        row=1,
        col=1,
    )
    fig.update_xaxes(title_text="X (m)", row=1, col=1)
    fig.update_yaxes(title_text="Y (m)", scaleanchor="x", scaleratio=1, row=1, col=1)

    # --- Row 2: CTE over time ---
    fig.add_trace(
        go.Scatter(
            x=t_rel, y=cte, mode="lines", name="CTE", line={"color": "royalblue", "width": 1}
        ),
        row=2,
        col=1,
    )
    # Threshold bands
    for val in [0.5, -0.5]:
        fig.add_hline(y=val, line_dash="dash", line_color="salmon", line_width=1, row=2, col=1)
    fig.add_hline(y=0, line_color="gray", line_width=0.5, row=2, col=1)
    fig.update_yaxes(title_text="CTE (m)", row=2, col=1)

    # --- Row 3: Heading error ---
    fig.add_trace(
        go.Scatter(
            x=t_rel,
            y=np.degrees(heading_err),
            mode="lines",
            name="Heading error",
            line={"color": "darkorange", "width": 1},
        ),
        row=3,
        col=1,
    )
    fig.add_hline(y=0, line_color="gray", line_width=0.5, row=3, col=1)
    fig.update_yaxes(title_text="Heading err (deg)", row=3, col=1)

    # --- Row 4: Speed ---
    actual_speeds = [r.speed for r in records]
    fig.add_trace(
        go.Scatter(
            x=t_rel,
            y=actual_speeds,
            mode="lines",
            name="Actual speed",
            line={"color": "steelblue", "width": 1},
        ),
        row=4,
        col=1,
    )
    if has_target_speed:
        target_sp = [r.target_speed if r.target_speed is not None else float("nan") for r in records]
        fig.add_trace(
            go.Scatter(
                x=t_rel,
                y=target_sp,
                mode="lines",
                name="Target speed",
                line={"color": "tomato", "width": 1, "dash": "dash"},
            ),
            row=4,
            col=1,
        )
    fig.update_yaxes(title_text="Speed (m/s)", row=4, col=1)

    # --- Row 5: Steering cmd vs feedback ---
    steer_cmds = [r.steering_cmd for r in records]
    steer_fbks = [r.steering_fbk for r in records]
    fig.add_trace(
        go.Scatter(
            x=t_rel,
            y=steer_cmds,
            mode="lines",
            name="Steering cmd",
            line={"color": "crimson", "width": 1},
        ),
        row=5,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=t_rel,
            y=steer_fbks,
            mode="lines",
            name="Steering fbk",
            line={"color": "dodgerblue", "width": 1},
            yaxis="y5",
        ),
        row=5,
        col=1,
    )
    # Saturation limit lines (on cmd axis)
    fig.add_hline(
        y=STEERING_MIN_PWM,
        line_dash="dot",
        line_color="black",
        line_width=1,
        row=5,
        col=1,
        annotation_text="min PWM",
        annotation_position="bottom left",
    )
    fig.add_hline(
        y=STEERING_MAX_PWM,
        line_dash="dot",
        line_color="black",
        line_width=1,
        row=5,
        col=1,
        annotation_text="max PWM",
        annotation_position="top left",
    )
    fig.update_yaxes(title_text="Steering cmd (PWM)", row=5, col=1)

    # --- Row 6: Steering rate ---
    t_rate = (t_rel[:-1] + t_rel[1:]) / 2  # midpoints
    fig.add_trace(
        go.Scatter(
            x=t_rate,
            y=steering_rate,
            mode="lines",
            name="dCmd/dt",
            line={"color": "mediumpurple", "width": 1},
        ),
        row=6,
        col=1,
    )
    fig.add_hline(y=0, line_color="gray", line_width=0.5, row=6, col=1)
    fig.update_yaxes(title_text="dCmd/dt (PWM/s)", row=6, col=1)

    # --- Row 7: GPS fix type timeline ---
    # Render as colored markers along a single horizontal line
    gps_colors = [FIX_TYPE_COLORS.get(r.fix_type, "gray") for r in records]
    fig.add_trace(
        go.Scatter(
            x=t_rel,
            y=[0] * len(records),
            mode="markers",
            marker={"color": gps_colors, "size": 6, "symbol": "square"},
            name="GPS fix",
            hovertext=[r.fix_type for r in records],
            showlegend=False,
        ),
        row=7,
        col=1,
    )
    fig.update_yaxes(visible=False, row=7, col=1)
    fig.update_xaxes(title_text="Time (s)", row=7, col=1)

    # Shared x-axis label only on bottom
    for row in range(2, n_rows):
        fig.update_xaxes(title_text="", row=row, col=1)
    fig.update_xaxes(title_text="Time (s)", row=n_rows, col=1)

    fig.update_layout(
        height=1600,
        title_text="Autonomous Driving Performance Analysis",
        showlegend=True,
        legend={"orientation": "h", "yanchor": "bottom", "y": 1.01, "xanchor": "center", "x": 0.5},
    )

    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Analyze autonomous driving performance from a ROS bag file."
    )
    parser.add_argument("--bag-path", type=str, required=True, help="Path to the ROS bag file")
    parser.add_argument(
        "--waypoints-path",
        type=str,
        required=True,
        help="Path to the waypoint file (space-separated x y [speed])",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output HTML file path (default: <bag_dir>/<bag_name>_analysis.html)",
    )
    args = parser.parse_args()

    # Load data
    records = load_bag(args.bag_path)
    if len(records) < 2:
        print("Error: not enough aligned records to analyze.")
        return

    waypoints = load_waypoints(args.waypoints_path)
    if len(waypoints) < 2:
        print("Error: need at least 2 waypoints.")
        return

    # Compute metrics
    cte, heading_err, seg_idx = compute_path_metrics(records, waypoints)
    times, steering_rate, sat_left, sat_right, sat_total = compute_steering_metrics(records)

    # Print summary
    print_summary(records, cte, heading_err, sat_left, sat_right, sat_total, steering_rate)

    # Build and save figure
    fig = build_figure(records, waypoints, cte, heading_err, steering_rate)

    if args.output:
        out_path = args.output
    else:
        bag_dir = os.path.dirname(os.path.abspath(args.bag_path))
        bag_name = os.path.splitext(os.path.basename(args.bag_path))[0]
        out_path = os.path.join(bag_dir, f"{bag_name}_analysis.html")

    fig.write_html(out_path)
    print(f"Interactive report saved to: {out_path}")


if __name__ == "__main__":
    main()
