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
import os
from typing import Dict, Optional, Tuple

import numpy as np
import plotly.graph_objs as go
from analysis_utils import add_time_window_args, resolve_waypoints, trim_records
from bagfile_loader import BagfileLoader, BagfileRecord
from geometry_utils_pkg.geometry_utils import compute_cross_track_errors, wrap_angle
from plotly.subplots import make_subplots
from vehicle_models_pkg.vehicle_models_constants import (
    STEERING_MAX_PWM,
    STEERING_MIN_PWM,
)


SATURATION_MARGIN = 2  # PWM units from limit to count as "saturated"


# ---------------------------------------------------------------------------
# Metrics computation
# ---------------------------------------------------------------------------
def compute_path_metrics(
    records: Dict[float, BagfileRecord], waypoints: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    For each record, compute:
      - signed cross-track error (positive = left of path)
      - heading error (radians, wrapped)
      - index of nearest segment

    Returns (cte_array, heading_err_array, seg_idx_array).
    """
    sorted_records = sorted(records.values(), key=lambda r: r.gps_msg_time)
    xs = np.array([r.state.x for r in sorted_records])
    ys = np.array([r.state.y for r in sorted_records])

    cte, seg_idx = compute_cross_track_errors(xs, ys, waypoints)

    # Heading error per segment
    seg_vecs = waypoints[1:] - waypoints[:-1]
    seg_angles = np.arctan2(seg_vecs[:, 1], seg_vecs[:, 0])
    heading_err = np.array(
        [
            wrap_angle(r.state.angle - seg_angles[seg_idx[i]])
            for i, r in enumerate(sorted_records)
        ]
    )

    return cte, heading_err, seg_idx


def compute_steering_metrics(
    records: Dict[float, BagfileRecord],
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
    sorted_records = sorted(records.values(), key=lambda r: r.gps_msg_time)
    times = np.array([r.gps_msg_time for r in sorted_records])
    times = times - times[0]  # relative
    cmds = np.array([r.steering_cmd for r in sorted_records])

    # Rate of change
    dt = np.diff(times)
    dt[dt == 0] = 1e-6
    steering_rate = np.diff(cmds) / dt

    # Saturation
    n = len(cmds)
    if STEERING_MIN_PWM + SATURATION_MARGIN >= STEERING_MAX_PWM - SATURATION_MARGIN:
        raise ValueError("Saturation margin is too large, no valid range remains.")
    sat_left = np.sum(cmds <= STEERING_MIN_PWM + SATURATION_MARGIN) / n * 100
    sat_right = np.sum(cmds >= STEERING_MAX_PWM - SATURATION_MARGIN) / n * 100
    sat_total = sat_left + sat_right

    return times, steering_rate, sat_left, sat_right, sat_total


# ---------------------------------------------------------------------------
# Summary statistics (printed to console)
# ---------------------------------------------------------------------------
def print_summary(
    records: Dict[float, BagfileRecord],
    cte: np.ndarray,
    heading_err: np.ndarray,
    sat_left: float,
    sat_right: float,
    sat_total: float,
    steering_rate: np.ndarray,
) -> None:
    sorted_records = sorted(records.values(), key=lambda r: r.gps_msg_time)
    abs_cte = np.abs(cte)
    rms_cte = np.sqrt(np.mean(cte**2))
    he_deg = np.degrees(heading_err)

    print("\n===== PERFORMANCE SUMMARY =====")
    print(f"  Records analysed:       {len(sorted_records)}")
    duration = sorted_records[-1].gps_msg_time - sorted_records[0].gps_msg_time
    print(f"  Duration:               {duration:.1f} s")

    print("\n-- Path Tracking --")
    print(f"  Cross-track error (mean):  {np.mean(abs_cte):.3f} m")
    print(f"  Cross-track error (RMS):   {rms_cte:.3f} m")
    print(f"  Cross-track error (max):   {np.max(abs_cte):.3f} m")
    print(f"  Heading error (mean):      {np.mean(np.abs(he_deg)):.1f} deg")
    print(f"  Heading error (max):       {np.max(np.abs(he_deg)):.1f} deg")

    # Speed tracking
    records_with_target_speed = [
        r for r in sorted_records if r.target_speed is not None
    ]
    if records_with_target_speed:
        actual = np.array([r.state.vx for r in records_with_target_speed])
        target = np.array([r.target_speed for r in records_with_target_speed])
        errs = actual - target
        print("\n-- Speed Tracking --")
        print(f"  Speed error (mean):        {np.mean(errs):.3f} m/s")
        print(f"  Speed error (RMS):         {np.sqrt(np.mean(errs**2)):.3f} m/s")

    print("\n-- Steering / Lateral Control --")
    print(f"  Saturation left:           {sat_left:.1f}%")
    print(f"  Saturation right:          {sat_right:.1f}%")
    print(f"  Saturation total:          {sat_total:.1f}%")
    print(
        f"  Steering rate (mean |dCmd/dt|): {np.mean(np.abs(steering_rate)):.1f} PWM/s"
    )
    print(
        f"  Steering rate (max |dCmd/dt|):  {np.max(np.abs(steering_rate)):.1f} PWM/s"
    )

    # GPS quality breakdown
    fix_counts: Dict[str, int] = {}
    for r in sorted_records:
        fix_counts[r.fix_type] = fix_counts.get(r.fix_type, 0) + 1
    print("\n-- GPS Fix Quality --")
    for ft, count in sorted(fix_counts.items(), key=lambda x: -x[1]):
        label = {
            "F": "RTK Fixed",
            "R": "RTK Float",
            "D": "Differential",
            "A": "Autonomous",
        }.get(ft, ft)
        print(f"  {label} ({ft}): {count / len(sorted_records) * 100:.1f}%")

    print("================================\n")


# ---------------------------------------------------------------------------
# Plotly visualizations
# ---------------------------------------------------------------------------
FIX_TYPE_COLORS = {
    "R": "green",
    "F": "gold",
    "D": "red",
    "A": "black",
    "N": "gray",
}


def build_figure(
    all_records: Dict[float, BagfileRecord],
    waypoints: np.ndarray,
    cte: np.ndarray,
    heading_err: np.ndarray,
    steering_rate: np.ndarray,
    trim_start: Optional[float] = None,
    trim_end: Optional[float] = None,
) -> go.Figure:
    """Build the multi-panel Plotly figure.

    all_records: full (untrimmed) records from the bag.
    cte/heading_err/steering_rate: computed on trimmed records only.
    trim_start/trim_end: relative times (seconds from bag start) defining the
        active window.  Data outside this window is shown but grayed out.
    """
    sorted_all = sorted(all_records.values(), key=lambda r: r.gps_msg_time)

    has_target_speed = any(r.target_speed is not None for r in sorted_all)

    # Number of rows: trajectory, CTE, heading, speed, steering cmd/fbk, steering rate, engagement, GPS fix
    n_rows = 8
    row_heights = [0.28, 0.11, 0.09, 0.11, 0.13, 0.11, 0.04, 0.04]
    subtitles = [
        "Trajectory (colored by cross-track error)",
        "Cross-Track Error (m)",
        "Heading Error (deg)",
        "Speed: Actual vs Target (m/s)",
        "Steering: Command vs Feedback (PWM)",
        "Steering Command Rate of Change (PWM/s)",
        "Engagement Mode",
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

    all_times = np.array([r.gps_msg_time for r in sorted_all])
    t0 = all_times[0]
    t_rel_all = all_times - t0

    # Build a boolean mask for the active (trimmed) window
    active_start = trim_start if trim_start is not None else 0.0
    active_end = trim_end if trim_end is not None else t_rel_all[-1]
    active_mask = (t_rel_all >= active_start) & (t_rel_all <= active_end)

    # --- Row 1: Trajectory ---
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
    all_xs = np.array([r.state.x for r in sorted_all])
    all_ys = np.array([r.state.y for r in sorted_all])

    # Discarded points in gray
    if not np.all(active_mask):
        fig.add_trace(
            go.Scatter(
                x=all_xs[~active_mask],
                y=all_ys[~active_mask],
                mode="markers",
                marker={"color": "lightgray", "size": 4},
                name="Discarded",
                showlegend=True,
            ),
            row=1,
            col=1,
        )

    # Active points colored by CTE
    abs_cte = np.abs(cte)
    fig.add_trace(
        go.Scatter(
            x=all_xs[active_mask],
            y=all_ys[active_mask],
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
    # Plot over active window only (metrics are only computed there)
    t_rel_active = t_rel_all[active_mask]
    fig.add_trace(
        go.Scatter(
            x=t_rel_active,
            y=cte,
            mode="lines",
            name="CTE",
            line={"color": "royalblue", "width": 1},
        ),
        row=2,
        col=1,
    )
    fig.add_hline(y=0, line_color="gray", line_width=0.5, row=2, col=1)
    fig.update_yaxes(title_text="CTE (m)", row=2, col=1)

    # --- Row 3: Heading error ---
    fig.add_trace(
        go.Scatter(
            x=t_rel_active,
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

    # --- Row 4: Speed (all data, with gray shading for discarded) ---
    actual_speeds = [r.state.vx for r in sorted_all]
    fig.add_trace(
        go.Scatter(
            x=t_rel_all,
            y=actual_speeds,
            mode="lines",
            name="Actual speed",
            line={"color": "steelblue", "width": 1},
        ),
        row=4,
        col=1,
    )
    if has_target_speed:
        target_sp = [
            r.target_speed if r.target_speed is not None else float("nan")
            for r in sorted_all
        ]
        fig.add_trace(
            go.Scatter(
                x=t_rel_all,
                y=target_sp,
                mode="lines",
                name="Target speed",
                line={"color": "tomato", "width": 1, "dash": "dash"},
            ),
            row=4,
            col=1,
        )
    fig.update_yaxes(title_text="Speed (m/s)", row=4, col=1)

    # --- Row 5: Steering cmd vs feedback (all data) ---
    steer_cmds = [r.steering_cmd for r in sorted_all]
    has_steering_fbk = any(r.steering_fbk is not None for r in sorted_all)
    fig.add_trace(
        go.Scatter(
            x=t_rel_all,
            y=steer_cmds,
            mode="lines",
            name="Steering cmd",
            line={"color": "crimson", "width": 1},
        ),
        row=5,
        col=1,
    )
    if has_steering_fbk:
        steer_fbks = [r.steering_fbk for r in sorted_all]
        fig.add_trace(
            go.Scatter(
                x=t_rel_all,
                y=steer_fbks,
                mode="lines",
                name="Steering fbk",
                line={"color": "dodgerblue", "width": 1},
            ),
            row=5,
            col=1,
        )
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

    # --- Row 6: Steering rate (active window only) ---
    t_rate = (t_rel_active[:-1] + t_rel_active[1:]) / 2  # midpoints
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

    # --- Row 7: Engagement mode timeline (all data) ---
    def _engagement_color(r: BagfileRecord) -> str:
        if not r.engaged_mode:
            return "gray"
        if r.override_steering:
            return "blue"
        if r.override_throttle:
            return "pink"
        return "green"

    def _engagement_label(r: BagfileRecord) -> str:
        if not r.engaged_mode:
            return "Disengaged"
        if r.override_steering:
            return "Override steering"
        if r.override_throttle:
            return "Override throttle"
        return "Engaged"

    engagement_colors = [_engagement_color(r) for r in sorted_all]
    engagement_labels = [_engagement_label(r) for r in sorted_all]
    fig.add_trace(
        go.Scatter(
            x=t_rel_all,
            y=[0] * len(sorted_all),
            mode="markers",
            marker={"color": engagement_colors, "size": 6, "symbol": "square"},
            name="Engagement",
            hovertext=engagement_labels,
            showlegend=False,
        ),
        row=7,
        col=1,
    )
    fig.update_yaxes(visible=False, row=7, col=1)

    # --- Row 8: GPS fix type timeline (all data) ---
    gps_colors = [FIX_TYPE_COLORS[r.fix_type] for r in sorted_all]
    fig.add_trace(
        go.Scatter(
            x=t_rel_all,
            y=[0] * len(sorted_all),
            mode="markers",
            marker={"color": gps_colors, "size": 6, "symbol": "square"},
            name="GPS fix",
            hovertext=[r.fix_type for r in sorted_all],
            showlegend=False,
        ),
        row=8,
        col=1,
    )
    fig.update_yaxes(visible=False, row=8, col=1)

    # --- Gray shading for discarded time regions (rows 2-7) ---
    if trim_start is not None or trim_end is not None:
        for row in range(2, n_rows + 1):
            if trim_start is not None and trim_start > 0:
                fig.add_vrect(
                    x0=0,
                    x1=trim_start,
                    fillcolor="gray",
                    opacity=0.15,
                    line_width=0,
                    row=row,
                    col=1,
                )
            if trim_end is not None and trim_end < t_rel_all[-1]:
                fig.add_vrect(
                    x0=trim_end,
                    x1=t_rel_all[-1],
                    fillcolor="gray",
                    opacity=0.15,
                    line_width=0,
                    row=row,
                    col=1,
                )

    # Shared x-axis label only on bottom
    for row in range(2, n_rows):
        fig.update_xaxes(title_text="", row=row, col=1)
    fig.update_xaxes(title_text="Time (s)", row=n_rows, col=1)

    fig.update_layout(
        height=1600,
        title_text="Autonomous Driving Performance Analysis",
        showlegend=True,
        legend={
            "orientation": "h",
            "yanchor": "bottom",
            "y": 1.01,
            "xanchor": "center",
            "x": 0.5,
        },
    )

    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Analyze autonomous driving performance from a ROS bag file."
    )
    parser.add_argument(
        "--bag-path", type=str, required=True, help="Path to the ROS bag file"
    )
    parser.add_argument(
        "--waypoints-path",
        type=str,
        default=None,
        help="Path to the waypoint file (space-separated x y [speed]). "
        "If not provided, waypoints are read from the /waypoints topic in the bag.",
    )
    add_time_window_args(parser)
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output HTML file path (default: <bag_dir>/<bag_name>_analysis.html)",
    )
    args = parser.parse_args()

    # Load data using shared BagfileLoader
    loader = BagfileLoader(args.bag_path)
    all_records = loader.bagfile_records_dicts
    if len(all_records) < 2:
        print("Error: not enough aligned records to analyze.")
        return

    # Trim records to [start, end] window for metrics (plots use all data)
    trimmed_records, _ = trim_records(all_records, args.start, args.end)
    if len(trimmed_records) < 2:
        print("Error: not enough records after trimming.")
        return

    waypoints = resolve_waypoints(loader, args.waypoints_path)
    if waypoints is None:
        print("Error: no waypoints found in bag and --waypoints-path not provided.")
        return

    if len(waypoints) < 2:
        print("Error: need at least 2 waypoints.")
        return

    # Compute metrics on trimmed records only
    cte, heading_err, seg_idx = compute_path_metrics(trimmed_records, waypoints)
    times, steering_rate, sat_left, sat_right, sat_total = compute_steering_metrics(
        trimmed_records
    )

    # Print summary
    print_summary(
        trimmed_records, cte, heading_err, sat_left, sat_right, sat_total, steering_rate
    )

    # Build figure with all data, gray-out discarded sections
    fig = build_figure(
        all_records,
        waypoints,
        cte,
        heading_err,
        steering_rate,
        trim_start=args.start,
        trim_end=args.end,
    )

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
