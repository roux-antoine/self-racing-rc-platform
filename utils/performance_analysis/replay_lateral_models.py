#!/usr/bin/env python3
"""
Replay lateral control with different vehicle models.

Given a bag file containing /target_curvature and /current_velocity,
recomputes the steering command that each vehicle model would have produced,
and optionally forward-simulates the resulting trajectory.

This lets you compare models (V0, V1, V2, V3) against what was actually
commanded, to decide which model best fits reality or to evaluate a model
change before deploying it.

Usage:
    python replay_lateral_models.py \
        --bag-path /path/to/bagfile.bag \
        --waypoints-path /path/to/waypoints.txt \
        --models V0 V1 V2 V3
"""

import argparse
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import plotly.graph_objs as go
from analysis_utils import add_time_window_args, resolve_waypoints, trim_records
from bagfile_loader import BagfileLoader, BagfileRecord
from geometry_utils_pkg.geometry_utils import wrap_angle
from plotly.subplots import make_subplots
from vehicle_models_pkg.vehicle_models import (
    CarModelBicyclePure,
    CarModelBicycleV0,
    CarModelBicycleV1,
    CarModelBicycleV2,
    CarModelBicycleV3,
)
from vehicle_models_pkg.vehicle_models_constants import (
    STEERING_IDLE_PWM,
    STEERING_MAX_PWM,
    STEERING_MIN_PWM,
)


MODEL_REGISTRY: Dict[str, type] = {
    "V0": CarModelBicycleV0,
    "V1": CarModelBicycleV1,
    "V2": CarModelBicycleV2,
    "V3": CarModelBicycleV3,
}

MODEL_COLORS = {
    "V0": "orange",
    "V1": "green",
    "V2": "dodgerblue",
    "V3": "mediumpurple",
}


# ---------------------------------------------------------------------------
# Steering command recomputation
# ---------------------------------------------------------------------------
def recompute_steering_commands(
    sorted_records: List[BagfileRecord], model: CarModelBicyclePure
) -> np.ndarray:
    """
    For each record, recompute the steering PWM command that `model` would
    produce given the recorded target_curvature and speed.

    Mirrors the logic in lateral_controller.py:loop().
    """
    cmds = np.full(len(sorted_records), float("nan"))
    for i, r in enumerate(sorted_records):
        if r.target_curvature is None:
            continue
        if abs(r.target_curvature) < 1e-3:
            cmd = STEERING_IDLE_PWM
        else:
            cmd = model.compute_steering_command_from_radius(
                radius=1.0 / r.target_curvature,
                speed=r.state.vx,
            )
        # Clamp to PWM bounds (same as the real controller)
        cmd = max(STEERING_MIN_PWM, min(STEERING_MAX_PWM, cmd))
        cmds[i] = cmd
    return cmds


# ---------------------------------------------------------------------------
# Forward simulation
# ---------------------------------------------------------------------------
def forward_simulate_open_loop(
    sorted_records: List[BagfileRecord], model: CarModelBicyclePure
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Fully open-loop: start from the first record's pose and integrate forward
    without ever resetting to the actual GPS position.

    Shows: "given what the car actually commanded, where does this model
    think the car would end up?"

    Returns (xs, ys, angles).
    """
    model.states.clear()
    r0 = sorted_records[0]
    model.init(x=r0.state.x, y=r0.state.y, vx=r0.state.vx, angle=r0.state.angle)

    for i in range(1, len(sorted_records)):
        dt = sorted_records[i].gps_msg_time - sorted_records[i - 1].gps_msg_time
        if dt <= 0:
            raise ValueError(f"Non-positive dt between records {i-1} and {i}")

        model.states[-1].vx = sorted_records[i - 1].state.vx
        model.step(dt=dt, cmd_steering=sorted_records[i - 1].steering_cmd)

    xs = np.array([s.x for s in model.states])
    ys = np.array([s.y for s in model.states])
    angles = np.array([s.angle for s in model.states])
    return xs, ys, angles


def forward_simulate_one_step(
    sorted_records: List[BagfileRecord], model: CarModelBicyclePure
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    One-step prediction: at each timestep, reset the model to the actual
    recorded pose, then step once. The predicted position shows how well the
    model predicts the *next* GPS fix given the current state and command.

    Returns (xs, ys, angles).
    """
    xs = np.full(len(sorted_records), float("nan"))
    ys = np.full(len(sorted_records), float("nan"))
    angles = np.full(len(sorted_records), float("nan"))

    for i in range(len(sorted_records) - 1):
        dt = sorted_records[i + 1].gps_msg_time - sorted_records[i].gps_msg_time
        if dt <= 0:
            raise ValueError(f"Non-positive dt between records {i-1} and {i}")

        r = sorted_records[i]
        model.states.clear()
        model.init(x=r.state.x, y=r.state.y, vx=r.state.vx, angle=r.state.angle)
        model.step(dt=dt, cmd_steering=r.steering_cmd)

        xs[i + 1] = model.states[-1].x
        ys[i + 1] = model.states[-1].y
        angles[i + 1] = model.states[-1].angle

    return xs, ys, angles


# ---------------------------------------------------------------------------
# Pairwise offset metrics
# ---------------------------------------------------------------------------
def compute_pairwise_offsets(
    sorted_records: List[BagfileRecord],
    sim_xs: np.ndarray,
    sim_ys: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute the signed lateral offset between each simulated position and the
    corresponding actual GPS position, along with the actual speed at each point.

    Sign convention (same as CTE): positive = simulated point is to the left
    of the travel direction, negative = to the right.  Determined by the cross
    product of the heading vector and the (actual → simulated) vector.
    """
    actual_xs = np.array([r.state.x for r in sorted_records])
    actual_ys = np.array([r.state.y for r in sorted_records])
    speeds = np.array([r.state.vx for r in sorted_records])
    headings = np.array([r.state.angle for r in sorted_records])

    dx = sim_xs - actual_xs
    dy = sim_ys - actual_ys
    dist = np.sqrt(dx**2 + dy**2)

    # Cross product of heading unit vector × offset vector gives signed lateral
    cross = np.cos(headings) * dy - np.sin(headings) * dx
    offsets = np.sign(cross) * dist

    # Preserve NaNs from simulation (e.g. one_step leaves index 0 as NaN)
    return offsets, speeds


def compute_yaw_offsets(
    sorted_records: List[BagfileRecord],
    sim_angles: np.ndarray,
) -> np.ndarray:
    """
    Compute the signed yaw offset (predicted angle - actual angle), wrapped
    to [-pi, pi].  Positive = model predicts more counter-clockwise (left),
    negative = more clockwise (right).
    """
    actual_angles = np.array([r.state.angle for r in sorted_records])
    yaw_offsets = np.array(
        [wrap_angle(s - a) for s, a in zip(sim_angles, actual_angles)]
    )
    return yaw_offsets


# ---------------------------------------------------------------------------
# Summary statistics
# ---------------------------------------------------------------------------
def print_summary(
    sorted_records: List[BagfileRecord],
    model_names: List[str],
    model_cmds: Dict[str, np.ndarray],
    model_offsets: Dict[str, Tuple[np.ndarray, np.ndarray]] = {},
    model_yaw_offsets: Dict[str, np.ndarray] = {},
) -> None:
    actual = np.array([r.steering_cmd for r in sorted_records])

    print("\n===== LATERAL MODEL COMPARISON SUMMARY =====")
    print(f"  Records:  {len(sorted_records)}")
    duration = sorted_records[-1].gps_msg_time - sorted_records[0].gps_msg_time
    print(f"  Duration: {duration:.1f} s")

    for name in model_names:
        cmds = model_cmds[name]
        valid = ~np.isnan(cmds)
        if not np.any(valid):
            print(f"\n  {name}: no valid target_curvature data")
            continue
        diff = cmds[valid] - actual[valid]
        print(f"\n  {name}:")
        print(f"    Steering cmd error (mean):  {np.mean(diff):+.2f} PWM")
        print(f"    Steering cmd error (RMS):   {np.sqrt(np.mean(diff**2)):.2f} PWM")
        print(f"    Steering cmd error (max):   {np.max(np.abs(diff)):.2f} PWM")
        corr = np.corrcoef(actual[valid], cmds[valid])[0, 1]
        print(f"    Correlation with actual:    {corr:.4f}")

        if name in model_offsets:
            offsets, speeds = model_offsets[name]
            valid_offsets = offsets[~np.isnan(offsets)]
            if len(valid_offsets) > 0:
                print(
                    f"    Pairwise offset (mean):     {np.mean(valid_offsets):+.4f} m  (+left/-right)"
                )
                print(
                    f"    Pairwise |offset| (mean):   {np.mean(np.abs(valid_offsets)):.4f} m"
                )
                print(
                    f"    Pairwise offset (RMS):      {np.sqrt(np.mean(valid_offsets**2)):.4f} m"
                )
                print(
                    f"    Pairwise |offset| (max):    {np.max(np.abs(valid_offsets)):.4f} m"
                )
                speed_threshold = 1.0
                fast_mask = (~np.isnan(offsets)) & (speeds > speed_threshold)
                if np.any(fast_mask):
                    fast_offsets = offsets[fast_mask]
                    print(
                        f"    Pairwise offset (mean, >{speed_threshold} m/s): {np.mean(fast_offsets):.4f} m"
                    )

        if name in model_yaw_offsets:
            yaw = model_yaw_offsets[name]
            valid_yaw = yaw[~np.isnan(yaw)]
            if len(valid_yaw) > 0:
                yaw_deg = np.degrees(valid_yaw)
                print(
                    f"    Yaw offset (mean):          {np.mean(yaw_deg):+.2f} deg  (+left/-right)"
                )
                print(
                    f"    Yaw |offset| (mean):        {np.mean(np.abs(yaw_deg)):.2f} deg"
                )
                print(
                    f"    Yaw offset (RMS):           {np.sqrt(np.mean(yaw_deg**2)):.2f} deg"
                )
                print(
                    f"    Yaw |offset| (max):         {np.max(np.abs(yaw_deg)):.2f} deg"
                )

    print("=============================================\n")


# ---------------------------------------------------------------------------
# Plotly figure
# ---------------------------------------------------------------------------
def build_figure(
    all_sorted_records: List[BagfileRecord],
    waypoints,
    model_names: List[str],
    model_cmds: Dict[str, np.ndarray],
    model_trajectories: Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]],
    model_offsets: Dict[str, Tuple[np.ndarray, np.ndarray]] = {},
    model_yaw_offsets: Dict[str, np.ndarray] = {},
    trim_start: Optional[float] = None,
    trim_end: Optional[float] = None,
) -> go.Figure:
    """Build the multi-panel Plotly figure.

    all_sorted_records: full (untrimmed) sorted records from the bag.
    model_cmds/model_trajectories/model_offsets/model_yaw_offsets: computed
        on trimmed records only.
    trim_start/trim_end: relative times (seconds from bag start) defining the
        active window.  Data outside this window is shown but grayed out.
    """
    has_trajectories = len(model_trajectories) > 0
    has_offsets = len(model_offsets) > 0
    has_yaw_offsets = len(model_yaw_offsets) > 0

    # Base rows: steering cmds, residuals, context, x-y path
    n_rows = 4
    row_heights = [0.30, 0.25, 0.20, 0.50]
    subtitles = [
        "Steering Command: Actual vs Models (PWM)",
        "Steering Command Residual: Model - Actual (PWM)",
        "Target Curvature & Speed Context",
        "Actual Path (X-Y)",
    ]
    if has_offsets:
        n_rows += 1
        row_heights.append(0.20)
        subtitles.append("Pairwise Offset: Simulated vs Actual (m)")
        n_rows += 1
        row_heights.append(0.30)
        subtitles.append("Offset vs Speed")
    if has_yaw_offsets:
        n_rows += 1
        row_heights.append(0.20)
        subtitles.append("Yaw Offset: Simulated vs Actual (deg)")
    if has_trajectories:
        n_rows += 1
        row_heights.append(0.50)
        subtitles.append("Forward-Simulated Trajectories")
    # Normalize row heights to sum to 1
    total = sum(row_heights)
    row_heights = [h / total for h in row_heights]

    fig = make_subplots(
        rows=n_rows,
        cols=1,
        subplot_titles=subtitles,
        row_heights=row_heights,
        vertical_spacing=0.05,
    )

    all_times = np.array([r.gps_msg_time for r in all_sorted_records])
    t0 = all_times[0]
    t_rel_all = all_times - t0

    # Build a boolean mask for the active (trimmed) window
    active_start = trim_start if trim_start is not None else 0.0
    active_end = trim_end if trim_end is not None else t_rel_all[-1]
    active_mask = (t_rel_all >= active_start) & (t_rel_all <= active_end)
    t_rel_active = t_rel_all[active_mask]

    actual_cmds_all = np.array([r.steering_cmd for r in all_sorted_records])

    # Legend name helper: row 1 → "legend", row N → "legendN"
    def _legend_for_row(row: int) -> str:
        return "legend" if row == 1 else f"legend{row}"

    # --- Row 1: Steering commands (all data + model overlays on active) ---
    fig.add_trace(
        go.Scatter(
            x=t_rel_all,
            y=actual_cmds_all,
            mode="lines",
            name="Actual",
            line={"color": "black", "width": 2},
            legend=_legend_for_row(1),
        ),
        row=1,
        col=1,
    )
    for name in model_names:
        fig.add_trace(
            go.Scatter(
                x=t_rel_active,
                y=model_cmds[name],
                mode="lines",
                name=name,
                line={"color": MODEL_COLORS.get(name, "gray"), "width": 1.5},
                legend=_legend_for_row(1),
            ),
            row=1,
            col=1,
        )
    fig.add_hline(
        y=STEERING_MIN_PWM,
        line_dash="dot",
        line_color="gray",
        row=1,
        col=1,
        annotation_text="min",
        annotation_position="bottom left",
    )
    fig.add_hline(
        y=STEERING_MAX_PWM,
        line_dash="dot",
        line_color="gray",
        row=1,
        col=1,
        annotation_text="max",
        annotation_position="top left",
    )
    fig.update_yaxes(title_text="Steering cmd (PWM)", row=1, col=1)

    # --- Row 2: Residuals (active window only) ---
    actual_cmds_active = actual_cmds_all[active_mask]
    for name in model_names:
        residual = model_cmds[name] - actual_cmds_active
        fig.add_trace(
            go.Scatter(
                x=t_rel_active,
                y=residual,
                mode="lines",
                name=f"{name}",
                line={"color": MODEL_COLORS.get(name, "gray"), "width": 1},
                legend=_legend_for_row(2),
            ),
            row=2,
            col=1,
        )
    fig.add_hline(y=0, line_color="black", line_width=0.5, row=2, col=1)
    fig.update_yaxes(title_text="Model - Actual (PWM)", row=2, col=1)

    # --- Row 3: Context (curvature and speed, all data) ---
    curvatures = [
        r.target_curvature if r.target_curvature is not None else float("nan")
        for r in all_sorted_records
    ]
    speeds = [r.state.vx for r in all_sorted_records]
    fig.add_trace(
        go.Scatter(
            x=t_rel_all,
            y=curvatures,
            mode="lines",
            name="Target curvature",
            line={"color": "tomato", "width": 1},
            legend=_legend_for_row(3),
        ),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=t_rel_all,
            y=speeds,
            mode="lines",
            name="Speed",
            line={"color": "steelblue", "width": 1},
            legend=_legend_for_row(3),
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Curvature (1/m) / Speed (m/s)", row=3, col=1)

    # --- Row 4: Actual X-Y path ---
    xy_row = 4
    if waypoints is not None:
        fig.add_trace(
            go.Scatter(
                x=waypoints[:, 0],
                y=waypoints[:, 1],
                mode="lines",
                line={"color": "lightgray", "width": 4},
                name="Waypoints",
                legend=_legend_for_row(xy_row),
            ),
            row=xy_row,
            col=1,
        )
    all_xs = np.array([r.state.x for r in all_sorted_records])
    all_ys = np.array([r.state.y for r in all_sorted_records])
    # Discarded points in gray
    if not np.all(active_mask):
        fig.add_trace(
            go.Scatter(
                x=all_xs[~active_mask],
                y=all_ys[~active_mask],
                mode="markers",
                marker={"color": "lightgray", "size": 4},
                name="Discarded",
                legend=_legend_for_row(xy_row),
            ),
            row=xy_row,
            col=1,
        )
    fig.add_trace(
        go.Scatter(
            x=all_xs[active_mask],
            y=all_ys[active_mask],
            mode="lines",
            name="Actual path",
            line={"color": "black", "width": 2},
            legend=_legend_for_row(xy_row),
        ),
        row=xy_row,
        col=1,
    )
    fig.update_xaxes(title_text="X (m)", row=xy_row, col=1)
    fig.update_yaxes(
        title_text="Y (m)", scaleanchor=f"x{xy_row}", scaleratio=1, row=xy_row, col=1
    )

    # --- Pairwise offset row (if simulated, active window only) ---
    if has_offsets:
        offset_row = 5
        for name in model_names:
            if name in model_offsets:
                offsets, speeds = model_offsets[name]
                fig.add_trace(
                    go.Scatter(
                        x=t_rel_active,
                        y=offsets,
                        mode="lines",
                        name=f"{name} offset",
                        line={"color": MODEL_COLORS.get(name, "gray"), "width": 1.5},
                        legend=_legend_for_row(offset_row),
                    ),
                    row=offset_row,
                    col=1,
                )
        # Horizontal dotted lines at mean offset per model
        for name in model_names:
            if name in model_offsets:
                offsets, _ = model_offsets[name]
                valid_offsets = offsets[~np.isnan(offsets)]
                if len(valid_offsets) > 0:
                    mean_val = float(np.mean(np.abs(valid_offsets)))
                    fig.add_trace(
                        go.Scatter(
                            x=[t_rel_active[0], t_rel_active[-1]],
                            y=[mean_val, mean_val],
                            mode="lines",
                            name=f"{name} mean |offset|: {mean_val:.4f} m",
                            line={
                                "color": MODEL_COLORS.get(name, "gray"),
                                "width": 1.5,
                                "dash": "dot",
                            },
                            legend=_legend_for_row(offset_row),
                        ),
                        row=offset_row,
                        col=1,
                    )
        fig.add_hline(y=0, line_color="gray", line_width=0.5, row=offset_row, col=1)
        fig.update_yaxes(title_text="Offset (m, +left/-right)", row=offset_row, col=1)
        fig.update_xaxes(title_text="Time (s)", row=offset_row, col=1)

        # --- Offset vs Speed scatter ---
        speed_offset_row = offset_row + 1
        for name in model_names:
            if name in model_offsets:
                offsets, speeds = model_offsets[name]
                valid = ~np.isnan(offsets)
                fig.add_trace(
                    go.Scatter(
                        x=speeds[valid],
                        y=offsets[valid],
                        mode="markers",
                        name=f"{name}",
                        marker={
                            "color": MODEL_COLORS.get(name, "gray"),
                            "size": 4,
                            "opacity": 0.6,
                        },
                        legend=_legend_for_row(speed_offset_row),
                    ),
                    row=speed_offset_row,
                    col=1,
                )
        fig.update_xaxes(title_text="Speed (m/s)", row=speed_offset_row, col=1)
        fig.update_yaxes(title_text="Offset (m)", row=speed_offset_row, col=1)

    # --- Yaw Offset over time ---
    next_row = 5 + (2 if has_offsets else 0)
    if has_yaw_offsets:
        yaw_row = next_row
        next_row += 1
        for name in model_names:
            if name in model_yaw_offsets:
                yaw = np.degrees(model_yaw_offsets[name])
                fig.add_trace(
                    go.Scatter(
                        x=t_rel_active,
                        y=yaw,
                        mode="lines",
                        name=f"{name}",
                        line={"color": MODEL_COLORS.get(name, "gray"), "width": 1.5},
                        legend=_legend_for_row(yaw_row),
                    ),
                    row=yaw_row,
                    col=1,
                )
        # Mean |yaw offset| lines
        for name in model_names:
            if name in model_yaw_offsets:
                yaw = np.degrees(model_yaw_offsets[name])
                valid_yaw = yaw[~np.isnan(yaw)]
                if len(valid_yaw) > 0:
                    mean_val = float(np.mean(np.abs(valid_yaw)))
                    fig.add_trace(
                        go.Scatter(
                            x=[t_rel_active[0], t_rel_active[-1]],
                            y=[mean_val, mean_val],
                            mode="lines",
                            name=f"{name} mean |yaw|: {mean_val:.2f} deg",
                            line={
                                "color": MODEL_COLORS.get(name, "gray"),
                                "width": 1.5,
                                "dash": "dot",
                            },
                            legend=_legend_for_row(yaw_row),
                        ),
                        row=yaw_row,
                        col=1,
                    )
        fig.add_hline(y=0, line_color="gray", line_width=0.5, row=yaw_row, col=1)
        fig.update_yaxes(
            title_text="Yaw offset (deg, +left/-right)", row=yaw_row, col=1
        )
        fig.update_xaxes(title_text="Time (s)", row=yaw_row, col=1)

    # --- Simulated Trajectories (if simulated) ---
    sim_row = next_row
    if has_trajectories:
        if waypoints is not None:
            fig.add_trace(
                go.Scatter(
                    x=waypoints[:, 0],
                    y=waypoints[:, 1],
                    mode="lines",
                    line={"color": "lightgray", "width": 4},
                    name="Waypoints",
                    legend=_legend_for_row(sim_row),
                ),
                row=sim_row,
                col=1,
            )
        fig.add_trace(
            go.Scatter(
                x=all_xs[active_mask],
                y=all_ys[active_mask],
                mode="lines+markers",
                name="Actual path",
                line={"color": "black", "width": 2},
                marker={"size": 6, "color": "black"},
                legend=_legend_for_row(sim_row),
            ),
            row=sim_row,
            col=1,
        )
        active_xs = all_xs[active_mask]
        active_ys = all_ys[active_mask]
        for name in model_names:
            if name in model_trajectories:
                xs, ys, sim_angles = model_trajectories[name]
                fig.add_trace(
                    go.Scatter(
                        x=xs,
                        y=ys,
                        mode="lines+markers",
                        name=f"{name} sim",
                        line={
                            "color": MODEL_COLORS[name],
                            "width": 1.5,
                            "dash": "dash",
                        },
                        marker={"size": 6, "color": MODEL_COLORS[name]},
                        legend=_legend_for_row(sim_row),
                    ),
                    row=sim_row,
                    col=1,
                )
                # Dotted gray lines connecting each simulated point to
                # the corresponding actual point (visualizes the offset)
                seg_x = []
                seg_y = []
                for j, (sx, sy) in enumerate(zip(xs, ys)):
                    if not (np.isnan(sx) or np.isnan(sy)):
                        seg_x.extend([active_xs[j], sx, None])
                        seg_y.extend([active_ys[j], sy, None])
                fig.add_trace(
                    go.Scatter(
                        x=seg_x,
                        y=seg_y,
                        mode="lines",
                        line={"color": "gray", "width": 1, "dash": "dot"},
                        name=f"{name} offset",
                        showlegend=False,
                    ),
                    row=sim_row,
                    col=1,
                )
        # Yaw arrows: short line segments showing heading direction
        # Compute a reasonable arrow length from the path extent
        arrow_len = 0.01 * max(np.ptp(active_xs), np.ptp(active_ys), 0.1)

        # Actual headings (black arrows)
        actual_angles = np.array([r.state.angle for r in all_sorted_records])[
            active_mask
        ]
        arrow_x: List[Optional[float]] = []
        arrow_y: List[Optional[float]] = []
        for ax, ay, ang in zip(active_xs, active_ys, actual_angles):
            arrow_x.extend([ax, ax + arrow_len * np.cos(ang), None])
            arrow_y.extend([ay, ay + arrow_len * np.sin(ang), None])
        fig.add_trace(
            go.Scatter(
                x=arrow_x,
                y=arrow_y,
                mode="lines",
                line={"color": "black", "width": 1.5},
                name="Actual yaw",
                legend=_legend_for_row(sim_row),
            ),
            row=sim_row,
            col=1,
        )

        # Simulated headings (per-model colored arrows)
        for name in model_names:
            if name in model_trajectories:
                xs, ys, sim_angles = model_trajectories[name]
                sarrow_x: List[Optional[float]] = []
                sarrow_y: List[Optional[float]] = []
                for sx, sy, sa in zip(xs, ys, sim_angles):
                    if not np.isnan(sx):
                        sarrow_x.extend([sx, sx + arrow_len * np.cos(sa), None])
                        sarrow_y.extend([sy, sy + arrow_len * np.sin(sa), None])
                fig.add_trace(
                    go.Scatter(
                        x=sarrow_x,
                        y=sarrow_y,
                        mode="lines",
                        line={
                            "color": MODEL_COLORS.get(name, "gray"),
                            "width": 1.5,
                        },
                        name=f"{name} yaw",
                        legend=_legend_for_row(sim_row),
                    ),
                    row=sim_row,
                    col=1,
                )

        fig.update_xaxes(title_text="X (m)", row=sim_row, col=1)
        fig.update_yaxes(
            title_text="Y (m)",
            scaleanchor=f"x{sim_row}",
            scaleratio=1,
            row=sim_row,
            col=1,
        )

    # --- Gray shading for discarded time regions (time-series rows) ---
    if trim_start is not None or trim_end is not None:
        time_series_rows = list(range(1, 4))  # rows 1-3 always present
        if has_offsets:
            time_series_rows.append(5)
        if has_yaw_offsets:
            time_series_rows.append(5 + (2 if has_offsets else 0))
        for row in time_series_rows:
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

    # Shared time axis label
    for row in range(1, 4):
        fig.update_xaxes(title_text="", row=row, col=1)
    fig.update_xaxes(title_text="Time (s)", row=3, col=1)

    # Position one legend per subplot, inside the plot area (top-right)
    legend_layout = {}
    for row in range(1, n_rows + 1):
        # Get the y-domain of this subplot's yaxis
        yaxis_key = "yaxis" if row == 1 else f"yaxis{row}"
        domain = fig.layout[yaxis_key].domain
        legend_key = "legend" if row == 1 else f"legend{row}"
        legend_layout[legend_key] = {
            "x": 1.0,
            "xanchor": "right",
            "y": domain[1],
            "yanchor": "top",
            "bgcolor": "rgba(255,255,255,0.7)",
            "bordercolor": "lightgray",
            "borderwidth": 1,
            "font": {"size": 10},
        }

    fig.update_layout(
        height=300 + n_rows * 500,
        title_text="Lateral Model Comparison",
        showlegend=True,
        **legend_layout,
    )
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Replay lateral control with different vehicle models."
    )
    parser.add_argument(
        "--bag-path", type=str, required=True, help="Path to the ROS bag file"
    )
    parser.add_argument(
        "--waypoints-path",
        type=str,
        default=None,
        help="Path to waypoint file (only needed for trajectory plot)",
    )
    parser.add_argument(
        "--models",
        nargs="+",
        default=["V0", "V3"],
        choices=list(MODEL_REGISTRY.keys()),
        help="Which models to compare (default: all)",
    )
    parser.add_argument(
        "--simulate",
        choices=["open_loop", "one_step"],
        help="Forward-simulate trajectories: 'open_loop' or 'one_step'",
    )
    add_time_window_args(parser)
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output HTML path (default: <bag_dir>/<bag_name>_model_comparison.html)",
    )
    args = parser.parse_args()

    # Load data using shared BagfileLoader
    loader = BagfileLoader(args.bag_path)
    all_records = loader.bagfile_records_dicts
    if len(all_records) < 2:
        print("Error: not enough aligned records.")
        return

    # Trim records to [start, end] window for metrics (plots use all data)
    trimmed_records, _ = trim_records(all_records, args.start, args.end)
    if len(trimmed_records) < 2:
        print("Error: not enough records after trimming.")
        return

    sorted_records = sorted(trimmed_records.values(), key=lambda r: r.gps_msg_time)
    all_sorted_records = sorted(all_records.values(), key=lambda r: r.gps_msg_time)

    n_with_curv = sum(1 for r in sorted_records if r.target_curvature is not None)
    if n_with_curv == 0:
        print(
            "Error: no /target_curvature data found in bag. Cannot recompute steering commands."
        )
        return

    waypoints = resolve_waypoints(loader, args.waypoints_path)

    # Recompute steering commands per model
    model_cmds: Dict[str, np.ndarray] = {}
    for name in args.models:
        model = MODEL_REGISTRY[name]()
        model_cmds[name] = recompute_steering_commands(sorted_records, model)

    # Optionally forward-simulate trajectories, pairwise offsets
    model_trajectories: Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    model_offsets: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    model_yaw_offsets: Dict[str, np.ndarray] = {}
    if args.simulate:
        if args.simulate not in ["open_loop", "one_step"]:
            print(f"Error: invalid simulation mode '{args.simulate}'.")
            return
        sim_fn = (
            forward_simulate_one_step
            if args.simulate == "one_step"
            else forward_simulate_open_loop
        )
        for name in args.models:
            model = MODEL_REGISTRY[name]()
            xs, ys, angles = sim_fn(sorted_records, model)
            model_trajectories[name] = (xs, ys, angles)
            model_offsets[name] = compute_pairwise_offsets(sorted_records, xs, ys)
            model_yaw_offsets[name] = compute_yaw_offsets(sorted_records, angles)

    # Print summary
    print_summary(
        sorted_records=sorted_records,
        model_names=args.models,
        model_cmds=model_cmds,
        model_offsets=model_offsets,
        model_yaw_offsets=model_yaw_offsets,
    )

    # Build figure with all data, gray-out discarded sections
    fig = build_figure(
        all_sorted_records,
        waypoints,
        args.models,
        model_cmds,
        model_trajectories,
        model_offsets,
        model_yaw_offsets,
        trim_start=args.start,
        trim_end=args.end,
    )

    if args.output:
        out_path = args.output
    else:
        bag_dir = os.path.dirname(os.path.abspath(args.bag_path))
        bag_name = os.path.splitext(os.path.basename(args.bag_path))[0]
        out_path = os.path.join(bag_dir, f"{bag_name}_model_comparison.html")

    fig.write_html(out_path)
    print(f"Interactive report saved to: {out_path}")


if __name__ == "__main__":
    main()
