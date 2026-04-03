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
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import plotly.graph_objs as go
from plotly.subplots import make_subplots

from analyze_bagfile import Record, load_bag, load_waypoints

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
    records: List[Record], model: CarModelBicyclePure
) -> np.ndarray:
    """
    For each record, recompute the steering PWM command that `model` would
    produce given the recorded target_curvature and speed.

    Mirrors the logic in lateral_controller.py:loop().
    """
    cmds = np.full(len(records), float("nan"))
    for i, r in enumerate(records):
        if r.target_curvature is None:
            continue
        if abs(r.target_curvature) < 1e-3:
            cmd = STEERING_IDLE_PWM
        else:
            cmd = model.compute_steering_command_from_radius(
                radius=1.0 / r.target_curvature,
                speed=r.speed,
            )
        # Clamp to PWM bounds (same as the real controller)
        cmd = max(STEERING_MIN_PWM, min(STEERING_MAX_PWM, cmd))
        cmds[i] = cmd
    return cmds


# ---------------------------------------------------------------------------
# Forward simulation
# ---------------------------------------------------------------------------
def forward_simulate(
    records: List[Record], model: CarModelBicyclePure
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Starting from the first record's pose, integrate the bicycle model forward
    using the *actual recorded* steering commands and speeds.

    This shows: "given what the car actually commanded, where does this model
    think the car would end up?"

    Returns (xs, ys) arrays of simulated positions.
    """
    model.states.clear()
    r0 = records[0]
    model.init(x=r0.x, y=r0.y, vx=r0.speed, angle=r0.yaw)

    for i in range(1, len(records)):
        dt = records[i].t - records[i - 1].t
        if dt <= 0:
            # Duplicate timestamp — just copy the last state
            model.states.append(model.states[-1])
            continue

        # Update the model's speed to match the actual recorded speed
        model.states[-1].vx = records[i - 1].speed

        model.step(dt=dt, cmd_steering=records[i - 1].steering_cmd)

    xs = np.array([s.x for s in model.states])
    ys = np.array([s.y for s in model.states])
    return xs, ys


# ---------------------------------------------------------------------------
# Summary statistics
# ---------------------------------------------------------------------------
def print_summary(
    records: List[Record],
    model_names: List[str],
    model_cmds: Dict[str, np.ndarray],
) -> None:
    actual = np.array([r.steering_cmd for r in records])

    print("\n===== LATERAL MODEL COMPARISON SUMMARY =====")
    print(f"  Records:  {len(records)}")
    print(f"  Duration: {records[-1].t - records[0].t:.1f} s")

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
        # Correlation
        corr = np.corrcoef(actual[valid], cmds[valid])[0, 1]
        print(f"    Correlation with actual:    {corr:.4f}")

    print("=============================================\n")


# ---------------------------------------------------------------------------
# Plotly figure
# ---------------------------------------------------------------------------
def build_figure(
    records: List[Record],
    waypoints: Optional[np.ndarray],
    model_names: List[str],
    model_cmds: Dict[str, np.ndarray],
    model_trajectories: Dict[str, Tuple[np.ndarray, np.ndarray]],
) -> go.Figure:
    has_trajectories = len(model_trajectories) > 0
    n_rows = 4 if has_trajectories else 3
    row_heights = [0.35, 0.25, 0.20, 0.20] if has_trajectories else [0.40, 0.30, 0.30]

    subtitles = [
        "Steering Command: Actual vs Models (PWM)",
        "Steering Command Residual: Model - Actual (PWM)",
        "Target Curvature & Speed Context",
    ]
    if has_trajectories:
        subtitles.append("Forward-Simulated Trajectories")

    fig = make_subplots(
        rows=n_rows,
        cols=1,
        subplot_titles=subtitles,
        row_heights=row_heights,
        vertical_spacing=0.05,
    )

    times = np.array([r.t for r in records])
    t_rel = times - times[0]
    actual_cmds = np.array([r.steering_cmd for r in records])

    # --- Row 1: Steering commands ---
    fig.add_trace(
        go.Scatter(
            x=t_rel,
            y=actual_cmds,
            mode="lines",
            name="Actual",
            line={"color": "black", "width": 2},
        ),
        row=1,
        col=1,
    )
    for name in model_names:
        fig.add_trace(
            go.Scatter(
                x=t_rel,
                y=model_cmds[name],
                mode="lines",
                name=name,
                line={"color": MODEL_COLORS.get(name, "gray"), "width": 1.5},
            ),
            row=1,
            col=1,
        )
    # Saturation lines
    fig.add_hline(
        y=STEERING_MIN_PWM, line_dash="dot", line_color="gray", row=1, col=1,
        annotation_text="min", annotation_position="bottom left",
    )
    fig.add_hline(
        y=STEERING_MAX_PWM, line_dash="dot", line_color="gray", row=1, col=1,
        annotation_text="max", annotation_position="top left",
    )
    fig.update_yaxes(title_text="Steering cmd (PWM)", row=1, col=1)

    # --- Row 2: Residuals ---
    for name in model_names:
        residual = model_cmds[name] - actual_cmds
        fig.add_trace(
            go.Scatter(
                x=t_rel,
                y=residual,
                mode="lines",
                name=f"{name} residual",
                line={"color": MODEL_COLORS.get(name, "gray"), "width": 1},
                showlegend=False,
            ),
            row=2,
            col=1,
        )
    fig.add_hline(y=0, line_color="black", line_width=0.5, row=2, col=1)
    fig.update_yaxes(title_text="Model - Actual (PWM)", row=2, col=1)

    # --- Row 3: Context (curvature and speed) ---
    curvatures = [
        r.target_curvature if r.target_curvature is not None else float("nan")
        for r in records
    ]
    speeds = [r.speed for r in records]
    fig.add_trace(
        go.Scatter(
            x=t_rel,
            y=curvatures,
            mode="lines",
            name="Target curvature",
            line={"color": "tomato", "width": 1},
        ),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=t_rel,
            y=speeds,
            mode="lines",
            name="Speed",
            line={"color": "steelblue", "width": 1},
            yaxis="y2",
        ),
        row=3,
        col=1,
    )
    fig.update_yaxes(title_text="Curvature (1/m) / Speed (m/s)", row=3, col=1)

    # --- Row 4: Trajectories (if simulated) ---
    if has_trajectories:
        # Waypoints background
        if waypoints is not None:
            fig.add_trace(
                go.Scatter(
                    x=waypoints[:, 0],
                    y=waypoints[:, 1],
                    mode="lines",
                    line={"color": "lightgray", "width": 4},
                    name="Waypoints",
                    showlegend=True,
                ),
                row=4,
                col=1,
            )
        # Actual path
        actual_x = [r.x for r in records]
        actual_y = [r.y for r in records]
        fig.add_trace(
            go.Scatter(
                x=actual_x,
                y=actual_y,
                mode="lines",
                name="Actual path",
                line={"color": "black", "width": 2},
            ),
            row=4,
            col=1,
        )
        # Simulated paths
        for name in model_names:
            if name in model_trajectories:
                xs, ys = model_trajectories[name]
                fig.add_trace(
                    go.Scatter(
                        x=xs,
                        y=ys,
                        mode="lines",
                        name=f"{name} sim",
                        line={
                            "color": MODEL_COLORS.get(name, "gray"),
                            "width": 1.5,
                            "dash": "dash",
                        },
                    ),
                    row=4,
                    col=1,
                )
        fig.update_xaxes(title_text="X (m)", row=4, col=1)
        fig.update_yaxes(
            title_text="Y (m)", scaleanchor="x", scaleratio=1, row=4, col=1
        )

    # Shared time axis label
    time_row = 3 if not has_trajectories else 3
    for row in range(1, n_rows + 1):
        fig.update_xaxes(title_text="", row=row, col=1)
    fig.update_xaxes(title_text="Time (s)", row=time_row, col=1)
    if has_trajectories:
        fig.update_xaxes(title_text="X (m)", row=4, col=1)

    fig.update_layout(
        height=300 + n_rows * 350,
        title_text="Lateral Model Comparison",
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
        default=["V0", "V1", "V2", "V3"],
        choices=list(MODEL_REGISTRY.keys()),
        help="Which models to compare (default: all)",
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Forward-simulate trajectories using each model (adds trajectory plot)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output HTML path (default: <bag_dir>/<bag_name>_model_comparison.html)",
    )
    args = parser.parse_args()

    # Load data
    records = load_bag(args.bag_path)
    if len(records) < 2:
        print("Error: not enough aligned records.")
        return

    n_with_curv = sum(1 for r in records if r.target_curvature is not None)
    if n_with_curv == 0:
        print("Error: no /target_curvature data found in bag. Cannot recompute steering commands.")
        return

    waypoints = None
    if args.waypoints_path:
        waypoints = load_waypoints(args.waypoints_path)

    # Recompute steering commands per model
    model_cmds: Dict[str, np.ndarray] = {}
    for name in args.models:
        model = MODEL_REGISTRY[name]()
        model_cmds[name] = recompute_steering_commands(records, model)

    # Optionally forward-simulate trajectories
    model_trajectories: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    if args.simulate:
        for name in args.models:
            model = MODEL_REGISTRY[name]()
            xs, ys = forward_simulate(records, model)
            model_trajectories[name] = (xs, ys)

    # Print summary
    print_summary(records, args.models, model_cmds)

    # Build and save figure
    fig = build_figure(records, waypoints, args.models, model_cmds, model_trajectories)

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
