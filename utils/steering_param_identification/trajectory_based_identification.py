#!/usr/bin/env python3
"""
Trajectory-based parameter identification for the kinematic bicycle model.

Instead of fitting parameters from steady-state circle data only, this script
optimizes the speed-to-coefficient mapping by minimizing one-step prediction
error directly on bag file trajectories.

The model being optimized follows the CarModelBicycleSpeedToParam structure:
    radius = STEERING_DIRECTION_FACTOR * param(speed) * speed / (cmd_steering - STEERING_IDLE_PWM)
where param(speed) is linearly interpolated from a set of (speed, param) pairs.

The optimizer finds the coefficient values that minimize:
    cost = mean(position_error^2) + yaw_weight * mean(yaw_error^2)
computed via one-step forward simulation against recorded GPS positions.

Usage:
    python trajectory_based_identification.py \
        --bag-paths bag1.bag bag2.bag \
        --speeds 2 3 4 5 6 7 \
        --base-model V3

    # Use a held-out bag for validation:
    python trajectory_based_identification.py \
        --bag-paths train1.bag train2.bag \
        --validation-bag-path val.bag \
        --speeds 2 3 4 5 6 7 \
        --base-model V3 \
"""

import argparse
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import differential_evolution


# Add performance_analysis to path so we can import its modules, HACK...
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PERF_ANALYSIS_DIR = os.path.join(SCRIPT_DIR, "..", "performance_analysis")
sys.path.insert(0, PERF_ANALYSIS_DIR)

from analysis_utils import trim_records  # noqa: E402
from bagfile_loader import BagfileLoader, BagfileRecord  # noqa: E402
from geometry_utils_pkg.geometry_utils import wrap_angle  # noqa: E402
from vehicle_models_pkg.vehicle_models import (  # noqa: E402
    CarModelBicyclePure,
    CarModelBicycleSpeedToParam,
    CarModelBicycleV2,
    CarModelBicycleV3,
    CarModelBicycleV4,
)
from vehicle_models_pkg.vehicle_models_constants import STEERING_IDLE_PWM  # noqa: E402


BASE_MODEL_REGISTRY: Dict[str, type] = {
    "V2": CarModelBicycleV2,
    "V3": CarModelBicycleV3,
    "V4": CarModelBicycleV4,
}

DEFAULT_PARAM_VALUE = (
    10.0  # Default param value for speeds not in the base model mapping (for bounds)
)


# ---------------------------------------------------------------------------
# Parameterized model: SpeedToParam-based with injectable coefficients
# ---------------------------------------------------------------------------
class CarModelParameterized(CarModelBicycleSpeedToParam):
    """A SpeedToParam model whose mapping can be set externally."""

    def __init__(self, speeds_to_param: Dict[float, float]) -> None:
        super().__init__()
        self.SPEEDS_TO_PARAM_MAPPING = dict(speeds_to_param)


# ---------------------------------------------------------------------------
# One-step cost computation (reuses the same logic as replay_lateral_models.py)
# ---------------------------------------------------------------------------
def compute_one_step_cost(
    sorted_records: List[BagfileRecord],
    model: CarModelBicyclePure,
    yaw_weight: float = 1.0,
    min_speed: float = 0.5,
) -> Tuple[float, Dict[str, float]]:
    """
    Run one-step forward simulation and return the cost (mean squared position
    error + yaw_weight * mean squared yaw error).

    Also returns a dict of diagnostic metrics.

    Records with speed below min_speed are skipped (near-stationary data is
    noisy and uninformative for steering identification).
    """
    position_sq_sum = 0.0
    yaw_sq_sum = 0.0
    n = 0

    for i in range(len(sorted_records) - 1):
        r = sorted_records[i]
        r_next = sorted_records[i + 1]

        # Skip low-speed records
        if r.state.vx < min_speed:
            continue
        # Skip records where steering is at idle (going straight, nothing to fit)
        if r.steering_cmd == STEERING_IDLE_PWM:
            continue

        dt = r_next.gps_msg_time - r.gps_msg_time
        if dt <= 0:
            continue

        # One-step: init at current GPS, step once, compare with next GPS
        model.states.clear()
        model.init(x=r.state.x, y=r.state.y, vx=r.state.vx, angle=r.state.angle)
        model.step(dt=dt, cmd_steering=r.steering_cmd)

        pred = model.states[-1]

        # Squared Euclidean position error
        dx = pred.x - r_next.state.x
        dy = pred.y - r_next.state.y

        position_sq_sum += dx**2 + dy**2

        # Yaw offset
        yaw_err = wrap_angle(pred.angle - r_next.state.angle)
        yaw_sq_sum += yaw_err**2

        n += 1

    if n == 0:
        return float("inf"), {
            "n": 0,
            "rms_offset_m": float("nan"),
            "rms_yaw_deg": float("nan"),
        }

    mean_position_sq = position_sq_sum / n
    mean_yaw_sq = yaw_sq_sum / n

    cost = mean_position_sq + yaw_weight * mean_yaw_sq

    metrics = {
        "n": n,
        "rms_offset_m": np.sqrt(mean_position_sq),
        "rms_yaw_deg": np.degrees(np.sqrt(mean_yaw_sq)),
    }
    return cost, metrics


# ---------------------------------------------------------------------------
# Load and prepare records from a bag file
# ---------------------------------------------------------------------------
def load_sorted_records(
    bag_path: str,
    start: Optional[float] = None,
    end: Optional[float] = None,
) -> List[BagfileRecord]:
    """Load a bag file and return sorted, optionally trimmed records."""
    loader = BagfileLoader(bag_path)
    records = loader.bagfile_records_dicts
    if len(records) < 2:
        raise ValueError(f"Not enough records in {bag_path}")

    trimmed, _ = trim_records(records, start, end)
    sorted_records = sorted(trimmed.values(), key=lambda r: r.gps_msg_time)
    return sorted_records


# ---------------------------------------------------------------------------
# Objective function for the optimizer
# ---------------------------------------------------------------------------
def make_objective(
    all_sorted_records: List[List[BagfileRecord]],
    speeds: List[float],
    yaw_weight: float,
    min_speed: float,
):
    """
    Return a callable objective(coeffs) -> cost for use with scipy optimizers.

    coeffs is a 1-D array of length len(speeds), one coefficient per speed point.
    """

    def objective(coeffs: np.ndarray) -> float:
        mapping = dict(zip(speeds, coeffs))
        total_cost = 0.0
        total_n = 0
        for sorted_records in all_sorted_records:
            model = CarModelParameterized(mapping)
            cost, metrics = compute_one_step_cost(
                sorted_records, model, yaw_weight=yaw_weight, min_speed=min_speed
            )
            n = metrics["n"]
            if n > 0:
                total_cost += cost * n
                total_n += n

        if total_n == 0:
            return float("inf")
        return total_cost / total_n

    return objective


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Trajectory-based parameter identification for the kinematic bicycle model."
    )
    parser.add_argument(
        "--bag-paths",
        nargs="+",
        required=True,
        help="Path(s) to training bag file(s) or folder(s) containing .bag files",
    )
    parser.add_argument(
        "--validation-bag-path",
        type=str,
        default=None,
        help="Optional held-out bag file for validation",
    )
    parser.add_argument(
        "--base-model",
        type=str,
        required=True,
        choices=list(BASE_MODEL_REGISTRY.keys()),
        help="Base model to use for baseline comparison and initial bounds",
    )
    parser.add_argument(
        "--speeds",
        nargs="+",
        type=float,
        default=None,
        help="Speed points for the coefficient mapping (default: taken from base model)",
    )
    parser.add_argument(
        "--yaw-weight",
        type=float,
        default=1.0,
        help="Weight for yaw error in cost function (default: 1.0)",
    )
    parser.add_argument(
        "--min-speed",
        type=float,
        default=0.5,
        help="Minimum speed to include in optimization (default: 0.5 m/s)",
    )
    parser.add_argument(
        "--start",
        type=float,
        default=None,
        help="Start time in seconds from bag start (trim earlier data)",
    )
    parser.add_argument(
        "--end",
        type=float,
        default=None,
        help="End time in seconds from bag start (trim later data)",
    )
    parser.add_argument(
        "--maxiter",
        type=int,
        default=100,
        help="Maximum iterations for differential_evolution (default: 100)",
    )
    args = parser.parse_args()

    # Resolve base model
    base_model_class = BASE_MODEL_REGISTRY[args.base_model]
    base_model_name = args.base_model

    # Default speeds from the base model's mapping if not provided
    if args.speeds is None:
        args.speeds = sorted(base_model_class.SPEEDS_TO_PARAM_MAPPING.keys())
    # Validate that the base model has params for the requested speeds (for bounds)
    for s in args.speeds:
        if s not in base_model_class.SPEEDS_TO_PARAM_MAPPING:
            print(
                f"WARNING: speed {s} not in {base_model_name}.SPEEDS_TO_PARAM_MAPPING "
                f"(available: {sorted(base_model_class.SPEEDS_TO_PARAM_MAPPING.keys())}). "
                f"Using default bound center of {DEFAULT_PARAM_VALUE}."
            )

    # Resolve bag paths: expand directories to their .bag files
    resolved_bag_paths = []
    for path in args.bag_paths:
        if os.path.isdir(path):
            bag_files = sorted(
                os.path.join(path, f) for f in os.listdir(path) if f.endswith(".bag")
            )
            if not bag_files:
                print(f"Warning: no .bag files found in {path}")
            resolved_bag_paths.extend(bag_files)
        else:
            resolved_bag_paths.append(path)
    args.bag_paths = resolved_bag_paths

    if not args.bag_paths:
        print("Error: no bag files to process.")
        return

    if len(args.bag_paths) > 1 and (args.start is not None or args.end is not None):
        print(
            "WARNING: --start/--end will be applied identically to all bag files. "
            "This is probably not what you want when using multiple bags."
        )

    # Load all training bags
    print(f"Loading {len(args.bag_paths)} training bag file(s)...")
    all_sorted_records = []
    for bag_path in args.bag_paths:
        print(f"  Loading {os.path.basename(bag_path)}...")
        records = load_sorted_records(bag_path, args.start, args.end)
        print(f"    {len(records)} records")
        all_sorted_records.append(records)

    # Print datapoint counts per speed region
    print(
        f"\n  Datapoints per speed region (min_speed={args.min_speed} m/s, excluding idle steering):"
    )
    speeds_sorted = sorted(args.speeds)
    # Regions: [min_speed, speeds[0]], [speeds[0], speeds[1]], ..., [speeds[-1], +inf)
    region_edges = [args.min_speed] + speeds_sorted + [float("inf")]
    all_records_flat = [r for records in all_sorted_records for r in records]
    usable = [
        r
        for r in all_records_flat
        if r.state.vx >= args.min_speed and r.steering_cmd != STEERING_IDLE_PWM
    ]
    for i in range(len(region_edges) - 1):
        lo, hi = region_edges[i], region_edges[i + 1]
        count = sum(1 for r in usable if lo <= r.state.vx < hi)
        hi_str = f"{hi:.1f}" if hi != float("inf") else "+"
        print(f"    [{lo:.1f}, {hi_str}) m/s: {count} datapoints")
    print(f"    Total usable: {len(usable)}")

    # Load validation bag if provided
    val_records = None
    if args.validation_bag_path:
        print(f"  Loading validation: {os.path.basename(args.validation_bag_path)}...")
        val_records = load_sorted_records(
            args.validation_bag_path, args.start, args.end
        )
        print(f"    {len(val_records)} records")

    # Evaluate baseline before optimization
    print(f"\n===== {base_model_name} BASELINE =====")
    baseline_model = base_model_class()
    for i, (bag_path, sorted_records) in enumerate(
        zip(args.bag_paths, all_sorted_records)
    ):
        _, metrics = compute_one_step_cost(
            sorted_records,
            baseline_model,
            yaw_weight=args.yaw_weight,
            min_speed=args.min_speed,
        )
        print(
            f"  {os.path.basename(bag_path)}: "
            f"RMS offset={metrics['rms_offset_m']:.4f} m, "
            f"RMS yaw={metrics['rms_yaw_deg']:.2f} deg  "
            f"(n={metrics['n']})"
        )
    if val_records:
        _, metrics = compute_one_step_cost(
            val_records,
            baseline_model,
            yaw_weight=args.yaw_weight,
            min_speed=args.min_speed,
        )
        print(
            f"  [VAL] {os.path.basename(args.validation_bag_path)}: "
            f"RMS offset={metrics['rms_offset_m']:.4f} m, "
            f"RMS yaw={metrics['rms_yaw_deg']:.2f} deg  "
            f"(n={metrics['n']})"
        )

    # Run optimization
    print("\n===== OPTIMIZATION =====")
    print(f"  Speed points: {args.speeds}")
    print(f"  Yaw weight: {args.yaw_weight}")
    print(f"  Min speed: {args.min_speed} m/s")
    print(f"  Training bags: {len(all_sorted_records)}")

    # Use base model's params as initial reference for bounds
    base_params = [
        base_model_class.SPEEDS_TO_PARAM_MAPPING[s]
        if s in base_model_class.SPEEDS_TO_PARAM_MAPPING
        else DEFAULT_PARAM_VALUE
        for s in args.speeds
    ]
    # Bounds: allow params to vary between 0.2x and 5x the base model value
    bounds = [(p * 0.2, p * 5.0) for p in base_params]

    objective = make_objective(
        all_sorted_records, args.speeds, args.yaw_weight, args.min_speed
    )

    print("  Running differential_evolution...")
    result = differential_evolution(
        objective,
        bounds=bounds,
        maxiter=args.maxiter,
        seed=42,
        tol=1e-8,
        disp=True,
        polish=True,
    )

    optimized_params = result.x
    optimized_mapping = dict(zip(args.speeds, optimized_params))

    print("\n===== RESULTS =====")
    print(f"  Optimizer success: {result.success}")
    print(f"  Optimizer message: {result.message}")
    print(f"  Final cost: {result.fun:.8f}")
    print()
    print("  Optimized SPEEDS_TO_PARAM_MAPPING:")
    print("  {")
    for s, p in sorted(optimized_mapping.items()):
        base_p = base_model_class.SPEEDS_TO_PARAM_MAPPING.get(s, None)
        delta_str = ""
        if base_p is not None:
            delta_pct = (p - base_p) / base_p * 100
            delta_str = f"  # {base_model_name}: {base_p:.2f}  ({delta_pct:+.1f}%)"
        print(f"      {s}: {p:.2f},{delta_str}")
    print("  }")

    # Evaluate optimized model on training data
    print("\n===== OPTIMIZED MODEL EVALUATION =====")
    opt_model = CarModelParameterized(optimized_mapping)
    for i, (bag_path, sorted_records) in enumerate(
        zip(args.bag_paths, all_sorted_records)
    ):
        _, metrics = compute_one_step_cost(
            sorted_records,
            opt_model,
            yaw_weight=args.yaw_weight,
            min_speed=args.min_speed,
        )
        # Also get baseline for comparison
        _, base_metrics = compute_one_step_cost(
            sorted_records,
            base_model_class(),
            yaw_weight=args.yaw_weight,
            min_speed=args.min_speed,
        )
        print(f"  {os.path.basename(bag_path)}:")
        print(
            f"    {base_model_name}:        RMS offset={base_metrics['rms_offset_m']:.4f} m, "
            f"RMS yaw={base_metrics['rms_yaw_deg']:.2f} deg"
        )
        print(
            f"    Optimized: RMS offset={metrics['rms_offset_m']:.4f} m, "
            f"RMS yaw={metrics['rms_yaw_deg']:.2f} deg"
        )
        offset_improv = (
            (1 - metrics["rms_offset_m"] / base_metrics["rms_offset_m"]) * 100
            if base_metrics["rms_offset_m"] > 0
            else 0
        )
        yaw_improv = (
            (1 - metrics["rms_yaw_deg"] / base_metrics["rms_yaw_deg"]) * 100
            if base_metrics["rms_yaw_deg"] > 0
            else 0
        )
        print(f"    Improvement: offset {offset_improv:+.1f}%, yaw {yaw_improv:+.1f}%")

    # Evaluate on validation bag
    if val_records:
        _, metrics = compute_one_step_cost(
            val_records, opt_model, yaw_weight=args.yaw_weight, min_speed=args.min_speed
        )
        _, base_metrics = compute_one_step_cost(
            val_records,
            base_model_class(),
            yaw_weight=args.yaw_weight,
            min_speed=args.min_speed,
        )
        print(f"\n  [VALIDATION] {os.path.basename(args.validation_bag_path)}:")
        print(
            f"    {base_model_name}:        RMS offset={base_metrics['rms_offset_m']:.4f} m, "
            f"RMS yaw={base_metrics['rms_yaw_deg']:.2f} deg"
        )
        print(
            f"    Optimized: RMS offset={metrics['rms_offset_m']:.4f} m, "
            f"RMS yaw={metrics['rms_yaw_deg']:.2f} deg"
        )
        offset_improv = (
            (1 - metrics["rms_offset_m"] / base_metrics["rms_offset_m"]) * 100
            if base_metrics["rms_offset_m"] > 0
            else 0
        )
        yaw_improv = (
            (1 - metrics["rms_yaw_deg"] / base_metrics["rms_yaw_deg"]) * 100
            if base_metrics["rms_yaw_deg"] > 0
            else 0
        )
        print(f"    Improvement: offset {offset_improv:+.1f}%, yaw {yaw_improv:+.1f}%")

    print("\n===== DONE =====")


if __name__ == "__main__":
    main()
