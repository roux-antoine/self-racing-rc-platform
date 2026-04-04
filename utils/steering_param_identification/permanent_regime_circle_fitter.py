#!/usr/bin/env python3

import argparse
import glob
import os
import sys
from typing import List

import numpy as np
import pandas as pd
import plotly.graph_objects as go


sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "performance_analysis")
)
from bagfile_loader import BagfileLoader
from geometry_utils_pkg.geometry_utils import fit_circle_to_points
from plotly.subplots import make_subplots
from vehicle_models_pkg.vehicle_models_constants import (
    MAX_STEERING_FBK,
    MIN_STEERING_FBK,
    STEERING_MAX_PWM,
    STEERING_MIN_PWM,
)


def find_bagfiles(folder_path: str) -> List[str]:
    """Find all .bag files in the given folder.

    Args:
        folder_path (str): Path to the folder containing bagfiles

    Returns:
        List[str]: List of paths to bagfiles
    """
    if not os.path.exists(folder_path):
        raise FileNotFoundError(f"Folder not found: {folder_path}")

    if not os.path.isdir(folder_path):
        raise NotADirectoryError(f"Path is not a directory: {folder_path}")

    # Find all .bag files in the folder
    bagfile_pattern = os.path.join(folder_path, "*.bag")
    bagfiles = glob.glob(bagfile_pattern)

    if not bagfiles:
        print(f"Warning: No .bag files found in {folder_path}")

    return sorted(bagfiles)


def fit_circles(
    bagfile_records,
    sliding_window_size: int,
    radius_threshold: float = 30,
    debug: bool = False,
):
    """Process bagfile records using a sliding window approach and fit circles to each window.

    Args:
        bagfile_records: Dictionary of bagfile records keyed by timestamp
        window_size (int): Number of points in each sliding window
        radius_threshold (float): Maximum radius to consider a valid fit (in meters)
        debug (bool): Whether to enable debug plots

    Returns:
        List of fitted circle parameters for each window
    """
    STEP = 3
    SPEED_RANGE_THRESHOLD = 0.8  # m/s, 'arbitrarily' chosen
    STEERING_CMD_RANGE_THRESHOLD = 3.0  # PWM units, 'arbitrarily' chosen
    MIN_POSSIBLE_RADIUS = 1.25  # meters, measured on the real car
    MIN_SPEED = 1  # m/s, below which we ignore the point

    # Convert records to a sorted list of (timestamp, record) tuples
    sorted_records = sorted(bagfile_records.items())

    if len(sorted_records) < sliding_window_size:
        print(
            f"Warning: Not enough records ({len(sorted_records)}) for window size {sliding_window_size}"
        )
        return []

    circle_fits = []

    print(
        f"Processing {len(sorted_records)} records with sliding window of size {sliding_window_size}"
    )

    # Iterate through all possible windows
    for i in range(0, len(sorted_records) - sliding_window_size + 1, STEP):
        window_records = sorted_records[i : i + sliding_window_size]

        # Extract (x, y) points from the window
        points = []
        for _, record in window_records:
            points.append((record.state.x, record.state.y))

        try:
            # Fit circle to the points in this window
            center_x, center_y, radius = fit_circle_to_points(points)

            if MIN_POSSIBLE_RADIUS < radius < radius_threshold:

                # Compute ranges of interesting metrics
                speeds = [record.state.vx for _, record in window_records]

                if min(speeds) < MIN_SPEED:
                    continue

                steering_commands = [
                    record.steering_cmd for _, record in window_records
                ]
                steering_feedbacks = [
                    record.steering_fbk for _, record in window_records
                ]
                speed_range = max(speeds) - min(speeds)
                steering_cmd_range = max(steering_commands) - min(steering_commands)
                steering_fbk_range = max(steering_feedbacks) - min(steering_feedbacks)
                relative_variation_steering_cmd = steering_cmd_range / (
                    STEERING_MAX_PWM - STEERING_MIN_PWM
                )
                relative_variation_steering_fbk = steering_fbk_range / (
                    MAX_STEERING_FBK - MIN_STEERING_FBK
                )
                if debug:
                    # Create subplot figure with 4 rows, 1 column
                    fig = make_subplots(
                        rows=4,
                        cols=1,
                        subplot_titles=(
                            "Trajectory and Fitted Circle",
                            f"Steering Commands  (full range: {STEERING_MIN_PWM}-{STEERING_MAX_PWM})",
                            f"Steering Feedback (full range: {MIN_STEERING_FBK}-{MAX_STEERING_FBK})",
                            "Speed (m/s)",
                        ),
                        specs=[
                            [{"secondary_y": False}],
                            [{"secondary_y": False}],
                            [{"secondary_y": False}],
                            [{"secondary_y": False}],
                        ],
                        row_heights=[
                            0.6,
                            0.13,
                            0.13,
                            0.13,
                        ],
                        vertical_spacing=0.05,
                    )

                    # Top subplot: trajectory and circle
                    x_vals, y_vals = zip(*points)
                    fig.add_trace(
                        go.Scatter(x=x_vals, y=y_vals, mode="markers", name="Points"),
                        row=1,
                        col=1,
                    )
                    outside_points = [
                        pt
                        for j, (_, rec) in enumerate(sorted_records)
                        if j < i or j >= i + sliding_window_size
                        for pt in [(rec.state.x, rec.state.y)]
                    ]
                    if outside_points:
                        out_x, out_y = zip(*outside_points)
                        fig.add_trace(
                            go.Scatter(
                                x=out_x,
                                y=out_y,
                                mode="markers",
                                name="Other Points",
                                marker={"color": "gray", "size": 5, "opacity": 0.5},
                            ),
                            row=1,
                            col=1,
                        )
                    # Plot fitted circle
                    theta = np.linspace(0, 2 * np.pi, 100)
                    circle_x = center_x + radius * np.cos(theta)
                    circle_y = center_y + radius * np.sin(theta)
                    fig.add_trace(
                        go.Scatter(
                            x=circle_x, y=circle_y, mode="lines", name="Fitted Circle"
                        ),
                        row=1,
                        col=1,
                    )

                    # Second subplot: steering commands
                    timestamps = [timestamp for timestamp, _ in window_records]
                    relative_times = [t - timestamps[0] for t in timestamps]
                    fig.add_trace(
                        go.Scatter(
                            x=relative_times,
                            y=steering_commands,
                            mode="lines+markers",
                            name="Steering Commands",
                            line={"color": "red"},
                        ),
                        row=2,
                        col=1,
                    )
                    fig.update_yaxes(
                        range=[STEERING_MIN_PWM - 1, STEERING_MAX_PWM + 1], row=2, col=1
                    )
                    fig.add_shape(
                        type="line",
                        x0=relative_times[0],
                        y0=STEERING_MIN_PWM,
                        x1=relative_times[-1],
                        y1=STEERING_MIN_PWM,
                        line={"color": "black", "width": 2},
                        row=2,
                        col=1,
                    )
                    fig.add_shape(
                        type="line",
                        x0=relative_times[0],
                        y0=STEERING_MAX_PWM,
                        x1=relative_times[-1],
                        y1=STEERING_MAX_PWM,
                        line={"color": "black", "width": 2},
                        row=2,
                        col=1,
                    )
                    fig.add_annotation(
                        x=relative_times[-1],
                        y=STEERING_MAX_PWM - 5,
                        showarrow=False,
                        text=f"Relative Variation: {relative_variation_steering_cmd:.0%}",  # noqa: E231
                        font={"size": 12, "color": "black"},
                        row=2,
                        col=1,
                    )

                    # Third subplot: steering feedback
                    fig.add_trace(
                        go.Scatter(
                            x=relative_times,
                            y=steering_feedbacks,
                            mode="lines+markers",
                            name="Steering Feedback",
                            line={"color": "blue"},
                        ),
                        row=3,
                        col=1,
                    )
                    fig.update_yaxes(
                        range=[MIN_STEERING_FBK - 10, MAX_STEERING_FBK + 10],
                        row=3,
                        col=1,
                    )
                    fig.add_shape(
                        type="line",
                        x0=relative_times[0],
                        y0=MIN_STEERING_FBK,
                        x1=relative_times[-1],
                        y1=MIN_STEERING_FBK,
                        line={"color": "black", "width": 2},
                        row=3,
                        col=1,
                    )
                    fig.add_shape(
                        type="line",
                        x0=relative_times[0],
                        y0=MAX_STEERING_FBK,
                        x1=relative_times[-1],
                        y1=MAX_STEERING_FBK,
                        line={"color": "black", "width": 2},
                        row=3,
                        col=1,
                    )
                    fig.add_annotation(
                        x=relative_times[-1],
                        y=MAX_STEERING_FBK - 20,
                        text=f"Relative Variation: {relative_variation_steering_fbk:.0%}",  # noqa: E231
                        showarrow=False,
                        font={"size": 12, "color": "black"},
                        row=3,
                        col=1,
                    )

                    # Fourth subplot: speed
                    fig.add_trace(
                        go.Scatter(
                            x=relative_times,
                            y=speeds,
                            mode="lines+markers",
                            name="Speed",
                            line={"color": "green"},
                        ),
                        row=4,
                        col=1,
                    )

                    fig.update_layout(
                        title=f"Circle Fit Debug - Window {i} to {i + sliding_window_size - 1}, Radius: {radius:.2f}m<br>"  # noqa: E231
                        f"Ranges: Speed: {speed_range:.2f} m/s, Steering Cmd: {steering_cmd_range:.1f}, Steering Fbk: {steering_fbk_range:.1f}",  # noqa: E231
                        showlegend=True,
                    )

                    # Update axes labels
                    fig.update_xaxes(title_text="X (m)", row=1, col=1)
                    fig.update_yaxes(title_text="Y (m)", row=1, col=1)
                    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
                    fig.update_yaxes(title_text="Steering Command", row=2, col=1)
                    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
                    fig.update_yaxes(title_text="Steering Feedback", row=3, col=1)
                    fig.update_xaxes(title_text="Time (s)", row=4, col=1)
                    fig.update_yaxes(title_text="Speed (m/s)", row=4, col=1)

                    # Make trajectory plot aspect ratio equal
                    fig.update_yaxes(scaleanchor="x", scaleratio=1, row=1, col=1)

                    fig.show()
                    input("Check the plot and press Enter to continue...")

                if speed_range > SPEED_RANGE_THRESHOLD:
                    continue

                if steering_cmd_range > STEERING_CMD_RANGE_THRESHOLD:
                    continue

                circle_fits.append(
                    {
                        "center_x": center_x,
                        "center_y": center_y,
                        "radius": radius,
                        "window_start_idx": i,
                        "window_end_idx": i + sliding_window_size - 1,
                        "points": points,
                        "speed_range": speed_range,
                        "steering_cmd_range": steering_cmd_range,
                        "steering_fbk_range": steering_fbk_range,
                        "mean_speed": np.mean(speeds),
                        "mean_steering_cmd": np.mean(steering_commands),
                        "mean_steering_fbk": np.mean(steering_feedbacks),
                        "relative_variation_steering_cmd": relative_variation_steering_cmd,
                        "relative_variation_steering_fbk": relative_variation_steering_fbk,
                    }
                )

        except (ValueError, np.linalg.LinAlgError) as e:
            print(
                f"Warning: Failed to fit circle for window starting at index {i}: {e}"
            )
            continue

    return circle_fits


def main():
    parser = argparse.ArgumentParser(
        description="Load and process all bagfiles in a folder for permanent regime steering parameter identification."
    )
    parser.add_argument(
        "--bagfiles-folder",
        type=str,
        required=True,
        help="Path to the folder containing ROS bag files",
    )
    parser.add_argument("--debug", action="store_true", help="Enable debug plots")
    parser.add_argument(
        "--window-size",
        type=int,
        default=10,
        help="Number of points in sliding window for circle fitting",
    )

    args = parser.parse_args()

    try:
        # Find all bagfiles in the specified folder
        bagfiles = find_bagfiles(args.bagfiles_folder)
        print(f"Found {len(bagfiles)} bagfile(s) in {args.bagfiles_folder}")

        # Load each bagfile using BagfileLoader
        loaders = []
        for bagfile_path in bagfiles:
            print(f"\nLoading bagfile: {os.path.basename(bagfile_path)}")
            try:
                loader = BagfileLoader(bagfile_path)
                loaders.append(loader)
                print(f"Successfully loaded {os.path.basename(bagfile_path)}")
            except (FileNotFoundError, OSError, ValueError) as e:
                print(f"Error loading {os.path.basename(bagfile_path)}: {e}")
                continue

        print(f"\nSuccessfully loaded {len(loaders)} out of {len(bagfiles)} bagfiles")

        # Process each loader's data using sliding windows
        all_circle_fits = []

        for i, loader in enumerate(loaders):
            print(
                f"\nProcessing bagfile {i+1}/{len(loaders)}: {len(loader.bagfile_records_dicts)} records"
            )

            # Apply sliding window circle fitting to this window
            circle_fits = fit_circles(
                bagfile_records=loader.bagfile_records_dicts,
                sliding_window_size=args.window_size,
                debug=args.debug,
            )
            print(f"Successfully fitted {len(circle_fits)} circles")
            all_circle_fits.extend(circle_fits)

        # Write CSV file with all fitted circles
        if all_circle_fits:
            csv_filename = os.path.join(args.bagfiles_folder, "fitted_circles.csv")
            print(f"\nWriting {len(all_circle_fits)} fitted circles to {csv_filename}")

            # Save all_circle_fits using pandas
            df = pd.DataFrame(all_circle_fits)
            df.to_csv(csv_filename, index=False)

            print(f"CSV file written successfully with {len(all_circle_fits)} rows")
        else:
            print("\nNo fitted circles found, skipping CSV output")

    except (FileNotFoundError, NotADirectoryError, OSError) as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
