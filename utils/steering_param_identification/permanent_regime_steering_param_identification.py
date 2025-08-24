#!/usr/bin/env python3

import argparse
import os
import glob
from typing import List
import sys
import numpy as np
import plotly.graph_objects as go

# HACK antoine, fix the import
sys.path.append(
    "/Users/antoineroux/Downloads/self-racing-rc-platform-master/src/utils/geometry_utils_pkg/src/geometry_utils_pkg"
)
from bagfile_loader import BagfileLoader  # noqa: E402
from geometry_utils import fit_circle_to_points  # noqa: E402


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
    radius_threshold: float = 50,
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
    for i in range(0, len(sorted_records) - sliding_window_size + 1, 5):
        window_records = sorted_records[i : i + sliding_window_size]

        # Extract (x, y) points from the window
        points = []
        for _, record in window_records:
            points.append((record.state.x, record.state.y))

        try:
            # Fit circle to the points in this window
            center_x, center_y, radius = fit_circle_to_points(points)

            if radius < radius_threshold:
                # Store the result with the middle timestamp of the window
                if debug:

                    fig = go.Figure()
                    x_vals, y_vals = zip(*points)
                    fig.add_trace(
                        go.Scatter(x=x_vals, y=y_vals, mode="markers", name="Points")
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
                                marker=dict(color="gray", size=5, opacity=0.5),
                            )
                        )
                    # Plot fitted circle
                    theta = np.linspace(0, 2 * np.pi, 100)
                    circle_x = center_x + radius * np.cos(theta)
                    circle_y = center_y + radius * np.sin(theta)
                    fig.add_trace(
                        go.Scatter(
                            x=circle_x, y=circle_y, mode="lines", name="Fitted Circle"
                        )
                    )

                    fig.update_layout(
                        title="Circle Fit Debug",
                        xaxis_title="X",
                        yaxis_title="Y",
                        showlegend=True,
                    )
                    fig.update_yaxes(scaleanchor="x", scaleratio=1)
                    fig.show()
                    input()

                circle_fits.append(
                    {
                        "center_x": center_x,
                        "center_y": center_y,
                        "radius": radius,
                        "window_start_idx": i,
                        "window_end_idx": i + sliding_window_size - 1,
                        "points": points,
                    }
                )

        except (ValueError, np.linalg.LinAlgError) as e:
            print(
                f"Warning: Failed to fit circle for window starting at index {i}: {e}"
            )
            continue

    print(f"Successfully fitted circles to {len(circle_fits)} windows")
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
        help="Number of points in sliding window for circle fitting (default: 10)",
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

        for i, loader in enumerate(loaders):
            print(
                f"\nProcessing bagfile {i+1}/{len(loaders)}: {len(loader.bagfile_records_dicts)} records"
            )

            # Apply sliding window circle fitting to this window
            _ = fit_circles(
                bagfile_records=loader.bagfile_records_dicts,
                sliding_window_size=args.window_size,
                debug=args.debug,
            )

    except (FileNotFoundError, NotADirectoryError, OSError) as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
