import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.figure_factory as ff
import plotly.graph_objects as go
import scipy
from scipy.spatial import Delaunay

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Process bag files with start and end times."
    )
    parser.add_argument(
        "--csv-path",
        "-c",
        type=str,
        required=True,
        help="Path to csv to process",
    )
    args = parser.parse_args()

    df = pd.read_csv(args.csv_path)

    steering_cmd = df["mean_window_steering_angle"]
    speed = df["mean_window_speed"]
    diff_to_neutral = df["mean_window_steering_angle_deviation_from_center"]
    radius = df["mean_window_radius_from_velocity"]

    # Plotting with matplotlib

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_trisurf(steering_cmd, speed, radius, linewidth=0)
    ax.set_xlabel("steering_cmd")
    ax.set_ylabel("speed")
    ax.set_zlabel("radius")
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_trisurf(diff_to_neutral, speed, radius, linewidth=0)
    ax.set_xlabel("diff_to_neutral")
    ax.set_ylabel("speed")
    ax.set_zlabel("radius")
    plt.show()

    # Trying things out with prediction

    points2D = np.vstack([radius, speed]).T
    tri = Delaunay(points2D)
    simplices = tri.simplices
    fig = ff.create_trisurf(x=radius, y=speed, z=diff_to_neutral, simplices=simplices)
    scatter_pred = go.Scatter3d(
        x=radius, y=speed, z=diff_to_neutral, mode="markers", marker=dict(size=5)
    )
    fig.add_trace(scatter_pred)

    points = []
    for r, s in zip(radius, speed):
        points.append([r, s])

    query = [[6, 6]]

    interpolator = scipy.interpolate.RBFInterpolator(
        points, diff_to_neutral, kernel="linear"
    )
    value = interpolator(query)

    scatter_mesh = go.Scatter3d(
        x=[query[0][0]],
        y=[query[0][1]],
        z=[value[0]],
        mode="markers",
        marker=dict(size=20),
    )

    fig.add_trace(scatter_mesh)

    fig.update_scenes(
        xaxis_title_text="radius",
        yaxis_title_text="speed",
        zaxis_title_text="diff_to_neutral",
    )

    fig.show()
    fig.write_html("surface.html")
