import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.figure_factory as ff
import plotly.graph_objects as go
import scipy
from scipy.spatial import Delaunay
from sklearn.linear_model import LinearRegression
from sklearn.neighbors import KNeighborsRegressor


def model_v1(radius, current_velocity):
    return 27 * 1.2 / radius


def model_v2(radius, current_velocity):
    bound_region_1 = 1.5
    bound_region_2 = 5
    bound_region_3 = 8
    coeff_region_1 = 27 * 1.25 # max steering_diff * radius of circle at max lateral acceleration
    coeff_region_2 = 24 * 2.3
    coeff_region_3 = 26 * 4
    if current_velocity == 0:
        # not too sure what to do here, figure out
        coeff = 1000
    elif current_velocity > 0 and current_velocity <= bound_region_1:
        coeff = 1 / (
            coeff_region_1
        )
    elif current_velocity > bound_region_1 and current_velocity <= bound_region_2:
        coeff = 1 / (
            coeff_region_1
            + (current_velocity - bound_region_1)
            * (coeff_region_2 - coeff_region_1)
            / (bound_region_2 - bound_region_1)
        )
    elif current_velocity > bound_region_2 and current_velocity <= bound_region_3:
        coeff = 1 / (
            coeff_region_2
            + (current_velocity - bound_region_2) * (coeff_region_3 - coeff_region_2) / (bound_region_3 - bound_region_2)
        )
    elif current_velocity > bound_region_3:
        coeff = 1 / coeff_region_3

    sterring_diff = 1 / radius / coeff 
    return min(27, sterring_diff)

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

    # plotting in 2D

    max_accel = 9.81
    velocities = np.linspace(start=0, stop=10, num=20)
    radiuses_at_limit = velocities ** 2 / max_accel
    fig = plt.figure()
    plt.scatter(speed, radius)
    plt.plot(velocities, radiuses_at_limit)
    plt.xlabel("speed")
    plt.ylabel("radius")
    plt.show()


    # Plotting in 3D with matplotlib

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_trisurf(steering_cmd, speed, radius, linewidth=0)
    ax.set_xlabel("steering_cmd")
    ax.set_ylabel("speed")
    ax.set_zlabel("radius")
    plt.show()

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection="3d")
    # surf = ax.plot_trisurf(diff_to_neutral, speed, radius, linewidth=0)
    # ax.set_xlabel("diff_to_neutral")
    # ax.set_ylabel("speed")
    # ax.set_zlabel("radius")
    # plt.show()

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection="3d")
    # surf = ax.plot_trisurf(speed, radius, diff_to_neutral, linewidth=0)
    # ax.set_xlabel("speed")
    # ax.set_ylabel("radius")
    # ax.set_zlabel("diff_to_neutral")
    # plt.show()


    # ###### delaunay stuff (useless) ######
    # points = []
    # for d, s, r in zip(diff_to_neutral, speed, radius):
    #     points.append([d, s, r])
    # points = np.array(points)
    # tri = scipy.spatial.Delaunay(points)
    # for simplex in tri.simplices:
    #     # print(points[simplex])
    #     print(simplex)
    #     print("-")

    # plotting with plotly

    points2D = np.vstack([radius, speed]).T
    tri = Delaunay(points2D)
    simplices = tri.simplices
    fig = ff.create_trisurf(x=radius, y=speed, z=diff_to_neutral, simplices=simplices)
    scatter_mesh = go.Scatter3d(
        x=radius, y=speed, z=diff_to_neutral, mode="markers", marker=dict(size=5)
    )
    fig.add_trace(scatter_mesh)
    fig.update_layout(scene=dict(xaxis_title='radius', yaxis_title='speed', zaxis_title='steering diff'))
    fig.layout.scene.camera.projection.type = "orthographic"
    fig.show()


    # ###### Trying things out with prediction: linear regression ######
    X = []
    for r, s in zip(radius, speed):
        X.append([1, r, s])  # degree 1
        # X.append([1, r, s, r**2, s**2, r*s]) # degree 2
        # X.append([1, r, s, r**2, s**2, r*s, r**3, s**3, r*s**2, r**2*s]) # degree 3

    X = np.array(X)
    y = diff_to_neutral
    linear_reg = LinearRegression().fit(X, y)

    points2D = np.vstack([radius, speed]).T
    tri = Delaunay(points2D)
    simplices = tri.simplices
    fig = ff.create_trisurf(x=radius, y=speed, z=diff_to_neutral, simplices=simplices)
    scatter_mesh = go.Scatter3d(
        x=radius, y=speed, z=diff_to_neutral, mode="markers", marker=dict(size=5)
    )

    X_test = []
    for r in range(1, 10):
        for s in range(2, 9):
            X_test.append([1, r, s])  # degree 1
            # X_test.append([1, r, s, r**2, s**2, r*s])  # degree 2
            # X_test.append([1, r, s, r**2, s**2, r*s, r**3, s**3, r*s**2, r**2*s])  # degree 3

    X_test = np.array(X_test)
    predicted = linear_reg.predict(X_test)

    zs = []
    scatter_our_model = go.Scatter3d(
            x=X_test[:,1],
            y=X_test[:,2],
            z=model_v1(X_test[:,1], X_test[:, 2]),
            mode="markers",
            marker=dict(size=5, color="blue"),
        )

    scatter_pred = go.Scatter3d(
            x=X_test[:,1],
            y=X_test[:,2],
            z=predicted,
            mode="markers",
            marker=dict(size=5, color="red"),
        )

    zs = []
    for r, s in zip(X_test[:,1], X_test[:,2]):
        zs.append(model_v2(r, s))
    scatter_my_model = go.Scatter3d(
            x=X_test[:,1],
            y=X_test[:,2],
            z=zs,
            mode="markers",
            marker=dict(size=5, color="orange"),
        )


    fig.add_trace(scatter_mesh)
    # fig.add_trace(scatter_pred)
    # fig.add_trace(scatter_our_model)
    fig.add_trace(scatter_my_model)
    fig.update_layout(scene=dict(xaxis_title='radius', yaxis_title='speed', zaxis_title='steering diff'))
    fig.layout.scene.camera.projection.type = "orthographic"
    fig.show()

    # evaluate on measured set
    predicted_our_model = model_v1(X[:,1], X[:, 2])
    predicted_my_model = []
    for r, s in zip(X[:,1], X[:,2]):
        predicted_my_model.append(model_v2(r, s))
    
    print(predicted_our_model)
    fig = go.Figure()

    fig.add_trace(go.Scatter3d(
            x=X[:,1],
            y=X[:,2],
            z=predicted_our_model - y,
            mode="markers",
            marker=dict(size=5, color="blue"),
        ))
    # fig.add_trace(go.Scatter3d(
    #         x=X[:,1],
    #         y=X[:,2],
    #         z=predicted_my_model - y,
    #         mode="markers",
    #         marker=dict(size=5, color="orange"),
    #     ))
    fig.layout.scene.camera.projection.type = "orthographic"

    print(np.linalg.norm(predicted_our_model - y))
    print(np.linalg.norm(predicted_my_model - y))
    fig.update_layout(scene=dict(xaxis_title='radius', yaxis_title='speed', zaxis_title='error'))
    fig.show()


    # ###### Trying things out with prediction: k nearest neighbors ######

    X = []
    for r, s in zip(radius, speed):
        X.append([r, s])

    X = np.array(X)
    y = diff_to_neutral
    knn_model = KNeighborsRegressor(n_neighbors=2, weights="distance").fit(X, y)

    points2D = np.vstack([radius, speed]).T
    tri = Delaunay(points2D)
    simplices = tri.simplices
    fig = ff.create_trisurf(x=radius, y=speed, z=diff_to_neutral, simplices=simplices)
    scatter_mesh = go.Scatter3d(
        x=radius, y=speed, z=diff_to_neutral, mode="markers", marker=dict(size=5)
    )

    X_test = []
    for r in range(1, 10):
        for s in range(1, 9):
            X_test.append([r, s])
    X_test = np.array(X_test)
    predicted = knn_model.predict(X_test)
    print(predicted)
    scatter_pred = go.Scatter3d(
            x=X_test[:,0],
            y=X_test[:,1],
            z=predicted,
            mode="markers",
            marker=dict(size=5, color="red"),
        )
    fig.add_trace(scatter_mesh)
    fig.add_trace(scatter_pred)
    fig.update_layout(scene=dict(xaxis_title='radius', yaxis_title='speed', zaxis_title='steering diff'))
    fig.show()
