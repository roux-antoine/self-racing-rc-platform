import rosbag
import math
import plotly.graph_objs as go
import plotly.subplots as sp
import numpy as np
import utm
import matplotlib.pyplot as plt
import scipy
import argparse


KNOTS_TO_MPS = 0.514444
EARTH_RADIUS = 6371000  # approximately 6,371 kilometers
MAX_RADIUS_THRESHOLD = 30  # m


def haversine(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Calculate the distance in meters
    distance = EARTH_RADIUS * c

    return distance


def define_circle(p1, p2, p3):
    """
    Returns the center and radius of the circle passing the given 3 points.
    In case the 3 points form a line, returns (None, infinity).
    """
    temp = p2[0] * p2[0] + p2[1] * p2[1]
    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

    if abs(det) < 1.0e-6:
        return (None, np.inf)

    # Center of circle
    cx = (bc * (p2[1] - p3[1]) - cd * (p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

    radius = np.sqrt((cx - p1[0]) ** 2 + (cy - p1[1]) ** 2)
    return ((cx, cy), radius)


if __name__ == "__main__":

    # Parse args
    parser = argparse.ArgumentParser(
        description="Process bag files with start and end times."
    )
    parser.add_argument("--bag-path", "-b", type=str, help="Path to the bag file.")
    parser.add_argument(
        "--start-time",
        "-s",
        type=float,
        default=None,
        help="Start time (in bag time) for plotting.",
    )
    parser.add_argument(
        "--end-time",
        "-e",
        type=float,
        default=None,
        help="End time (in bag time) for plotting.",
    )
    args = parser.parse_args()
    bag_path = args.bag_path
    start_time = args.start_time
    end_time = args.end_time

    # Initialize lists
    timestamps = []
    speed_data = []  # in m/s
    track_angle_data = []  # in deg
    latitudes = []
    longitudes = []
    steering_angles = []
    utm_xs = []
    utm_ys = []

    # Open the bagfile
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(
            topics=["/gps_info", "/arduino_logging"]
        ):
            if topic == "/gps_info":
                timestamps.append(t.to_sec())
                speed_data.append(msg.speed * KNOTS_TO_MPS)
                track_angle_data.append(msg.track)
                latitudes.append(msg.lat)
                longitudes.append(msg.lon)
            elif topic == "/arduino_logging":
                steering_angles.append(msg.steering_angle_final)

    # Smooth some signals
    track_angle_data_smoothed = scipy.ndimage.gaussian_filter1d(track_angle_data, 1)
    speed_data_smoothed = scipy.ndimage.gaussian_filter1d(speed_data, 1)

    # Compute UTM corrdinates
    for latitude, longitude in zip(latitudes, longitudes):
        utm_x, utm_y, _, _ = utm.from_latlon(latitude, longitude)
        utm_xs.append(utm_x)
        utm_ys.append(utm_y)

    # Option checking if distances make sense
    check_if_3_distance_computations_match = False
    if check_if_3_distance_computations_match:
        distances_haversine = []
        distances_utm = []

        for i in range(len(latitudes) - 1):
            distances_haversine.append(
                haversine(
                    latitudes[i], longitudes[i], latitudes[i + 1], longitudes[i + 1]
                )
            )
            distances_utm.append(
                np.linalg.norm([utm_xs[i + 1] - utm_xs[i], utm_ys[i + 1] - utm_ys[i]])
            )

        plt.plot(np.array(distances_haversine) * 10)  # factor 10 because 10Hz
        plt.plot(np.array(distances_utm) * 10)  # factor 10 because 10Hz
        plt.plot(np.array(speed_data))
        plt.show()

    # Compute the center of rotation using utm, with optional steps to "average" things out
    step = 5
    centers_from_utm = []
    radiuses_from_utm = []
    for i in range(step):
        centers_from_utm.append(0)
        radiuses_from_utm.append(0)
    for i in range(step, len(utm_xs) - step, step):
        center, radius = define_circle(
            [utm_xs[i - step], utm_ys[i - step]],
            [utm_xs[i], utm_ys[i]],
            [utm_xs[i + step], utm_ys[i + step]],
        )

        if center and radius < MAX_RADIUS_THRESHOLD:
            centers_from_utm.append(center)
            radiuses_from_utm.append(radius)
        else:
            centers_from_utm.append(None)
            radiuses_from_utm.append(None)
        for i in range(step - 1):
            centers_from_utm.append(0)
            radiuses_from_utm.append(0)

    for i in range(step):
        centers_from_utm.append(0)
        radiuses_from_utm.append(0)

    # Option: Plot the circle of rotation using utm
    plt_center_of_rotation = False
    if plt_center_of_rotation:
        starting_point = 100
        for i, (center, radius) in enumerate(
            zip(centers_from_utm[starting_point:], radiuses_from_utm[starting_point:])
        ):
            if center:
                figure, axes = plt.subplots()
                plt.scatter(utm_xs, utm_ys)
                plt.scatter(
                    utm_xs[i + starting_point], utm_ys[i + starting_point], color="red"
                )
                plt.scatter(center[0], center[1])
                circle = plt.Circle((center[0], center[1]), radius, fill=False)
                axes.set_aspect(1)
                axes.add_artist(circle)
                plt.show()
                plt.clf()

    # Computing the radius using the smoothed velocities from the GPS, valid with assumption of circle motion
    track_angle_data_derivatives = []
    angle_diffs = []
    radiuses_from_velocity = []
    step = 1
    for i in range(0, len(track_angle_data_smoothed) - step, step):
        angle_diff = track_angle_data_smoothed[i + step] - track_angle_data_smoothed[i]
        # assumption: if the difference between 2 timestamps is greater than 180 degrees, we assume the 360 mark was crossed
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        elif angle_diff < -180:
            angle_diff = 360 + angle_diff
        time_diff = timestamps[i + step] - timestamps[i]
        track_angle_data_derivative = angle_diff / time_diff
        if track_angle_data_derivative != 0:
            radius_from_velocity = speed_data_smoothed[i] / abs(
                (track_angle_data_derivative * np.pi / 180)
            )
        else:
            radius_from_velocity = 0
        angle_diffs.append(angle_diff)
        track_angle_data_derivatives.append(track_angle_data_derivative)
        radiuses_from_velocity.append(radius_from_velocity)
        for i in range(step - 1):
            angle_diffs.append(0)
            track_angle_data_derivatives.append(0)
            radiuses_from_velocity.append(0)
    for i in range(step):
        angle_diffs.append(0)
        track_angle_data_derivatives.append(0)
        radiuses_from_velocity.append(0)
    radiuses_from_velocity = [
        radius if radius < MAX_RADIUS_THRESHOLD else MAX_RADIUS_THRESHOLD
        for radius in radiuses_from_velocity
    ]

    # zero-ing the timestamps
    timestamps = np.array(timestamps) - timestamps[0]

    # Create traces
    speed_trace = go.Scatter(
        x=timestamps, y=speed_data, mode="lines", name="Speed (m/s)"
    )
    speed_trace_smoothed = go.Scatter(
        x=timestamps, y=speed_data_smoothed, mode="lines", name="Speed smoothed (m/s)"
    )
    track_angle_trace = go.Scatter(
        x=timestamps, y=track_angle_data, mode="lines", name="Track Angle (degrees)"
    )
    track_angle_smoothed_trace = go.Scatter(
        x=timestamps,
        y=track_angle_data_smoothed,
        mode="lines",
        name="Track Angle smoothed (degrees)",
    )
    radiuses_from_utm_trace = go.Scatter(
        x=timestamps,
        y=radiuses_from_utm,
        mode="lines",
        name="Turning radius computed from utm (m)",
    )
    radiuses_from_velocity_trace = go.Scatter(
        x=timestamps,
        y=radiuses_from_velocity,
        mode="lines",
        name="Turning radius computed form gps velocities (m)",
    )
    steering_angles_trace = go.Scatter(
        x=timestamps, y=steering_angles, mode="lines", name="Steering command"
    )

    # Create a subplot with two subplots (Speed and Track Angle)
    # if start_time != -1 and end_time != -1:
    fig = sp.make_subplots(
        rows=4,
        cols=1,
        subplot_titles=("Speed", "Track Angle", "Turning radius", "Steering command"),
    )

    # fig.update_layout(xaxis_range=[args.start_time, args.end_time])

    # Add the traces to the respective subplots
    fig.add_trace(speed_trace, row=1, col=1)
    fig.add_trace(speed_trace_smoothed, row=1, col=1)
    fig.add_trace(track_angle_trace, row=2, col=1)
    fig.add_trace(track_angle_smoothed_trace, row=2, col=1)
    fig.add_trace(radiuses_from_utm_trace, row=3, col=1)
    fig.add_trace(radiuses_from_velocity_trace, row=3, col=1)
    fig.add_trace(steering_angles_trace, row=4, col=1)

    # Update the layout of the subplots
    fig.update_xaxes(title_text="Time", row=1, col=1)
    fig.update_yaxes(title_text="Speed (m/s)", row=1, col=1)
    fig.update_xaxes(title_text="Time", row=2, col=1)
    fig.update_yaxes(title_text="Track Angle (degrees)", row=2, col=1)
    fig.update_yaxes(title_text="Turning radius (m)", row=3, col=1)
    fig.update_yaxes(title_text="Steering angle final cmd", row=4, col=1)
    # TODO the rest

    fig.update_xaxes(range=[args.start_time, args.end_time])

    # Save the subplots to a single PNG file
    fig.write_image("combined_subplot.png")
    fig.write_html("combined_subplot.html")

    # Show the subplots
    fig.show()
