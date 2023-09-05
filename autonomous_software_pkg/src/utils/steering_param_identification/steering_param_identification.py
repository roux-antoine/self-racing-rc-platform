import argparse
import math

import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objs as go
import plotly.subplots as sp
import rosbag
import scipy
import utm

KNOTS_TO_MPS = 0.514444
EARTH_RADIUS = 6371000  # approximately 6,371 kilometers
MAX_RADIUS_THRESHOLD = 30  # m
GPS_PUBLISHING_RATE = 10


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


class BagPlotter:
    def __init__(self, bag_path, start_time, end_time) -> None:

        self.bag_path = bag_path
        self.start_time = start_time
        self.end_time = end_time

        # Initialize things
        self.timestamps = []
        self.speeds = []  # in m/s
        self.track_angles = []  # in deg
        self.latitudes = []
        self.longitudes = []
        self.steering_angles = []
        self.utm_xs = []
        self.utm_ys = []
        self.window_start_time = None
        self.window_end_time = None

        # Open the bagfile
        with rosbag.Bag(bag_path, "r") as bag:
            for topic, msg, t in bag.read_messages(
                topics=["/gps_info", "/arduino_logging"]
            ):
                if topic == "/gps_info":
                    self.timestamps.append(t.to_sec())
                    self.speeds.append(msg.speed * KNOTS_TO_MPS)
                    self.track_angles.append(msg.track)
                    self.latitudes.append(msg.lat)
                    self.longitudes.append(msg.lon)
                elif topic == "/arduino_logging":
                    self.steering_angles.append(msg.steering_angle_final)

        # zero-ing the timestamps
        self.zeroed_timestamps = np.array(self.timestamps) - self.timestamps[0]

        # Smooth some signals
        self.track_angles_smoothed = scipy.ndimage.gaussian_filter1d(
            self.track_angles, 1
        )
        self.speeds_smoothed = scipy.ndimage.gaussian_filter1d(self.speeds, 1)
        self.latitudes_smoothed = scipy.ndimage.gaussian_filter1d(self.latitudes, 1)
        self.longitudes_smoothed = scipy.ndimage.gaussian_filter1d(self.longitudes, 1)

        # Compute UTM corrdinates
        for latitude, longitude in zip(self.latitudes, self.longitudes):
            utm_x, utm_y, _, _ = utm.from_latlon(latitude, longitude)
            self.utm_xs.append(utm_x)
            self.utm_ys.append(utm_y)
        self.utm_xs_smoothed = scipy.ndimage.gaussian_filter1d(self.utm_xs, 1)
        self.utm_ys_smoothed = scipy.ndimage.gaussian_filter1d(self.utm_ys, 1)

    def compare_3_speed_computations(self):
        """
        Compare speeds computed from 3 different ways:
        - distance between 2 points computed with haversine from lat/lon, times the rate
        - distance between 2 points computed with linalg.norm from utm, times the rate
        - raw speed from the gps
        """
        distances_haversine = []
        distances_utm = []
        # adding some 0s to have an array of the correct length
        distances_haversine.append(0)
        distances_utm.append(0)
        for i in range(len(self.latitudes) - 1):
            distances_haversine.append(
                haversine(
                    self.latitudes[i],
                    self.longitudes[i],
                    self.latitudes[i + 1],
                    self.longitudes[i + 1],
                )
            )
            distances_utm.append(
                np.linalg.norm(
                    [
                        self.utm_xs[i + 1] - self.utm_xs[i],
                        self.utm_ys[i + 1] - self.utm_ys[i],
                    ]
                )
            )

        plt.plot(
            self.zeroed_timestamps,
            np.array(distances_haversine) * GPS_PUBLISHING_RATE,
            label="speed haversine",
        )  # factor 10 because 10Hz
        plt.plot(
            self.zeroed_timestamps,
            np.array(distances_utm) * GPS_PUBLISHING_RATE,
            label="speed utm",
        )  # factor 10 because 10Hz
        plt.plot(self.zeroed_timestamps, np.array(self.speeds), label="speed gps")
        plt.legend()
        plt.show()

    def compute_radius_from_smoothed_utm(self):
        """Compute the radius of the turn using utm coordinates and geometry"""
        self.centers_from_utm = []
        self.radiuses_from_utm = []
        for i in range(len(self.utm_xs_smoothed) - 1):
            center, radius = define_circle(
                [self.utm_xs_smoothed[i - 1], self.utm_ys_smoothed[i - 1]],
                [self.utm_xs_smoothed[i], self.utm_ys_smoothed[i]],
                [self.utm_xs_smoothed[i + 1], self.utm_ys_smoothed[i + 1]],
            )

            if center and radius < MAX_RADIUS_THRESHOLD:
                self.centers_from_utm.append(center)
                self.radiuses_from_utm.append(radius)
            else:
                self.centers_from_utm.append(None)
                self.radiuses_from_utm.append(None)
        # adding some 0s to have an array of the correct length
        self.centers_from_utm.append(0)
        self.radiuses_from_utm.append(0)

        # Option: Plot the circle of rotation using utm
        plt_center_of_rotation = False
        if plt_center_of_rotation:
            starting_point = 100
            for i, (center, radius) in enumerate(
                zip(
                    self.centers_from_utm[starting_point:],
                    self.radiuses_from_utm[starting_point:],
                )
            ):
                if center:
                    figure, axes = plt.subplots()
                    plt.scatter(self.utm_xs_smoothed, self.utm_ys_smoothed)
                    plt.scatter(
                        self.utm_xs_smoothed[i + starting_point],
                        self.utm_ys_smoothed[i + starting_point],
                        color="red",
                    )
                    plt.scatter(center[0], center[1])
                    circle = plt.Circle((center[0], center[1]), radius, fill=False)
                    axes.set_aspect(1)
                    axes.add_artist(circle)
                    plt.show()
                    plt.clf()

    def compute_radius_from_smoothed_velocities(self):
        """Compute the radius of the turn using the linear and angular velocities from the GPS"""
        # Computing the radius using the smoothed velocities from the GPS, valid with assumption of circle motion
        self.track_angles_derivatives = []
        self.radiuses_from_velocity = []
        self.accelerations_from_velocity = []
        for i in range(len(self.track_angles) - 1):
            angle_diff = self.track_angles[i + 1] - self.track_angles[i]
            # assumption: if the difference between 2 timestamps is greater than 180 degrees, we assume the 360 mark was crossed
            if angle_diff > 180:
                angle_diff = 360 - angle_diff
            elif angle_diff < -180:
                angle_diff = 360 + angle_diff
            time_diff = self.timestamps[i + 1] - self.timestamps[i]
            track_angles_derivative = angle_diff / time_diff

            if track_angles_derivative != 0:
                radius_from_velocity = self.speeds_smoothed[i] / abs(
                    (track_angles_derivative * np.pi / 180)
                )
            else:
                radius_from_velocity = 0

            acceleration = (
                radius_from_velocity * (track_angles_derivative * np.pi / 180) ** 2
            )

            self.track_angles_derivatives.append(track_angles_derivative)
            self.radiuses_from_velocity.append(radius_from_velocity)
            self.accelerations_from_velocity.append(acceleration)
        # adding some 0s to have an array of the correct length
        self.track_angles_derivatives.append(0)
        self.radiuses_from_velocity.append(0)
        self.accelerations_from_velocity.append(0)
        # bounding the very large radiuses
        self.radiuses_from_velocity = [
            radius if radius < MAX_RADIUS_THRESHOLD else MAX_RADIUS_THRESHOLD
            for radius in self.radiuses_from_velocity
        ]

    def plot_traces(self):
        """Plot traces of the different signals"""
        speed_trace = go.Scatter(
            x=self.zeroed_timestamps,
            y=self.speeds,
            mode="lines",
            name="Speed (m/s)",
        )
        speed_trace_smoothed = go.Scatter(
            x=self.zeroed_timestamps,
            y=self.speeds_smoothed,
            mode="lines",
            name="Speed smoothed (m/s)",
        )
        track_angle_trace = go.Scatter(
            x=self.zeroed_timestamps,
            y=self.track_angles,
            mode="lines",
            name="Track Angle (degrees)",
        )
        track_angle_smoothed_trace = go.Scatter(
            x=self.zeroed_timestamps,
            y=self.track_angles_smoothed,
            mode="lines",
            name="Track Angle smoothed (degrees)",
        )
        radiuses_from_utm_trace = go.Scatter(
            x=self.zeroed_timestamps,
            y=self.radiuses_from_utm,
            mode="lines",
            name="Turning radius computed from utm (m)",
        )
        radiuses_from_velocity_trace = go.Scatter(
            x=self.zeroed_timestamps,
            y=self.radiuses_from_velocity,
            mode="lines",
            name="Turning radius computed from gps velocities (m)",
        )
        steering_angles_trace = go.Scatter(
            x=self.zeroed_timestamps,
            y=self.steering_angles,
            mode="lines",
            name="Steering command",
        )
        accelerations_from_velocity_trace = go.Scatter(
            x=self.zeroed_timestamps,
            y=self.accelerations_from_velocity,
            mode="lines",
            name="Acceleration computed from gps velocities (m/s^2)",
        )

        # Create a subplot with subplots
        fig = sp.make_subplots(
            rows=5,
            cols=1,
            subplot_titles=(
                "Speed",
                "Track Angle",
                "Turning radius",
                "Steering command",
                "Acceleration",
            ),
        )

        # Add the traces to the respective subplots
        fig.add_trace(speed_trace, row=1, col=1)
        fig.add_trace(speed_trace_smoothed, row=1, col=1)
        fig.add_trace(track_angle_trace, row=2, col=1)
        fig.add_trace(track_angle_smoothed_trace, row=2, col=1)
        fig.add_trace(radiuses_from_utm_trace, row=3, col=1)
        fig.add_trace(radiuses_from_velocity_trace, row=3, col=1)
        fig.add_trace(steering_angles_trace, row=4, col=1)
        fig.add_trace(accelerations_from_velocity_trace, row=5, col=1)

        # Update the layout of the subplots
        fig.update_xaxes(title_text="Time", row=1, col=1)
        fig.update_yaxes(title_text="Speed (m/s)", row=1, col=1)
        fig.update_xaxes(title_text="Time", row=2, col=1)
        fig.update_yaxes(title_text="Track Angle (degrees)", row=2, col=1)
        fig.update_yaxes(title_text="Turning radius (m)", row=3, col=1)
        fig.update_yaxes(title_text="Steering angle final cmd", row=4, col=1)
        fig.update_yaxes(title_text="Acceleration (m/s^2)", row=5, col=1)

        fig.update_xaxes(range=[self.start_time, self.end_time])

        # adding the horizontal lines if they have been computed
        if self.window_start_time and self.window_end_time:
            fig.add_vrect(
                self.window_start_time,
                self.window_end_time,
                row="all",
                col="all",
                fillcolor="green",
                opacity=0.25,
                line_width=0,
            )
            fig.add_hline(y=self.mean_window_speed, line_dash="solid", row=1, col=1)
            fig.add_hline(
                y=self.mean_window_radius_from_velocity, line_dash="solid", row=3, col=1
            )
            fig.add_hline(
                y=self.mean_window_steering_angle, line_dash="solid", row=4, col=1
            )
            fig.add_hline(
                y=self.mean_window_acceleration_from_velocity_window,
                line_dash="solid",
                row=5,
                col=1,
            )

        # Save the subplots
        fig.write_image("combined_subplot.png")
        fig.write_html("combined_subplot.html")

        # Show the subplots
        fig.show()

    def compute_average_values_in_window(self):
        """Asks the user for the time window to compute the average values in"""
        self.window_start_time = float(input("Enter start time for the window: "))
        self.window_end_time = float(input("Enter end time for the window: "))

        window_start_index = next(
            (
                i
                for i, x in enumerate(self.zeroed_timestamps)
                if x > self.window_start_time
            ),
            -1,
        )
        window_end_index = next(
            (
                i
                for i, x in enumerate(self.zeroed_timestamps)
                if x > self.window_end_time
            ),
            -1,
        )

        speeds_window = self.speeds[window_start_index:window_end_index]
        radiuses_from_velocity_window = self.radiuses_from_velocity[
            window_start_index:window_end_index
        ]
        steering_angles_window = self.steering_angles[
            window_start_index:window_end_index
        ]
        accelerations_from_velocity_window = self.accelerations_from_velocity[
            window_start_index:window_end_index
        ]

        self.mean_window_speed = round(np.mean(speeds_window), 2)
        self.mean_window_radius_from_velocity = round(
            np.mean(radiuses_from_velocity_window), 2
        )
        self.mean_window_steering_angle = round(np.mean(steering_angles_window), 2)
        self.mean_window_acceleration_from_velocity_window = round(
            np.mean(accelerations_from_velocity_window), 2
        )

        print(f"mean_window_speed: {self.mean_window_speed}")
        print(
            f"mean_window_radius_from_velocity: {self.mean_window_radius_from_velocity}"
        )
        print(f"mean_window_steering_angle: {self.mean_window_steering_angle}")
        print(
            f"mean_window_acceleration_from_velocity_window: {self.mean_window_acceleration_from_velocity_window}"
        )


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

    bag_plotter = BagPlotter(
        bag_path=bag_path, start_time=start_time, end_time=end_time
    )
    bag_plotter.compute_radius_from_smoothed_utm()
    bag_plotter.compute_radius_from_smoothed_velocities()
    bag_plotter.plot_traces()
    bag_plotter.compute_average_values_in_window()
    bag_plotter.plot_traces()
