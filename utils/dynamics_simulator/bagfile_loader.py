import sys
from typing import Dict

import matplotlib.pyplot as plt
import ros_geometry_utils
import rosbag
from car_models import State
import plotly.graph_objs as go
import numpy as np


COLORS = [
    "blue",
    "red",
    "green",
    "orange",
    "purple",
    "brown",
    "pink",
    "gray",
    "olive",
    "cyan",
]
KNOTS_TO_MPS = 0.514444
EARTH_RADIUS = 6371000  # approximately 6,371 kilometers
MAX_RADIUS_THRESHOLD = 50  # m


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


def compute_radius_3_points(x_vals, y_vals):
    """TODO"""
    centers_from_utm = [0]
    radiuses_from_utm = [0]

    for i in range(1, len(x_vals) - 1):
        center, radius = define_circle(
            [x_vals[i - 1], y_vals[i - 1]],
            [x_vals[i], y_vals[i]],
            [x_vals[i + 1], y_vals[i + 1]],
        )

        if center and radius < MAX_RADIUS_THRESHOLD:
            centers_from_utm.append(center)
            radiuses_from_utm.append(radius)
        else:
            centers_from_utm.append(None)
            radiuses_from_utm.append(None)

    # adding some 0s to have an array of the correct length
    centers_from_utm.append(0)
    radiuses_from_utm.append(0)

    return centers_from_utm, radiuses_from_utm


class BagfileRecord:
    def __init__(
        self, state: State, steering_cmd: float, steering_fbk: float, fix_type: str
    ) -> None:
        self.state = state
        self.steering_cmd = steering_cmd
        self.steering_fbk = steering_fbk
        self.fix_type = fix_type


class BagfileSectionExtractor:
    def __init__(
        self, bag_path: str, min_window_size: int = 10, debug: bool = True
    ) -> None:
        self.bag_path = bag_path
        self.min_window_size = min_window_size
        self.debug = debug
        self.bagfile_records_dicts: Dict[float, BagfileRecord] = {}
        self.sorted_timestamps = []
        self.windows = []

    def load(self) -> None:

        xs = []
        ys = []
        yaws = []
        velocities = []
        servo_cmds = []
        steering_fbk = []
        fix_types = []
        pose_timestamps = []
        velocity_timestamps = []
        arduino_timestamps = []
        gps_timestamps = []

        with rosbag.Bag(self.bag_path) as bag:
            for topic, msg, t in bag.read_messages(
                topics=[
                    "/current_pose",
                    "/current_velocity",
                    "/arduino_logging",
                    "/gps_info",
                ]
            ):
                if topic == "/current_pose":
                    orientation_as_vec = [
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w,
                    ]
                    (roll, pitch, yaw) = ros_geometry_utils.euler_from_quaternion(
                        orientation_as_vec
                    )
                    position = msg.pose.position

                    pose_timestamps.append(t.to_sec())
                    xs.append(position.x)
                    ys.append(position.y)
                    yaws.append(yaw)

                elif topic == "/current_velocity":
                    velocity_timestamps.append(t.to_sec())
                    velocities.append(msg.twist.linear.x)

                elif topic == "/arduino_logging":
                    arduino_timestamps.append(t.to_sec())
                    servo_cmds.append(msg.steering_cmd_final)
                    steering_fbk.append(msg.steering_fbk)

                elif topic == "/gps_info":
                    gps_timestamps.append(t.to_sec())
                    fix_types.append(msg.mode_indicator)

        # first we align the pose and velocities
        if abs(pose_timestamps[0] - velocity_timestamps[0]) < 0.01:
            # then it means that the messages from the two topics are aligned
            for t, x, y, yaw, velocity in zip(
                pose_timestamps, xs, ys, yaws, velocities
            ):
                # setting the unknowns as -1, will be populated later
                bagfile_record = BagfileRecord(
                    state=State(x=x, y=y, yaw=yaw, v=velocity),
                    steering_cmd=-1,
                    steering_fbk=-1,
                    fix_type=-1,
                )
                self.bagfile_records_dicts[t] = bagfile_record

        else:
            print("figure it out")
            sys.exit()

        self.sorted_timestamps = list(self.bagfile_records_dicts.keys())
        self.sorted_timestamps.sort()

        # next we align with the arduino messages.
        search_index = 0
        t_diff = []
        for t in self.sorted_timestamps:
            for i in range(search_index, len(arduino_timestamps) - 1):
                i += 1
                if arduino_timestamps[i] > t:
                    t_diff.append(arduino_timestamps[i] - t)
                    self.bagfile_records_dicts[t].steering_cmd = servo_cmds[i]
                    self.bagfile_records_dicts[t].steering_fbk = steering_fbk[i]
                    break
        if self.debug:
            plt.plot(t_diff)
            plt.show()

        # next we align the gps messages
        search_index = 0
        t_diff = []
        for t in self.sorted_timestamps:
            for i in range(search_index, len(gps_timestamps) - 1):
                i += 1
                if gps_timestamps[i] > t:
                    t_diff.append(gps_timestamps[i] - t)
                    self.bagfile_records_dicts[t].fix_type = fix_types[i]
                    break

        if self.debug:
            plt.plot(t_diff)
            plt.show()

    def create_windows(self) -> None:
        # now we go over the datapoints and create windows where the fix type does not change, and is in {F, R}

        current_window = {}
        last_fix_type = None
        for t in self.sorted_timestamps:

            current_fix_type = self.bagfile_records_dicts[t].fix_type

            if not last_fix_type:
                if current_fix_type in ["F", "R"]:
                    last_fix_type = current_fix_type
                else:
                    continue
            else:
                if current_fix_type not in ["F", "R"]:
                    # we close the window
                    if len(current_window) >= self.min_window_size:
                        self.windows.append(current_window)
                    current_window = {}
                else:
                    if current_fix_type != last_fix_type:
                        # we close the window
                        if len(current_window) >= self.min_window_size:
                            self.windows.append(current_window)
                        current_window = {}
                    else:
                        # we continue the window
                        current_window[t] = self.bagfile_records_dicts[t]
            last_fix_type = current_fix_type

        if len(current_window) >= self.min_window_size:
            self.windows.append(current_window)

        print(f"Found {len(self.windows)} windows: ")
        for window in self.windows:
            window_start = min(window.keys())
            window_end = max(window.keys())
            print(f"  {window_start} to {window_end} ({len(window)})")

    def plot_xy_coordinates(self, use_windows: bool = False):
        if use_windows:
            if not self.windows:
                print("No windows available to plot.")
                return
            fig = go.Figure()
            for idx, window in enumerate(self.windows):
                x_vals = [record.state.x for record in window.values()]
                y_vals = [record.state.y for record in window.values()]
                color = COLORS[idx % len(COLORS)]
                fig.add_trace(
                    go.Scatter(
                        x=x_vals,
                        y=y_vals,
                        mode="lines+markers",
                        name=f"Window {idx+1}",
                        line=dict(color=color),
                    )
                )
        else:
            x_vals = [
                self.bagfile_records_dicts[t].state.x for t in self.sorted_timestamps
            ]
            y_vals = [
                self.bagfile_records_dicts[t].state.y for t in self.sorted_timestamps
            ]
            fig = go.Figure(data=go.Scatter(x=x_vals, y=y_vals, mode="lines+markers"))

        fig.update_layout(title="X vs Y Coordinates", xaxis_title="X", yaxis_title="Y")
        fig.show()

    def plot_xy_coordinates_and_centers(self):

        for window in self.windows:
            x_vals = [record.state.x for record in window.values()]
            y_vals = [record.state.y for record in window.values()]
            steering_cmds = [record.steering_cmd for record in window.values()]
            steering_fbks = [record.steering_fbk for record in window.values()]

            centers_from_utm, radiuses_from_utm = compute_radius_3_points(
                x_vals, y_vals
            )

            # Plot x, y coordinates and center of rotation for each point using Plotly
            fig = go.Figure()
            for i, (x, y, center, radius) in enumerate(
                zip(x_vals, y_vals, centers_from_utm, radiuses_from_utm)
            ):

                # # HACK for plotting purposes
                # if radius is None or not (0.5 <= radius <= 5):
                #     continue

                print(f"Point {i+1}: x={x}, y={y}, center={center}, radius={radius}")

                color = COLORS[i % len(COLORS)]
                # Plot the x, y coordinate
                if radius is None:
                    radius = 0
                fig.add_trace(
                    go.Scatter(
                        x=[x],
                        y=[y],
                        mode="markers",
                        marker=dict(color=color, size=8),
                        name=f"{i+1} r={radius:.2f}",
                        hoverinfo="name",
                    )
                )
                # Plot the center of rotation if available
                if center:
                    center_x, center_y = center
                    fig.add_trace(
                        go.Scatter(
                            x=[center_x],
                            y=[center_y],
                            mode="markers",
                            marker=dict(color=color, symbol="x", size=10),
                            name=f"Center {i+1}",
                        )
                    )

                    # Add thin line connecting point to its center
                    fig.add_trace(
                        go.Scatter(
                            x=[x, center_x],
                            y=[y, center_y],
                            mode="lines",
                            line=dict(color=color, width=1, dash="dot"),
                            showlegend=False,
                            hoverinfo="skip",
                        )
                    )

            fig.update_layout(
                title="X, Y Coordinates and Centers of Rotation",
                xaxis_title="X",
                yaxis_title="Y",
                showlegend=True,
            )
            fig.update_yaxes(scaleanchor="x", scaleratio=1)
            fig.show()

            fig = go.Figure()
            fig.add_trace(
                go.Scatter(
                    x=steering_cmds,
                    y=radiuses_from_utm,
                    mode="markers",
                )
            )
            fig.update_layout(
                title="Steering Cmd vs Radius",
                xaxis_title="Steering Cmd",
                yaxis_title="Radius",
            )
            fig.show()

            fig = go.Figure()
            fig.add_trace(
                go.Scatter(
                    x=steering_fbks,
                    y=radiuses_from_utm,
                    mode="markers",
                )
            )
            fig.update_layout(
                title="Steering Feedback vs Radius",
                xaxis_title="Steering Feedback",
                yaxis_title="Radius",
            )
            fig.show()

            # HACK return after the first window
            break
