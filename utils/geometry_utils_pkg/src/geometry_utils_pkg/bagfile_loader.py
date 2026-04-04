import argparse
from typing import Dict, List, Optional

import matplotlib.pyplot as plt
import plotly.graph_objs as go
import rosbag
from geometry_utils_pkg.geometry_utils import State


try:
    # if we are on linux, we should have tf installed
    import tf

    TF_AVAILABLE = True
except ImportError:
    # if we are on mac, we don't have tf, so we use this custom package instead
    from geometry_utils_pkg import ros_geometry_utils

    TF_AVAILABLE = False


class BagfileRecord:
    def __init__(
        self,
        state: State,
        steering_cmd: float,
        fix_type: str,
        gps_msg_time: float,
        state_msg_time: float,
        arduino_msg_time: float,
        steering_fbk: Optional[float] = None,
        throttle_cmd: float = 0.0,
        target_curvature: Optional[float] = None,
        target_speed: Optional[float] = None,
    ) -> None:
        self.state: State = state
        self.steering_cmd: float = steering_cmd
        self.steering_fbk: Optional[float] = steering_fbk
        self.throttle_cmd: float = throttle_cmd
        self.fix_type: str = fix_type
        self.gps_msg_time: float = gps_msg_time
        self.state_msg_time: float = state_msg_time
        self.arduino_msg_time: float = arduino_msg_time
        self.target_curvature: Optional[float] = target_curvature
        self.target_speed: Optional[float] = target_speed


class BagfileLoader:
    def __init__(self, bag_path: str, debug: bool = False) -> None:
        self.bag_path: str = bag_path
        self.debug: bool = debug
        self.bagfile_records_dicts: Dict[float, BagfileRecord] = {}
        self.windows: List[Dict[float, BagfileRecord]] = []

        with rosbag.Bag(self.bag_path) as bag:

            xs = []
            ys = []
            yaws = []
            velocities = []
            servo_cmds = []
            steering_fbks = []
            throttle_cmds = []
            fix_types = []
            pose_timestamps = []
            velocity_timestamps = []
            arduino_timestamps = []
            gps_timestamps = []
            target_curvature_timestamps = []
            target_curvatures = []
            target_speed_timestamps = []
            target_speeds = []

            for topic, msg, t in bag.read_messages(
                topics=[
                    "/current_pose",
                    "/current_velocity",
                    "/arduino_logging",
                    "/gps_info",
                    "/target_curvature",
                    "/target_velocity",
                ]
            ):
                if topic == "/current_pose":
                    orientation_as_vec = [
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w,
                    ]
                    if TF_AVAILABLE:
                        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                            orientation_as_vec
                        )
                    else:
                        (
                            roll,
                            pitch,
                            yaw,
                        ) = ros_geometry_utils.euler_from_quaternion(  # pylint: disable=used-before-assignment
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
                    # steering_fbks.append(msg.steering_fbk)  # HACK 
                    throttle_cmds.append(msg.throttle_cmd_final)

                elif topic == "/gps_info":
                    gps_timestamps.append(t.to_sec())
                    fix_types.append(msg.mode_indicator)

                elif topic == "/target_curvature":
                    target_curvature_timestamps.append(t.to_sec())
                    target_curvatures.append(msg.data)

                elif topic == "/target_velocity":
                    target_speed_timestamps.append(t.to_sec())
                    target_speeds.append(msg.twist.linear.x)

        self._has_target_curvature = len(target_curvature_timestamps) > 0
        self._has_target_speed = len(target_speed_timestamps) > 0

        # Align all msgs to the GPS timestamps
        for gps_msg_time in gps_timestamps:

            # Find the closest pose timestamp
            closest_pose_time = [ts for ts in pose_timestamps if ts >= gps_msg_time][0]
            if (closest_pose_time - gps_msg_time) > 0.25:
                raise ValueError(
                    f"Pose and GPS timestamps are too far apart: {closest_pose_time - gps_msg_time}"
                )

            # Find the closest velocity timestamp
            closest_velocity_time = [
                ts for ts in velocity_timestamps if ts >= gps_msg_time
            ][0]
            if (closest_velocity_time - gps_msg_time) > 0.25:
                raise ValueError(
                    f"Velocity and GPS timestamps are too far apart: {closest_velocity_time - gps_msg_time}"
                )

            # Find the closest arduino timestamp
            closest_arduino_time = [
                ts for ts in arduino_timestamps if ts >= gps_msg_time
            ][0]
            if (closest_arduino_time - gps_msg_time) > 0.25:
                raise ValueError(
                    f"Arduino and GPS timestamps are too far apart: {closest_arduino_time - gps_msg_time}"
                )

            state = State(
                x=xs[pose_timestamps.index(closest_pose_time)],
                y=ys[pose_timestamps.index(closest_pose_time)],
                vx=velocities[velocity_timestamps.index(closest_velocity_time)],
                angle=yaws[pose_timestamps.index(closest_pose_time)],
            )

            arduino_idx = arduino_timestamps.index(closest_arduino_time)

            # Align optional topics (target_curvature, target_velocity)
            aligned_target_curvature = None
            if self._has_target_curvature:
                closest_tc = [ts for ts in target_curvature_timestamps if ts >= gps_msg_time]
                if closest_tc:
                    if (closest_tc[0] - gps_msg_time) > 0.25:
                        raise ValueError(
                            f"Target curvature and GPS timestamps are too far apart: {closest_tc[0] - gps_msg_time}"
                        )
                    aligned_target_curvature = target_curvatures[
                        target_curvature_timestamps.index(closest_tc[0])
                    ]

            aligned_target_speed = None
            if self._has_target_speed:
                closest_tv = [ts for ts in target_speed_timestamps if ts >= gps_msg_time]
                if closest_tv:
                    if (closest_tv[0] - gps_msg_time) > 0.25:
                        raise ValueError(
                            f"Target velocity and GPS timestamps are too far apart: {closest_tv[0] - gps_msg_time}"
                        )
                    aligned_target_speed = target_speeds[
                        target_speed_timestamps.index(closest_tv[0])
                    ]

            # Create a new BagfileRecord with the aligned timestamps
            bagfile_record = BagfileRecord(
                state=state,
                steering_cmd=servo_cmds[arduino_idx],
                # steering_fbk=steering_fbks[arduino_idx],
                throttle_cmd=throttle_cmds[arduino_idx],
                fix_type=fix_types[gps_timestamps.index(gps_msg_time)],
                gps_msg_time=gps_msg_time,
                state_msg_time=closest_pose_time,
                arduino_msg_time=closest_arduino_time,
                target_curvature=aligned_target_curvature,
                target_speed=aligned_target_speed,
            )
            self.bagfile_records_dicts[gps_msg_time] = bagfile_record

        if self.debug:
            for t, record in self.bagfile_records_dicts.items():
                plt.scatter(
                    t,
                    record.state_msg_time - t,
                    color="blue",
                    label="State msg time diff",
                )
                plt.scatter(
                    t,
                    record.arduino_msg_time - t,
                    color="red",
                    label="Arduino msg time diff",
                )
            plt.xlabel("GPS Message Time (s)")
            plt.ylabel("Time Difference (s)")
            plt.title(
                "Time Difference between GPS and State/Arduino Message Timestamps"
            )
            plt.show()

    def plot_xy_coordinates_plotly(self, use_windows: bool = False) -> None:
        """Plot the X and Y coordinates from the bagfile records using Plotly.

        Args:
            - use_windows (bool): If True, plot only the extracted windows. If False, plot all data.
        """
        if use_windows and not self.windows:
            raise ValueError(
                "No windows extracted. Please run extract_windows() first."
            )
        if use_windows:
            fig = go.Figure()
            for window in self.windows:
                x_vals = [bagfile_record.state.x for bagfile_record in window.values()]
                y_vals = [bagfile_record.state.y for bagfile_record in window.values()]
                fig.add_trace(go.Scatter(x=x_vals, y=y_vals, mode="lines+markers"))
            fig.show()
        else:
            x_vals = [
                bagfile_record.state.x
                for bagfile_record in self.bagfile_records_dicts.values()
            ]
            y_vals = [
                bagfile_record.state.y
                for bagfile_record in self.bagfile_records_dicts.values()
            ]
            fig = go.Figure(data=go.Scatter(x=x_vals, y=y_vals, mode="lines+markers"))
            fig.show()

    def extract_windows(
        self,
        min_window_size: int = 10,
        tolerance: int = 0,
        allowed_fix_types: list = ["F", "R"],
    ) -> None:
        """Find portions of the data where the GPS fix type is constant and valid (F or R).

        Args:
            - min_window_size (int): Minimum number of consecutive records with the same valid fix type to consider it a window.
            - tolerance (int): Number of consecutive invalid fix types allowed within a window before closing it.
            - allowed_fix_types (list): List of fix types (e.g. Differential, Float, Rtk) considered valid for window extraction.
        """
        current_window = {}
        last_fix_type = None
        tolerance_counter = 0
        for t, bagfile_record in self.bagfile_records_dicts.items():

            current_fix_type = bagfile_record.fix_type

            if not last_fix_type:
                if current_fix_type in allowed_fix_types:
                    last_fix_type = current_fix_type
                else:
                    continue
            else:
                if current_fix_type not in allowed_fix_types:
                    if tolerance_counter < tolerance:
                        tolerance_counter += 1
                    else:
                        # we close the window
                        if len(current_window) >= min_window_size:
                            self.windows.append(current_window)
                        current_window = {}
                        tolerance_counter = 0
                else:
                    if current_fix_type != last_fix_type:
                        if tolerance_counter < tolerance:
                            tolerance_counter += 1
                        else:
                            # we close the window
                            if len(current_window) >= min_window_size:
                                self.windows.append(current_window)
                            current_window = {}
                            tolerance_counter = 0
                    else:
                        # we continue the window
                        current_window[t] = bagfile_record
            last_fix_type = current_fix_type

        if len(current_window) >= min_window_size:
            self.windows.append(current_window)

        print(f"Found {len(self.windows)} windows: ")
        for window in self.windows:
            window_start = min(window.keys())
            window_end = max(window.keys())
            print(f"  {window_start} to {window_end} ({len(window)})")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load and process a ROS bag file.")
    parser.add_argument("--bag-path", type=str, help="Path to the ROS bag file")
    parser.add_argument("--debug", action="store_true", help="Enable debug plots")
    args = parser.parse_args()

    loader = BagfileLoader(args.bag_path, debug=args.debug)

    loader.extract_windows(min_window_size=10)

    loader.plot_xy_coordinates_plotly(use_windows=True)
