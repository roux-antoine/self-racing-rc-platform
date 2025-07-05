import sys
from typing import Dict

import matplotlib.pyplot as plt
import ros_geometry_utils
import rosbag
from car_models import State

xs = []
ys = []
yaws = []
velocities = []
servo_cmds = []
fix_types = []
pose_timestamps = []
velocity_timestamps = []
arduino_timestamps = []
gps_timestamps = []


class BagfileRecord:
    def __init__(self, state: State, steering_cmd: float, fix_type: str) -> None:
        self.state = state
        self.steering_cmd = steering_cmd
        self.fix_type = fix_type


class BagfileSectionExtractor:
    def __init__(
        self, bag_path: str, min_window_size: int = 10, debug: bool = True
    ) -> None:
        self.bag_path = bag_path
        self.min_window_size = min_window_size
        self.debug = debug
        self.bagfile_records_dicts: Dict[float, BagfileRecord] = {}

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
                    fix_type=-1,
                )
                self.bagfile_records_dicts[t] = bagfile_record

        else:
            print("figure it out")
            sys.exit()

        sorted_timestamps = list(self.bagfile_records_dicts.keys())
        sorted_timestamps.sort()

        # next we align with the arduino messages.
        search_index = 0
        t_diff = []
        for t in sorted_timestamps:
            for i in range(search_index, len(arduino_timestamps) - 1):
                i += 1
                if arduino_timestamps[i] > t:
                    t_diff.append(arduino_timestamps[i] - t)
                    self.bagfile_records_dicts[t].steering_cmd = servo_cmds[i]
                    break
        if self.debug:
            plt.plot(t_diff)
            plt.show()

        # next we align the gps messages
        search_index = 0
        t_diff = []
        for t in sorted_timestamps:
            for i in range(search_index, len(gps_timestamps) - 1):
                i += 1
                if gps_timestamps[i] > t:
                    t_diff.append(gps_timestamps[i] - t)
                    self.bagfile_records_dicts[t].fix_type = fix_types[i]
                    break

        if self.debug:
            plt.plot(t_diff)
            plt.show()

        for key, value in self.bagfile_records_dicts.items():
            print(key, value)
            break

        # now we go over the datapoints and create windows where the fix type does not change, and is in {F, R}

        windows = []
        current_window = {}
        last_fix_type = None
        for t in sorted_timestamps:

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
                        windows.append(current_window)
                    current_window = {}
                else:
                    if current_fix_type != last_fix_type:
                        # we close the window
                        if len(current_window) >= self.min_window_size:
                            windows.append(current_window)
                        current_window = {}
                    else:
                        # we continue the window
                        current_window[t] = self.bagfile_records_dicts[t]
            last_fix_type = current_fix_type

        if len(current_window) >= self.min_window_size:
            windows.append(current_window)

        print(f"Found {len(windows)} windows: ")
        for window in windows:
            window_start = min(window.keys())
            window_end = max(window.keys())
            print(f"  {window_start} to {window_end} ({len(window)})")
