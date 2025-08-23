from typing import Dict
import argparse

import matplotlib.pyplot as plt
import rosbag
from geometry_utils import State


try:
    # if we are on linux, we should have tf installed
    import tf

    TF_AVAILABLE = True
except ImportError:
    # if we are on mac, we don't have tf, so we use this custom package instead
    import ros_geometry_utils

    TF_AVAILABLE = False


class BagfileRecord:
    def __init__(
        self,
        state: State,
        steering_cmd: float,
        steering_fbk: float,
        fix_type: str,
        gps_msg_time: float,
        state_msg_time: float,
        arduino_msg_time: float,
    ) -> None:
        self.state: State = state
        self.steering_cmd: float = steering_cmd
        self.steering_fbk: float = steering_fbk
        self.fix_type: str = fix_type
        self.gps_msg_time: float = gps_msg_time
        self.state_msg_time: float = state_msg_time
        self.arduino_msg_time: float = arduino_msg_time


class BagfileLoader:
    def __init__(self, bag_path: str, debug: bool = True) -> None:
        self.bag_path: str = bag_path
        self.debug: bool = debug
        self.bagfile_records_dicts: Dict[float, BagfileRecord] = {}

        with rosbag.Bag(self.bag_path) as bag:

            xs = []
            ys = []
            yaws = []
            velocities = []
            servo_cmds = []
            steering_fbks = []
            fix_types = []
            pose_timestamps = []
            velocity_timestamps = []
            arduino_timestamps = []
            gps_timestamps = []

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
                    if TF_AVAILABLE:
                        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                            orientation_as_vec
                        )
                    else:
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
                    steering_fbks.append(msg.steering_fbk)

                elif topic == "/gps_info":
                    gps_timestamps.append(t.to_sec())
                    fix_types.append(msg.mode_indicator)

        # Maybe we should use the GPS time as the reference time?

        # Align all msgs to the GPS timestamps
        for gps_msg_time in gps_timestamps:

            # Find the closest pose timestamp
            closest_pose_time = [ts for ts in pose_timestamps if ts >= gps_msg_time][0]
            if (closest_pose_time - gps_msg_time) > 0.2:
                raise ValueError(
                    f"Pose and GPS timestamps are too far apart: {closest_pose_time - gps_msg_time}"
                )

            # Find the closest velocity timestamp
            closest_velocity_time = [
                ts for ts in velocity_timestamps if ts >= gps_msg_time
            ][0]
            if (closest_velocity_time - gps_msg_time) > 0.2:
                raise ValueError(
                    f"Velocity and GPS timestamps are too far apart: {closest_velocity_time - gps_msg_time}"
                )

            # Find the closest arduino timestamp
            closest_arduino_time = [
                ts for ts in arduino_timestamps if ts >= gps_msg_time
            ][0]
            if (closest_arduino_time - gps_msg_time) > 0.2:
                raise ValueError(
                    f"Arduino and GPS timestamps are too far apart: {closest_arduino_time - gps_msg_time}"
                )

            state = State(
                x=xs[pose_timestamps.index(closest_pose_time)],
                y=ys[pose_timestamps.index(closest_pose_time)],
                vx=velocities[velocity_timestamps.index(closest_velocity_time)],
                angle=yaws[pose_timestamps.index(closest_pose_time)],
            )

            # Create a new BagfileRecord with the aligned timestamps
            bagfile_record = BagfileRecord(
                state=state,
                steering_cmd=servo_cmds[arduino_timestamps.index(closest_arduino_time)],
                steering_fbk=steering_fbks[
                    arduino_timestamps.index(closest_arduino_time)
                ],
                fix_type=fix_types[gps_timestamps.index(gps_msg_time)],
                gps_msg_time=gps_msg_time,
                state_msg_time=closest_pose_time,
                arduino_msg_time=closest_arduino_time,
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load and process a ROS bag file.")
    parser.add_argument("--bag-path", type=str, help="Path to the ROS bag file")
    parser.add_argument("--debug", action="store_true", help="Enable debug plots")
    args = parser.parse_args()

    loader = BagfileLoader(args.bag_path, debug=args.debug)
