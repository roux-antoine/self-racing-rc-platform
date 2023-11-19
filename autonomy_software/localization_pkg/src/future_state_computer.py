#!/usr/bin/python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math
import utm

"""
Script to tryout different models for the future pose prediction to be used in the vehicle_state_publisher
"""

DELTA_TIME = (
    0.1  # don't change it for validation purposes, since we get a position every 100ms
)

PLOT_DEBUG = False

bag = rosbag.Bag(
    "/Users/antoineroux/Downloads/circuit_5m_104pwm_2023-11-11-23-32-16.bag"
)

topics = ["/gps_info", "/arduino_logging"]

current_poses = []
future_poses_v1 = []
future_poses_v2 = []


fig, ax = plt.subplots()
last_steering_cmd = None
for topic, msg, t in bag.read_messages(topics=topics):
    if topic == "/arduino_logging":
        last_steering_cmd = msg.steering_cmd_final
    if topic == "/gps_info":
        if last_steering_cmd is not None:
            speed_mps = msg.speed * 0.5144
            yaw_rad = (math.pi / 2) - msg.track * math.pi / 180

            utm_values = utm.from_latlon(msg.lat, msg.lon)
            x_utm, y_utm = utm_values[0], utm_values[1]

            current_pose = [x_utm, y_utm, yaw_rad]
            current_poses.append(current_pose)

            # V1
            # very simple model: future position is the current one + the velocity vector
            future_pose_x = x_utm + speed_mps * math.cos(yaw_rad) * DELTA_TIME
            future_pose_y = y_utm + speed_mps * math.sin(yaw_rad) * DELTA_TIME
            future_pose = [future_pose_x, future_pose_y, yaw_rad]
            future_poses_v1.append(future_pose)

            # V2 # TODO do the orentation
            # model that accounds for the steering command
            steering_diff = last_steering_cmd - 98
            if steering_diff != 0:
                # in theory, this shoudl use the same formula as in the lateral controller
                # but experimental results are better with this value
                # TODO investigate and adapt if needed
                R = 15 / steering_diff
            else:
                R = 10000  # big number

            # computing the x and y coordinates of the future pose
            # TODO TBH I started from a formula derived by hand, and tweaked it until it looked correct. Re-do the derivation
            future_pose_x = (
                x_utm
                + np.cos(yaw_rad + np.pi / 2)
                * R
                * (np.cos(speed_mps / -R * DELTA_TIME) - 1)
                - np.sin(yaw_rad + np.pi / 2) * R * np.sin(speed_mps / -R * DELTA_TIME)
            )
            future_pose_y = (
                y_utm
                + np.sin(yaw_rad + np.pi / 2)
                * R
                * (np.cos(speed_mps / -R * DELTA_TIME) - 1)
                + np.cos(yaw_rad + np.pi / 2) * R * np.sin(speed_mps / -R * DELTA_TIME)
            )

            # computing the angle of the future pose # TODO does not work
            # vec_between_center_and_future_pose = [future_pose_x - (x_utm - R * np.cos(yaw_rad+np.pi/2)),
            #                                       future_pose_y - (y_utm - R * np.sin(yaw_rad+np.pi/2))]

            # if np.sign(R) == -1: # hack makes it work more but not fully
            #     normal_vector_to_vec_between_center_and_future_pose = [-vec_between_center_and_future_pose[1], vec_between_center_and_future_pose[0]]
            # else:
            #     normal_vector_to_vec_between_center_and_future_pose = [vec_between_center_and_future_pose[1], -vec_between_center_and_future_pose[0]]
            # normalized_normal_vector_to_vec_between_center_and_future_pose = np.array(normal_vector_to_vec_between_center_and_future_pose) / np.linalg.norm(normal_vector_to_vec_between_center_and_future_pose)

            # angle = np.arccos(np.dot(normalized_normal_vector_to_vec_between_center_and_future_pose, [1, 0]))
            angle = yaw_rad

            future_pose = [future_pose_x, future_pose_y, angle]
            future_poses_v2.append(future_pose)
            last_steering_cmd = None
            if PLOT_DEBUG:
                # if np.abs(R) > 5: # uncomment to help debug
                #     continue
                # if int(t.to_sec()) != int(1699745545.1612222):
                #     continue
                # if t.to_sec() != 1699745546.1593683:
                #     continue
                plt.scatter(future_pose_x, future_pose_y)
                plt.scatter(x_utm, y_utm, color="green")
                plt.scatter(
                    x_utm - R * np.cos(yaw_rad + np.pi / 2),
                    y_utm - R * np.sin(yaw_rad + np.pi / 2),
                )
                plt.arrow(x_utm, y_utm, np.cos(yaw_rad), np.sin(yaw_rad), color="blue")
                plt.plot(
                    [x_utm, x_utm - R * np.cos(yaw_rad + np.pi / 2)],
                    [y_utm, y_utm - R * np.sin(yaw_rad + np.pi / 2)],
                )
                circle1 = plt.Circle(
                    (
                        x_utm - R * np.cos(yaw_rad + np.pi / 2),
                        y_utm - R * np.sin(yaw_rad + np.pi / 2),
                    ),
                    R,
                    fill=False,
                )
                ax.add_patch(circle1)
                plt.text(x=x_utm + 1, y=y_utm + 1, s=t.to_sec())
                plt.arrow(
                    future_pose_x,
                    future_pose_y,
                    np.cos(angle),
                    np.sin(angle),
                    color="blue",
                )
                # plt.plot([future_pose_x, future_pose_x + normalized_normal_vector_to_vec_between_center_and_future_pose[0]], [future_pose_y, future_pose_y + normalized_normal_vector_to_vec_between_center_and_future_pose[1]])
                # plt.plot([future_pose_x, future_pose_x - vec_between_center_and_future_pose[0]], [future_pose_y, future_pose_y - vec_between_center_and_future_pose[1]])

if PLOT_DEBUG:
    plt.axis("equal")
    plt.show()


# just plotting the real positions to figure out if the track angle is weird

for current_pose in current_poses:
    plt.scatter(current_pose[0], current_pose[1], c="green")
    plt.arrow(
        current_pose[0],
        current_pose[1],
        0.1 * np.cos(current_pose[2]),
        0.1 * np.sin(current_pose[2]),
        color="blue",
    )
plt.axis("equal")
plt.show()

# plotting the predicted collisions and computing metrics

errors_xy_v1 = []
errors_angle_v1 = []
errors_xy_v2 = []
errors_angle_v2 = []

fig = plt.Figure()

for i in range(len(current_poses) - 1):
    plt.scatter(current_poses[i][0], current_poses[i][1], c="green")
    plt.arrow(
        current_poses[i][0],
        current_poses[i][1],
        0.1 * np.cos(current_poses[i][2]),
        0.1 * np.sin(current_poses[i][2]),
        color="blue",
    )
    plt.scatter(future_poses_v1[i][0], future_poses_v1[i][1], c="red")
    plt.arrow(
        future_poses_v1[i][0],
        future_poses_v1[i][1],
        0.1 * np.cos(future_poses_v1[i][2]),
        0.1 * np.sin(future_poses_v1[i][2]),
        color="blue",
    )
    plt.scatter(future_poses_v2[i][0], future_poses_v2[i][1], c="orange")
    plt.arrow(
        future_poses_v2[i][0],
        future_poses_v2[i][1],
        0.1 * np.cos(future_poses_v2[i][2]),
        0.1 * np.sin(future_poses_v2[i][2]),
        color="blue",
    )

    error_xy_v1 = np.linalg.norm(
        [
            future_poses_v1[i][0] - current_poses[i + 1][0],
            future_poses_v1[i][1] - current_poses[i + 1][1],
        ]
    )
    errors_xy_v1.append(error_xy_v1)
    error_angle_v1 = np.abs(current_poses[i + 1][2] - future_poses_v1[i][2]) % 2 * np.pi
    errors_angle_v1.append(error_angle_v1)
    error_xy_v2 = np.linalg.norm(
        [
            future_poses_v2[i][0] - current_poses[i + 1][0],
            future_poses_v2[i][1] - current_poses[i + 1][1],
        ]
    )
    errors_xy_v2.append(error_xy_v2)
    error_angle_v2 = np.abs(current_poses[i + 1][2] - future_poses_v2[i][2]) % 2 * np.pi
    errors_angle_v2.append(error_angle_v2)
plt.axis("equal")
plt.show()

print(f"errors_xy_v1: {np.mean(errors_xy_v1)}")
print(f"errors_xy_v2: {np.mean(errors_xy_v2)}")


print(f"errors_angle_v1: {np.mean(errors_angle_v1)}")
print(f"errors_angle_v2: {np.mean(errors_angle_v2)}")

plt.hist(errors_xy_v1, bins=50, alpha=0.5, label="errors_xy_v1")
plt.hist(errors_xy_v2, bins=50, alpha=0.5, label="errors_xy_v2")
plt.legend()
plt.show()

plt.hist(errors_angle_v1, bins=50, alpha=0.5, label="errors_angle_v1")
plt.hist(errors_angle_v2, bins=50, alpha=0.5, label="errors_angle_v2")
plt.legend()
plt.show()
