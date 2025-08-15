#!/usr/bin/python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math
import utm

"""
Script to tryout different models for the future pose prediction to be used in the vehicle_state_publisher
"""

"""
radius(t) = f[steering_diff(t), steering_diff(t-1), speed(t)]
Idea: use the linear-by-part steering model, compute error on random drives. Do it with a steering angle that is not the last steering angle, but a linear combination of the last 2. And find if there's a minimum.
"""

DELTA_TIME = (
    0.1  # don't change it for validation purposes, since we get a position every 100ms
)

PLOT_DEBUG = False

bag = rosbag.Bag(
    # "/app/circuit_5m_104pwm_2023-11-11-23-32-16.bag"
    "/Users/antoineroux/Downloads/self-racing-rc-platform-2/circuit_5m_104pwm_2023-11-11-23-32-16.bag"
)

topics = ["/gps_info", "/arduino_logging"]

current_poses = []
future_poses_v1 = []
future_poses_v2 = []
future_poses_v3 = []
last_engaged = False

fig, ax = plt.subplots()
last_steering_cmd = None
last_minus_one_steering_cmd = None
last_minus_two_steering_cmd = None
for topic, msg, t in bag.read_messages(topics=topics):
    if topic == "/arduino_logging":
        last_minus_two_steering_cmd = last_minus_one_steering_cmd
        last_minus_one_steering_cmd = last_steering_cmd
        last_steering_cmd = msg.steering_cmd_final

        if msg.engaged_mode == True and msg.override_throttle == False:
            last_engaged = True
        else:
            last_engaged = False

    if topic == "/gps_info":
        if last_steering_cmd is not None and last_minus_two_steering_cmd is not None:
            speed_mps = msg.speed * 0.5144
            yaw_rad = (math.pi / 2) - msg.track * math.pi / 180

            if speed_mps < 3:
                continue

            utm_values = utm.from_latlon(msg.lat, msg.lon)
            x_utm, y_utm = utm_values[0], utm_values[1]

            current_pose = [x_utm, y_utm, yaw_rad, speed_mps]
            current_poses.append(current_pose)

            # V1
            # very simple model: future position is the current one + the velocity vector
            future_pose_x = x_utm + speed_mps * math.cos(yaw_rad) * DELTA_TIME
            future_pose_y = y_utm + speed_mps * math.sin(yaw_rad) * DELTA_TIME
            future_poses_v1.append([future_pose_x, future_pose_y, yaw_rad])

            # V2
            # model that accounts for the steering command
            steering_diff = last_steering_cmd - 98
            if steering_diff != 0:
                # in theory, this should use the same formula as in the lateral controller
                # but experimental results are better with this value
                # TODO investigate and adapt if needed
                R = 15 / steering_diff
            else:
                R = 10000  # big number

            # computing the x and y coordinates of the future pose
            theta = speed_mps / abs(R) * DELTA_TIME
            phi = yaw_rad
            alpha = phi - np.pi / 2

            if R > 0:
                theta = -theta  # TODO wtf
                
            future_pose_x = x_utm + np.sign(R) * abs(R) * (np.cos(alpha) - np.cos(alpha + theta))
            future_pose_y = y_utm + np.sign(R) * abs(R) * (np.sin(alpha) - np.sin(alpha + theta))
            angle = theta + alpha + np.pi / 2


            future_poses_v2.append([future_pose_x, future_pose_y, angle])

            # last_steering_cmd = None  # QUESTION why was this here?
            if PLOT_DEBUG:
                # if np.abs(R) > 5: # uncomment to help debug
                #     continue
                # if int(t.to_sec()) != int(1699745545.1612222) and int(t.to_sec()) != int(1699745546.1593683):
                #     continue
                if int(t.to_sec()) != int(1699745545.1612222):
                    continue
                plt.scatter(x_utm, y_utm, color="green")
                plt.scatter(future_pose_x, future_pose_y, color="red")
                plt.scatter(
                    x_utm + np.sign(R) * abs(R) * np.cos(yaw_rad - np.pi / 2),
                    y_utm + np.sign(R) * abs(R) * np.sin(yaw_rad - np.pi / 2),
                    color = "black"
                )
                plt.arrow(x_utm, y_utm, np.cos(yaw_rad), np.sin(yaw_rad), color="green")
                plt.arrow(future_pose_x, future_pose_y, np.cos(angle), np.sin(angle), color="red")
                # plt.plot(
                #     [x_utm, x_utm + np.sign(R) * abs(R) * np.cos(yaw_rad - np.pi / 2)],
                #     [y_utm, y_utm + np.sign(R) *abs(R) * np.sin(yaw_rad - np.pi / 2)],
                # )
                plt.plot([x_utm, future_pose_x], [y_utm, future_pose_y], color="gray")
                circle1 = plt.Circle(
                    (
                        x_utm + np.sign(R) * abs(R) * np.cos(yaw_rad - np.pi / 2),
                        y_utm + np.sign(R) * abs(R) * np.sin(yaw_rad - np.pi / 2),
                    ),
                    R,
                    fill=False,
                )
               
                ax.add_patch(circle1)
                plt.text(x=x_utm + 1, y=y_utm + 1, s=t.to_sec())

            # V3
            # model that acounts for the steering command, but using the one before the last one
            steering_diff = last_minus_one_steering_cmd - 98
            if steering_diff != 0:
                # in theory, this should use the same formula as in the lateral controller
                # but experimental results are better with this value
                # TODO investigate and adapt if needed
                R = 15 / steering_diff
            else:
                R = 10000  # big number

            # computing the x and y coordinates of the future pose
            theta = speed_mps / abs(R) * DELTA_TIME
            phi = yaw_rad
            alpha = phi - np.pi / 2

            if R > 0:
                theta = -theta  # TODO wtf
                
            future_pose_x = x_utm + np.sign(R) * abs(R) * (np.cos(alpha) - np.cos(alpha + theta))
            future_pose_y = y_utm + np.sign(R) * abs(R) * (np.sin(alpha) - np.sin(alpha + theta))
            angle = theta + alpha + np.pi / 2

            future_poses_v3.append([future_pose_x, future_pose_y, angle])

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

print(len(current_poses))

errors_xy_v1 = []
errors_angle_v1 = []
errors_xy_v2 = []
errors_angle_v2 = []
errors_xy_v3 = []
errors_angle_v3 = []

fig = plt.Figure()

for i in range(len(current_poses) - 1):
    plt.scatter(current_poses[i][0], current_poses[i][1], c="green")
    plt.arrow(
        current_poses[i][0],
        current_poses[i][1],
        0.1 * np.cos(current_poses[i][2]),
        0.1 * np.sin(current_poses[i][2]),
        color="green",
    )
    # plt.scatter(future_poses_v1[i][0], future_poses_v1[i][1], c="red")
    # plt.arrow(
    #     future_poses_v1[i][0],
    #     future_poses_v1[i][1],
    #     0.1 * np.cos(future_poses_v1[i][2]),
    #     0.1 * np.sin(future_poses_v1[i][2]),
    #     color="blue",
    # )
    plt.scatter(future_poses_v2[i][0], future_poses_v2[i][1], c="orange")
    plt.arrow(
        future_poses_v2[i][0],
        future_poses_v2[i][1],
        0.1 * np.cos(future_poses_v2[i][2]),
        0.1 * np.sin(future_poses_v2[i][2]),
        color="orange",
    )

    plt.plot([current_poses[i+1][0], future_poses_v2[i][0]],
             [current_poses[i+1][1], future_poses_v2[i][1]], color="gray"
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
    error_xy_v3 = np.linalg.norm(
        [
            future_poses_v3[i][0] - current_poses[i + 1][0],
            future_poses_v3[i][1] - current_poses[i + 1][1],
        ]
    )
    errors_xy_v3.append(error_xy_v3)
    error_angle_v3 = np.abs(current_poses[i + 1][2] - future_poses_v3[i][2]) % 2 * np.pi
    errors_angle_v3.append(error_angle_v3)

plt.axis("equal")
plt.show()

print(f"errors_xy_v1: {np.mean(errors_xy_v1)}")
print(f"errors_xy_v2: {np.mean(errors_xy_v2)}")
print(f"errors_xy_v3: {np.mean(errors_xy_v3)}")

print(f"errors_angle_v1: {np.mean(errors_angle_v1)}")
print(f"errors_angle_v2: {np.mean(errors_angle_v2)}")
print(f"errors_angle_v3: {np.mean(errors_angle_v3)}")

plt.hist(errors_xy_v1, bins=50, alpha=0.5, label="errors_xy_v1")
plt.hist(errors_xy_v2, bins=50, alpha=0.5, label="errors_xy_v2")
plt.hist(errors_xy_v3, bins=50, alpha=0.5, label="errors_xy_v3")
plt.legend()
plt.show()

plt.hist(errors_angle_v1, bins=50, alpha=0.5, label="errors_angle_v1")
plt.hist(errors_angle_v2, bins=50, alpha=0.5, label="errors_angle_v2")
plt.hist(errors_angle_v3, bins=50, alpha=0.5, label="errors_angle_v3")
plt.legend()
plt.show()


# NOTE
# there is no point trying to predict the heading until we use the "bad" heading from the gps
# we need a better way of determining the heading of the vehicle
# in fact it could even make sense of trying the current V2 algo, maybe the angle is correct enough
# the fact that the heading from the gps is so incorrect might also negtively impact the pure pursuit calculations (e.g. the circle we compute is far from the real circle)
# end NOTE

# for elem, b in zip(current_poses[:-1], errors_angle_v2):
#     if b > 1:
#         plt.scatter(elem[0], elem[1])
#     # plt.scatter(elem[3], b)
# plt.show()
