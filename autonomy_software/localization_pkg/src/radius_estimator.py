
import rosbag
import utm
import matplotlib.pyplot as plt
import numpy as np
import math
import sys

"""
Script to tryout different models for the future pose prediction to be used in the vehicle_state_publisher
"""

"""
radius(t) = f[steering_diff(t), steering_diff(t-1), speed(t)]
Or even better:
delta_x(t+1) = f_x[x(t), y(t), theta(t), steering(t), speed(t)]
delta_y(t+1) = f_y[x(t), y(t), theta(t), steering(t), speed(t)]
Idea: use the linear-by-part steering model, compute error on random drives. Do it with a steering angle that is not the last steering angle, but a linear combination of the last 2. And find if there's a minimum.


steering_diff = curvature / coeff
<=> curvature = steering_diff * coeff
<=> radius = 1 / (steering_diff * coeff)
<=> coeff = curvature / steering_diff

How to estimate the curvature? Without vehicle geometry? Maybe with previous and next position?
There is only 1 circle going through 3 points. We say that it's the circle. Repeat the work from last time, but with a lot more points.
"""


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
    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det
    
    radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
    return ((cx, cy), radius)


UPPER_BOUND_REGION_1 = 1.5  # m/s
UPPER_BOUND_REGION_2 = 5  # m/s
UPPER_BOUND_REGION_3 = 8  # m/s
COEFF_REGION_1 = (
    27 * 1.25
)  # max steering_diff * radius of circle at max lateral acceleration
COEFF_REGION_2 = 24 * 2.3
COEFF_REGION_3 = 26 * 4

PLOT_DEBUG = False



topics = ["/gps_info", "/arduino_logging"]

current_poses = []

bag = rosbag.Bag(
    # "/app/circuit_5m_104pwm_2023-11-11-23-32-16.bag"
    "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.8_fast_2024-05-12-23-20-19.bag"
)

current_steering_cmd = None
last_minus_one_steering_cmd = None
last_minus_two_steering_cmd = None

current_pose = None
last_minus_one_pose = None
last_minus_two_pose = None


angles = []

PLOT_ALL = True
PLOT_SINGLE = not PLOT_ALL
if PLOT_ALL and PLOT_SINGLE:
    print("Not both at the same time")
    sys.exit()

if PLOT_ALL:
    fig, ax = plt.subplots()


for topic, msg, t in bag.read_messages(topics=topics):
    # if topic == "/arduino_logging":
    #     last_minus_two_steering_cmd = last_minus_one_steering_cmd
    #     last_minus_one_steering_cmd = current_steering_cmd
    #     current_steering_cmd = msg.steering_cmd_final

    #     if msg.engaged_mode == True and msg.override_throttle == False:
    #         last_engaged = True
    #     else:
    #         last_engaged = False

    if topic == "/gps_info":
        if True:  # current_steering_cmd is not None and last_minus_two_steering_cmd is not None:
        # if current_steering_cmd is not None and last_minus_two_steering_cmd is not None:
            speed_mps = msg.speed * 0.5144
            yaw_rad = (math.pi / 2) - msg.track * math.pi / 180

            utm_values = utm.from_latlon(msg.lat, msg.lon)
            x_utm, y_utm = utm_values[0], utm_values[1]

            last_minus_two_pose = last_minus_one_pose
            last_minus_one_pose = current_pose
            current_pose = [x_utm, y_utm, yaw_rad, speed_mps]

            # last_minus_one_steering_diff = last_minus_one_steering_cmd - 98
            if last_minus_one_pose == None or last_minus_two_pose == None:
                continue
            last_minus_one_yaw = last_minus_one_pose[2]
            last_minus_one_speed = last_minus_one_pose[3]

            if speed_mps < 1:
                continue

            # METHOD 1: computing the center of rotation using 3 points
            
            center_3_points, radius_3_points = define_circle(last_minus_two_pose, last_minus_one_pose, current_pose)

            # METHOD 2: computing the center of rotation using the COG and fixed radius
            RADIUS = 2.4

            last_minus_one_yaw += 0.0

            center_fixed_radius = (
                last_minus_one_pose[0] + 1 * RADIUS * np.cos(last_minus_one_yaw - np.pi / 2),
                last_minus_one_pose[1] + 1 * RADIUS * np.sin(last_minus_one_yaw - np.pi / 2),
            )
            # center_fixed_radius = (
            #     last_minus_one_pose[0] + np.sign(last_minus_one_steering_diff) * RADIUS * np.cos(last_minus_one_yaw - np.pi / 2),
            #     last_minus_one_pose[1] + np.sign(last_minus_one_steering_diff) * RADIUS * np.sin(last_minus_one_yaw - np.pi / 2),
            # )

            # METHOD 3: computing the center of rotation using our steering model
            
            # if last_minus_one_speed == 0:
            #     coeff = 1000  # just so that we get a small number later on
            # elif (
            #     last_minus_one_speed > 0
            #     and last_minus_one_speed <= UPPER_BOUND_REGION_1
            # ):
            #     coeff = 1 / (COEFF_REGION_1)
            # elif (
            #     last_minus_one_speed > UPPER_BOUND_REGION_1
            #     and last_minus_one_speed <= UPPER_BOUND_REGION_2
            # ):
            #     coeff = 1 / (
            #         COEFF_REGION_1
            #         + (last_minus_one_speed - UPPER_BOUND_REGION_1)
            #         * (COEFF_REGION_2 - COEFF_REGION_1)
            #         / (UPPER_BOUND_REGION_2 - UPPER_BOUND_REGION_1)
            #     )
            # elif (
            #     last_minus_one_speed > UPPER_BOUND_REGION_2
            #     and last_minus_one_speed <= UPPER_BOUND_REGION_3
            # ):
            #     coeff = 1 / (
            #         COEFF_REGION_2
            #         + (last_minus_one_speed - UPPER_BOUND_REGION_2)
            #         * (COEFF_REGION_3 - COEFF_REGION_2)
            #         / (UPPER_BOUND_REGION_3 - UPPER_BOUND_REGION_2)
            #     )
            # elif last_minus_one_speed > UPPER_BOUND_REGION_3:
            #     coeff = 1 / COEFF_REGION_3

            # radius_model = 1 / (last_minus_one_steering_diff * coeff)
            # center_model = (
            #         last_minus_one_pose[0] + np.sign(radius_model) * abs(radius_model) * np.cos(last_minus_one_yaw - np.pi / 2),
            #         last_minus_one_pose[1] + np.sign(radius_model) * abs(radius_model) * np.sin(last_minus_one_yaw - np.pi / 2),
            #     )

            if center_3_points: # and abs(radius_model) < 10 and abs(radius_3_points) < 10:
            # if center_3_points and abs(radius_model) < 10 and abs(radius_3_points) < 10:
                if PLOT_SINGLE:
                    fig, ax = plt.subplots()

                    plt.scatter(last_minus_two_pose[0], last_minus_two_pose[1])
                    plt.scatter(last_minus_one_pose[0], last_minus_one_pose[1])
                    plt.scatter(current_pose[0], current_pose[1])

                    # plt.scatter(center_3_points[0], center_3_points[1], color="black")
                    # plt.scatter(center_fixed_radius[0], center_fixed_radius[1], color="pink")
                    # plt.scatter(center_model[0], center_model[1], color="red")
                    # plt.arrow(last_minus_one_pose[0], last_minus_one_pose[1], np.sign(last_minus_one_steering_diff) * 10 * np.sin(last_minus_one_yaw), np.sign(last_minus_one_steering_diff) * 10 * -np.cos(last_minus_one_yaw), color="green")
                    
                    circle_3_points = plt.Circle(
                        center_3_points,
                        radius_3_points,
                        fill=False,
                    )
                    ax.add_patch(circle_3_points)
                    
                    plt.axis("equal")
                    plt.show()
                if PLOT_ALL:
                    plt.scatter(last_minus_one_pose[0], last_minus_one_pose[1])
                    # plt.scatter(center_3_points[0], center_3_points[1], color="k")
                    plt.scatter(center_fixed_radius[0], center_fixed_radius[1], color="pink")

                    # plt.scatter(center_model[0], center_model[1], color="red")

                # v1 = [center_3_points[0] - last_minus_one_pose[0], center_3_points[1] - last_minus_one_pose[1]]
                # v2 = [center_model[0] - last_minus_one_pose[0], center_model[1] - last_minus_one_pose[1]]
                # angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))) * 180 / 3.14
                # if angle > 90:
                #     angle = angle - 180
                # angles.append(angle)

if PLOT_ALL:
    plt.axis('equal')
    plt.show()

# plt.hist(angles, bins=50, alpha=0.5, label="angles")
# plt.xlabel("Angle difference (deg) between yaw extracted from the gps, and yaw inferred from a 3-point circle fit")
# plt.ylabel("count")
# plt.show()


    
            