# from nmea_msgs.msg import Gprmc

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math
import utm




path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/baseline_slow_2024-05-12-22-30-37.bag"
path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/baseline_slow_2024-05-12-22-32-22.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/baseline_fast_2024-05-12-22-33-51.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/baseline_fast_2024-05-12-22-37-00.bag"

# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/dynamic_fast_2024-05-12-22-52-53.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/dynamic_slow_2024-05-12-22-51-39.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/dynamic_slow_2024-05-12-22-54-05.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/badynamic_slow_2024-05-12-22-48-36.bag"


# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.2_slow_2024-05-12-23-05-20.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.2_slow_2024-05-12-23-08-23.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.2_fast_2024-05-12-23-09-24.bag"  # wtf
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.2_fast_2024-05-12-23-10-31.bag" # wtf

# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.5_slow_2024-05-12-23-28-26.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.5_slow_2024-05-12-23-29-09.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.5_fast_after_restart_pos_mode_F_2024-05-13-00-00-49.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.5_slow_after_restart_pos_mode_F_2024-05-12-23-59-59.bag"

# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.8_fast_2024-05-12-23-19-35.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.8_fast_2024-05-12-23-20-19.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.8_slow_2024-05-12-23-16-25.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.8_slow_2024-05-12-23-17-58.bag"

# useless?
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.8_fast_2024-05-12-23-19-35.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.5_slow_after_restart_pos_mode_F_2024-05-12-23-59-59.bag"
# path = "/Users/antoineroux/Downloads/bags_gps_2024-05-12/cog_low_pass_0.5_slow_after_restart_pos_mode_D_2024-05-12-23-40-43.bag"

bag = rosbag.Bag(path)

MAGIC_FACTOR = 1 # to account for rotation direction

# point_close_to_real_center = np.array([554320.4, 4178261.0])  # SF
# point_close_to_real_center = np.array([558620.0, 4154730.4])  # SM

def compute_center_position_dispersion(radius, angle_offset, plot):

    discarded_center_positions = []
    center_positions = []

    for topic, msg, t in bag.read_messages(topics=["/gps_info"]):

        utm_values = utm.from_latlon(msg.lat, msg.lon)
        yaw_rad = (math.pi / 2) - msg.track * math.pi / 180
        yaw_rad += angle_offset  # HACK HACK HACK
        

        normal = np.array([np.sin(yaw_rad), -np.cos(yaw_rad)])
        center = np.array([
            utm_values[0] + MAGIC_FACTOR * radius * normal[0],
            utm_values[1] + MAGIC_FACTOR * radius * normal[1],

        ])


        if msg.speed < 0.5:
            discarded_center_positions.append(center)
            print(f"discaring message at t={t}")
        else:
            center_positions.append(center)

        if plot:
            plt.scatter(utm_values[0], utm_values[1])
            plt.plot([utm_values[0], utm_values[0] + 0.1 * np.cos(yaw_rad)], [utm_values[1], utm_values[1] + 0.1 * np.sin(yaw_rad)])
            plt.scatter(center[0], center[1], color="blue") 

    print(f"keeping {len(center_positions)} points")


    center_of_centers_x = 0
    center_of_centers_y = 0
    for point in center_positions:
        center_of_centers_x += point[0]
        center_of_centers_y += point[1]
    center_of_centers = np.array([center_of_centers_x, center_of_centers_y]) / len(center_positions)

    residuals = 0
    for point in center_positions:
        residuals += np.linalg.norm(point - center_of_centers)
    residuals = residuals / len(center_positions)

    if plot:
        for discarded_center in discarded_center_positions:
            plt.scatter(discarded_center[0], discarded_center[1], color="red")    

        plt.plot([utm_values[0], utm_values[0] + 0.1 * np.cos(yaw_rad)], [utm_values[1], utm_values[1] + 0.1 * np.sin(yaw_rad)])

        plt.scatter(center_of_centers[0], center_of_centers[1], marker="+")    
        plt.axis("equal")
        plt.show()

    # computing the deviation of angles.
    angle_diffs = []
    angle_reals = []
    angle_reals_derivative = []
    angle_measureds = []
    times = []
    for topic, msg, t in bag.read_messages(topics=["/gps_info"]):

        utm_values = utm.from_latlon(msg.lat, msg.lon)
        yaw_rad = (math.pi / 2) - msg.track * math.pi / 180
        yaw_rad = (yaw_rad + angle_offset) % (2*np.pi)  # That's to perform the parameter sweep!

        normal = np.array([np.sin(yaw_rad), -np.cos(yaw_rad)])
        center = np.array([
            utm_values[0] + MAGIC_FACTOR * radius * normal[0],
            utm_values[1] + MAGIC_FACTOR * radius * normal[1],

        ])

        if msg.speed < 0.5:
            discarded_center_positions.append(center)
            print(f"discaring message at t={t}")
        else:
            vec_to_center_of_centers = np.array([center_of_centers[0] - utm_values[0],
                                        center_of_centers[1] - utm_values[1]])
            vec_to_center = np.array([center[0] - utm_values[0],
                                      center[1] - utm_values[1]])
            vec_from_center_of_centers = -vec_to_center_of_centers
            horizontal = np.array([1, 0])
            angle_real = (-MAGIC_FACTOR * np.pi / 2 + np.arctan2(horizontal[0] * vec_from_center_of_centers[1] - horizontal[1] * vec_from_center_of_centers[0], horizontal[0]*vec_from_center_of_centers[0]+horizontal[1]*vec_from_center_of_centers[1])) % (2*np.pi)

            if len(angle_reals):
                angle_real_derivative = (angle_real - angle_reals[-1]) / 0.1
                if abs(angle_real_derivative) > 10:
                    angle_reals_derivative.append(0)
                else: 
                    angle_reals_derivative.append(angle_real_derivative)
            angle_reals.append(angle_real)
            angle_measureds.append(yaw_rad)
            times.append(t.to_sec())
            # print(angle_real)
            # print(yaw_rad)
            # plt.scatter(center_of_centers[0], center_of_centers[1])
            # plt.scatter(utm_values[0], utm_values[1])
            # plt.plot([utm_values[0], utm_values[0] + 0.1 * np.cos(yaw_rad)], [utm_values[1], utm_values[1] + 0.1 * np.sin(yaw_rad)])
            # plt.plot([utm_values[0], utm_values[0] + 0.1 * np.cos(angle_real)], [utm_values[1], utm_values[1] + 0.1 * np.sin(angle_real)])
            # plt.axis("equal")
            # plt.show()

            # import sys
            # sys.exit()
    
            angle_diff = yaw_rad - angle_real
            angle_diffs.append(angle_diff)


    # plt.plot(times, angle_reals)
    # plt.plot(times, angle_measureds)
    # # plt.hlines(3, 0, 25)
    # plt.show()

    # plt.plot(times[1:], angle_reals_derivative)
    # plt.show()

    # plt.plot([elem for elem in angle_diffs if abs(elem) < 1])
    # plt.show()

    # if plot:
    #     print(np.mean(angle_diffs))
    #     plt.hist(angle_diffs)
    #     plt.xlabel("angle offset")
    #     plt.ylabel("count")
    #     plt.show()
    return residuals

compute_center_position_dispersion(2.37, 0, True)

radiuses = np.linspace(2.36, 2.38, 5)
angle_offsets = np.linspace(-0.2, 0.2, 30)
for radius in radiuses:
    residuals = []
    for angle_offset in angle_offsets:
        residuals.append(compute_center_position_dispersion(radius, angle_offset, False))

    plt.plot(angle_offsets, residuals, label=f"radius: {radius}")

plt.legend()
plt.xlabel("artificial angle_offset (rad)")
plt.ylabel("average dispersion of the 'centers' around the real center of rotation (m)")
plt.show()


"""
baseline
w          0.6 0.7 1.25 1.50
theta_diff 0.05 0.04 0.07 0.07

vehicle
w          2.10 0.90 2.30 0.80
theta_diff 0.12 0.04 0.13 0.05

lowpass 0.2
w          0.85 1.10
theta_diff 0.40 0.50

lowpass 0.5
w          1.30 1.20 2.50 1.00
theta_diff 0.25 0.18 0.40 0.18


lowpass 0.8
w          2.60 2.30 1.10 1.20
theta_diff 0.22 0.17 0.10 0.09

"""

