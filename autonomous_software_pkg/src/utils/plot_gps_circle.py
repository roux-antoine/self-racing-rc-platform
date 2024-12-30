import math

import circle_fit
import matplotlib.pyplot as plt
import numpy as np
import utm
from sklearn.linear_model import LinearRegression

import rosbag

"""
Not very clean script to process the bagfiles that we recorded to characterize the low pass filter on the course-over-ground from the GPS
"""

MAGIC_FACTOR = 1  # to account for rotation direction, 1 for trigo, -1 for anti-trigo
SPEED_THRESHOLD = 0.5  # m/s
DEBUG = True
PARENT_FOLDER_BAGS = "TODO"

path = f"{PARENT_FOLDER_BAGS}/baseline_slow_2024-05-12-22-30-37.bag"
# path = f"{PARENT_FOLDER_BAGS}/baseline_slow_2024-05-12-22-32-22.bag"
# path = f"{PARENT_FOLDER_BAGS}/baseline_fast_2024-05-12-22-33-51.bag"
# path = f"{PARENT_FOLDER_BAGS}/baseline_fast_2024-05-12-22-37-00.bag"

# path = f"{PARENT_FOLDER_BAGS}/dynamic_fast_2024-05-12-22-52-53.bag"
# path = f"{PARENT_FOLDER_BAGS}/dynamic_slow_2024-05-12-22-51-39.bag"
# path = f"{PARENT_FOLDER_BAGS}/dynamic_slow_2024-05-12-22-54-05.bag"
# path = f"{PARENT_FOLDER_BAGS}/badynamic_slow_2024-05-12-22-48-36.bag"


# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.2_slow_2024-05-12-23-05-20.bag"
# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.2_slow_2024-05-12-23-08-23.bag"
# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.2_fast_2024-05-12-23-09-24.bag"  # wtf
# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.2_fast_2024-05-12-23-10-31.bag"  # wtf

# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.5_slow_2024-05-12-23-28-26.bag"
# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.5_slow_2024-05-12-23-29-09.bag"
# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.5_fast_after_restart_pos_mode_F_2024-05-13-00-00-49.bag"
# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.5_slow_after_restart_pos_mode_F_2024-05-12-23-59-59.bag"

# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.8_fast_2024-05-12-23-19-35.bag"
# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.8_fast_2024-05-12-23-20-19.bag"
# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.8_slow_2024-05-12-23-16-25.bag"
# path = f"{PARENT_FOLDER_BAGS}/cog_low_pass_0.8_slow_2024-05-12-23-17-58.bag"

bag = rosbag.Bag(path)


def compute_angle_difference():

    # extracting the data from the bag
    gps_positions = []
    gps_yaw_rads = []
    gps_speeds = []
    gps_times = []
    for topic, msg, t in bag.read_messages(topics=["/gps_info"]):

        gps_position = utm.from_latlon(msg.lat, msg.lon)
        yaw_rad = ((math.pi / 2) - msg.track * math.pi / 180) % (2 * np.pi)

        gps_positions.append(gps_position)
        gps_yaw_rads.append(yaw_rad)
        gps_speeds.append(msg.speed)
        gps_times.append(t.to_sec())

        if DEBUG:
            plt.scatter(gps_position[0], gps_position[1])
            # TODO plot the direction

    # fitting a circle
    xc_taubinSVD, yc_taubinSVD, r_taubinSVD, sigma_taubinSVD = circle_fit.taubinSVD(gps_positions)
    print(xc_taubinSVD, yc_taubinSVD, r_taubinSVD, sigma_taubinSVD)
    fitted_center = [xc_taubinSVD, yc_taubinSVD]
    if DEBUG:
        print(xc_taubinSVD, yc_taubinSVD, r_taubinSVD, sigma_taubinSVD)
        plt.scatter(fitted_center[0], fitted_center[1])
        plt.axis("equal")
        plt.xlabel("utm x (m)")
        plt.ylabel("utm y (m)")
        plt.show()

        distances_to_center = []
        for point in gps_positions:
            distances_to_center.append(np.linalg.norm([point[0] - fitted_center[0], point[1] - fitted_center[1]]))

        print(min(distances_to_center))
        print(max(distances_to_center))

        plt.hist(distances_to_center)
        plt.xlabel("distance from GPS point to fitted circle center (m)")
        plt.ylabel("count")
        plt.show()

    # computing the deviation of angles.
    angle_reals_filtered = []
    angle_diffs_filtered = []
    angles_gps_filtered = []
    times = []
    for gps_position, gps_yaw_rad, gps_speed, gps_time in zip(gps_positions, gps_yaw_rads, gps_speeds, gps_times):

        if gps_speed < SPEED_THRESHOLD:
            print(f"discarding message at t={t}")
        else:
            vec_to_fitted_center = np.array([fitted_center[0] - gps_position[0], fitted_center[1] - gps_position[1]])

            vec_from_fitted_center = -vec_to_fitted_center
            horizontal = np.array([1, 0])
            angle_real = (
                -MAGIC_FACTOR * np.pi / 2
                + np.arctan2(
                    horizontal[0] * vec_from_fitted_center[1] - horizontal[1] * vec_from_fitted_center[0],
                    horizontal[0] * vec_from_fitted_center[0] + horizontal[1] * vec_from_fitted_center[1],
                )
            ) % (2 * np.pi)

            angle_reals_filtered.append(angle_real)
            angles_gps_filtered.append(gps_yaw_rad)
            times.append(gps_time)

            angle_diff = gps_yaw_rad - angle_real
            if abs(angle_diff) > 3:  # HACK HACK to account for going over 2pi
                angle_diff = 0
            angle_diffs_filtered.append(angle_diff)

            if DEBUG:
                plt.scatter(gps_position[0], gps_position[1])
                plt.plot(
                    [gps_position[0], gps_position[0] + 0.5 * np.cos(gps_yaw_rad)],
                    [gps_position[1], gps_position[1] + 0.5 * np.sin(gps_yaw_rad)],
                    color="red",
                )
                plt.plot(
                    [gps_position[0], gps_position[0] + 0.5 * np.cos(angle_real)],
                    [gps_position[1], gps_position[1] + 0.5 * np.sin(angle_real)],
                    color="green",
                )
    if DEBUG:
        plt.scatter(fitted_center[0], fitted_center[1])
        plt.xlabel("utm x (m)")
        plt.ylabel("utm y (m)")
        plt.axis("equal")
        plt.title("TODO")
        plt.show()

    if DEBUG:
        plt.plot(times, angle_reals_filtered)
        plt.plot(times, angles_gps_filtered)
        plt.show()

    if DEBUG:
        print(np.mean(angle_diffs_filtered))
        plt.hist(angle_diffs_filtered)
        plt.xlabel("angle offset")
        plt.ylabel("count")
        plt.title("TODO")
        plt.show()


def plot_stats_and_do_regression():

    # measured using compute_angle_difference()
    w_baseline = [0.6, 0.7, 1.25, 1.5]
    theta_diff_baseline = [0.05, 0.04, 0.07, 0.07]
    w_vehicle_no_pass = [2.10, 0.90, 2.30, 0.80]
    theta_diff_vehicle_no_pass = [0.12, 0.04, 0.13, 0.05]
    w_vehicle_weak_pass = [2.60, 2.30, 1.1, 1.20]
    theta_diff_vehicle_weak_pass = [0.22, 0.17, 0.10, 0.09]
    w_vehicle_med_pass = [1.30, 1.20, 2.50, 1.00]
    theta_diff_vehicle_med_pass = [0.25, 0.18, 0.40, 0.18]
    w_vehicle_strong_pass = [0.85, 1.10]
    theta_diff_vehicle_strong_pass = [0.40, 0.50]

    x_min = 0
    x_mid = 1.3
    x_max = 2.7
    x_min_array = np.array([x_min]).reshape(-1, 1)
    x_mid_array = np.array([x_mid]).reshape(-1, 1)
    x_max_array = np.array([x_max]).reshape(-1, 1)

    reg_baseline = LinearRegression().fit(
        np.asarray(w_baseline).reshape(-1, 1), np.asarray(theta_diff_baseline).reshape(-1, 1)
    )
    reg_vehicle_no_pass = LinearRegression().fit(
        np.asarray(w_vehicle_no_pass).reshape(-1, 1), np.asarray(theta_diff_vehicle_no_pass).reshape(-1, 1)
    )
    reg_vehicle_weak_pass = LinearRegression().fit(
        np.asarray(w_vehicle_weak_pass).reshape(-1, 1), np.asarray(theta_diff_vehicle_weak_pass).reshape(-1, 1)
    )
    reg_vehicle_med_pass = LinearRegression().fit(
        np.asarray(w_vehicle_med_pass).reshape(-1, 1), np.asarray(theta_diff_vehicle_med_pass).reshape(-1, 1)
    )
    reg_vehicle_strong_pass = LinearRegression().fit(
        np.asarray(w_vehicle_strong_pass).reshape(-1, 1), np.asarray(theta_diff_vehicle_strong_pass).reshape(-1, 1)
    )

    plt.scatter(w_baseline, theta_diff_baseline, label="baseline - no low pass")
    plt.plot([x_min, x_max], [reg_baseline.predict(x_min_array)[0][0], reg_baseline.predict(x_max_array)[0][0]])
    plt.scatter(w_vehicle_no_pass, theta_diff_vehicle_no_pass, label="vehicle - no low pass")
    plt.plot(
        [x_min, x_max], [reg_vehicle_no_pass.predict(x_min_array)[0][0], reg_vehicle_no_pass.predict(x_max_array)[0][0]]
    )
    plt.scatter(w_vehicle_weak_pass, theta_diff_vehicle_weak_pass, label="vehicle - weak low pass")
    plt.plot(
        [x_min, x_max],
        [reg_vehicle_weak_pass.predict(x_min_array)[0][0], reg_vehicle_weak_pass.predict(x_max_array)[0][0]],
    )
    plt.scatter(w_vehicle_med_pass, theta_diff_vehicle_med_pass, label="vehicle - medium low pass")
    plt.plot(
        [x_min, x_max],
        [reg_vehicle_med_pass.predict(x_min_array)[0][0], reg_vehicle_med_pass.predict(x_max_array)[0][0]],
    )
    plt.scatter(w_vehicle_strong_pass, theta_diff_vehicle_strong_pass, label="vehicle - strong low pass")
    plt.plot(
        [x_min, x_mid],
        [reg_vehicle_strong_pass.predict(x_min_array)[0][0], reg_vehicle_strong_pass.predict(x_mid_array)[0][0]],
    )

    plt.xlim([x_min, x_max])
    plt.xlabel("rotation rate (rad/s)")
    plt.ylabel("average offset between real and GPS CoG (rad)")
    plt.legend()
    plt.show()


##############

compute_angle_difference()
plot_stats_and_do_regression()
