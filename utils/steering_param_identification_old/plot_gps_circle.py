import math

import circle_fit
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import utm
from sklearn.linear_model import LinearRegression


"""
Not very clean script to process the bagfiles that we recorded to characterize the low pass filter on the course-over-ground from the GPS
"""

MAGIC_FACTOR = -1  # to account for rotation direction, 1 for trigo, -1 for anti-trigo
SPEED_THRESHOLD = 0.5  # m/s
DEBUG = True

# Estimated GPS CoG filter time constant (seconds), derived from the slope of
# the linear regression: offset ≈ ω · τ.  Use the "vehicle - no low pass"
# regression slope as the default (≈ 0.06 s).
GPS_COG_FILTER_TAU = 0.03
PARENT_FOLDER_BAGS = "/Users/antoineroux/Downloads/bags_gps_2024-05-12"

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


def inverse_first_order_lpf(filtered_signal, timestamps, tau):
    """Reverse a first-order low-pass filter applied to an angle signal.

    Given the filtered output and the filter time constant τ, recovers an
    estimate of the original (unfiltered) signal.

    For a first-order LPF:  y[n] = α·x[n] + (1-α)·y[n-1]
    Inverting:               x[n] = (y[n] - (1-α)·y[n-1]) / α
    where α = dt / (τ + dt).
    """
    recovered = [filtered_signal[0]]
    for i in range(1, len(filtered_signal)):
        dt = timestamps[i] - timestamps[i - 1]
        alpha = dt / (tau + dt)
        prev = filtered_signal[i - 1]
        diff = filtered_signal[i] - prev
        if diff > np.pi:
            prev += 2 * np.pi
        elif diff < -np.pi:
            prev -= 2 * np.pi
        raw = (filtered_signal[i] - (1 - alpha) * prev) / alpha
        recovered.append(raw)
    return recovered


def compute_angle_difference():

    # extracting the data from the bag
    gps_positions = []
    gps_yaw_rads = []
    gps_speeds = []
    gps_times = []

    counter = 0
    for topic, msg, t in bag.read_messages(topics=["/gps_info"]):

        if counter > 100000:
            continue
        counter += 1

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
    xc_taubinSVD, yc_taubinSVD, r_taubinSVD, sigma_taubinSVD = circle_fit.taubinSVD(
        gps_positions
    )
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
            distances_to_center.append(
                np.linalg.norm(
                    [point[0] - fitted_center[0], point[1] - fitted_center[1]]
                )
            )

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
    corrected_angle_diffs_filtered = []
    corrected_cog_inline = []
    prev_gps_yaw_rad = None
    prev_gps_time = None
    times = []
    for gps_position, gps_yaw_rad, gps_speed, gps_time in zip(
        gps_positions, gps_yaw_rads, gps_speeds, gps_times
    ):

        if gps_speed < SPEED_THRESHOLD:
            print(f"discarding message at t={t}")
        else:
            vec_to_fitted_center = np.array(
                [fitted_center[0] - gps_position[0], fitted_center[1] - gps_position[1]]
            )

            vec_from_fitted_center = -vec_to_fitted_center
            horizontal = np.array([1, 0])
            angle_real = (
                -MAGIC_FACTOR * np.pi / 2
                + np.arctan2(
                    horizontal[0] * vec_from_fitted_center[1]
                    - horizontal[1] * vec_from_fitted_center[0],
                    horizontal[0] * vec_from_fitted_center[0]
                    + horizontal[1] * vec_from_fitted_center[1],
                )
            ) % (2 * np.pi)

            angle_reals_filtered.append(angle_real)
            angles_gps_filtered.append(gps_yaw_rad)
            times.append(gps_time)

            # Inverse-filter the GPS CoG inline
            if prev_gps_yaw_rad is None:
                corrected_yaw = gps_yaw_rad
            else:
                dt = gps_time - prev_gps_time
                alpha = dt / (GPS_COG_FILTER_TAU + dt)
                # Unwrap prev relative to current to avoid 0/2π boundary jump
                prev_unwrapped = prev_gps_yaw_rad
                diff = gps_yaw_rad - prev_unwrapped
                if diff > np.pi:
                    prev_unwrapped += 2 * np.pi
                elif diff < -np.pi:
                    prev_unwrapped -= 2 * np.pi
                corrected_yaw = (gps_yaw_rad - (1 - alpha) * prev_unwrapped) / alpha
            corrected_cog_inline.append(corrected_yaw)
            prev_gps_yaw_rad = gps_yaw_rad
            prev_gps_time = gps_time

            angle_diff = gps_yaw_rad - angle_real
            if abs(angle_diff) > 3:  # HACK HACK to account for going over 2pi
                angle_diff = 0
            angle_diffs_filtered.append(angle_diff)

            corrected_angle_diff = corrected_yaw - angle_real
            if abs(corrected_angle_diff) > 3:  # HACK HACK to account for going over 2pi
                corrected_angle_diff = 0
            corrected_angle_diffs_filtered.append(corrected_angle_diff)

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
                plt.plot(
                    [gps_position[0], gps_position[0] + 0.5 * np.cos(corrected_yaw)],
                    [gps_position[1], gps_position[1] + 0.5 * np.sin(corrected_yaw)],
                    color="blue",
                )

            
    if DEBUG:
        plt.scatter(fitted_center[0], fitted_center[1])
        plt.xlabel("utm x (m)")
        plt.ylabel("utm y (m)")
        plt.axis("equal")
        plt.title("TODO")
        plt.show()

    if DEBUG:
        plt.plot(times, angle_reals_filtered, label="ground truth")
        plt.plot(times, angles_gps_filtered, label="GPS CoG (raw)")
        plt.plot(times, corrected_cog_inline, label="GPS CoG (corrected)")
        plt.xlabel("time (s)")
        plt.ylabel("angle (rad)")
        plt.legend()
        plt.show()

    if DEBUG:
        plt.hist(angle_diffs_filtered, alpha=0.5, label="raw GPS offset")
        plt.hist(corrected_angle_diffs_filtered, alpha=0.5, label="corrected GPS offset")
        plt.xlabel("angle offset (rad)")
        plt.ylabel("count")
        plt.legend()
        plt.title("Offset distribution: raw vs corrected")
        plt.show()

    if DEBUG:
        raw_mean = np.mean(np.abs(angle_diffs_filtered))
        corrected_mean = np.mean(np.abs(corrected_angle_diffs_filtered))
        plt.hist(np.abs(angle_diffs_filtered), bins=20, alpha=0.5, label="raw GPS offset")
        plt.hist(np.abs(corrected_angle_diffs_filtered), bins=20, alpha=0.5, label="corrected GPS offset")
        plt.axvline(raw_mean, color="tab:blue", linestyle="--", label=f"raw mean ({raw_mean:.3f})")
        plt.axvline(corrected_mean, color="tab:orange", linestyle="--", label=f"corrected mean ({corrected_mean:.3f})")
        plt.xlabel("angle offset (rad)")
        plt.ylabel("count")
        plt.legend()
        plt.title("Offset distribution: raw vs corrected (abs)")
        plt.show()

    print(f"Raw GPS mean offset: {np.mean(np.abs(angle_diffs_filtered)):.4f} rad = {np.mean(np.abs(angle_diffs_filtered)) * 180 / np.pi:.2f} deg")
    print(f"Corrected mean offset: {np.mean(np.abs(corrected_angle_diffs_filtered)):.4f} rad = {np.mean(np.abs(corrected_angle_diffs_filtered)) * 180 / np.pi:.2f} deg)")

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
        np.asarray(w_baseline).reshape(-1, 1),
        np.asarray(theta_diff_baseline).reshape(-1, 1),
    )
    reg_vehicle_no_pass = LinearRegression().fit(
        np.asarray(w_vehicle_no_pass).reshape(-1, 1),
        np.asarray(theta_diff_vehicle_no_pass).reshape(-1, 1),
    )
    reg_vehicle_weak_pass = LinearRegression().fit(
        np.asarray(w_vehicle_weak_pass).reshape(-1, 1),
        np.asarray(theta_diff_vehicle_weak_pass).reshape(-1, 1),
    )
    reg_vehicle_med_pass = LinearRegression().fit(
        np.asarray(w_vehicle_med_pass).reshape(-1, 1),
        np.asarray(theta_diff_vehicle_med_pass).reshape(-1, 1),
    )
    reg_vehicle_strong_pass = LinearRegression().fit(
        np.asarray(w_vehicle_strong_pass).reshape(-1, 1),
        np.asarray(theta_diff_vehicle_strong_pass).reshape(-1, 1),
    )

    # The slope of offset vs ω gives τ (the first-order LPF time constant)
    tau_baseline = reg_baseline.coef_[0][0]
    tau_vehicle_no_pass = reg_vehicle_no_pass.coef_[0][0]
    tau_vehicle_weak_pass = reg_vehicle_weak_pass.coef_[0][0]
    tau_vehicle_med_pass = reg_vehicle_med_pass.coef_[0][0]
    tau_vehicle_strong_pass = reg_vehicle_strong_pass.coef_[0][0]

    print(f"Estimated tau (baseline):          {tau_baseline:.4f} s")
    print(f"Estimated tau (vehicle, no pass):   {tau_vehicle_no_pass:.4f} s")
    print(f"Estimated tau (vehicle, weak pass): {tau_vehicle_weak_pass:.4f} s")
    print(f"Estimated tau (vehicle, med pass):  {tau_vehicle_med_pass:.4f} s")
    print(f"Estimated tau (vehicle, strong pass): {tau_vehicle_strong_pass:.4f} s")

    plt.scatter(w_baseline, theta_diff_baseline, label="baseline - no low pass")
    plt.plot(
        [x_min, x_max],
        [
            reg_baseline.predict(x_min_array)[0][0],
            reg_baseline.predict(x_max_array)[0][0],
        ],
    )
    plt.scatter(
        w_vehicle_no_pass, theta_diff_vehicle_no_pass, label="vehicle - no low pass"
    )
    plt.plot(
        [x_min, x_max],
        [
            reg_vehicle_no_pass.predict(x_min_array)[0][0],
            reg_vehicle_no_pass.predict(x_max_array)[0][0],
        ],
    )
    plt.scatter(
        w_vehicle_weak_pass,
        theta_diff_vehicle_weak_pass,
        label="vehicle - weak low pass",
    )
    plt.plot(
        [x_min, x_max],
        [
            reg_vehicle_weak_pass.predict(x_min_array)[0][0],
            reg_vehicle_weak_pass.predict(x_max_array)[0][0],
        ],
    )
    plt.scatter(
        w_vehicle_med_pass,
        theta_diff_vehicle_med_pass,
        label="vehicle - medium low pass",
    )
    plt.plot(
        [x_min, x_max],
        [
            reg_vehicle_med_pass.predict(x_min_array)[0][0],
            reg_vehicle_med_pass.predict(x_max_array)[0][0],
        ],
    )
    plt.scatter(
        w_vehicle_strong_pass,
        theta_diff_vehicle_strong_pass,
        label="vehicle - strong low pass",
    )
    plt.plot(
        [x_min, x_mid],
        [
            reg_vehicle_strong_pass.predict(x_min_array)[0][0],
            reg_vehicle_strong_pass.predict(x_mid_array)[0][0],
        ],
    )

    plt.xlim([x_min, x_max])
    plt.xlabel("rotation rate (rad/s)")
    plt.ylabel("average offset between real and GPS CoG (rad)")
    plt.legend()
    plt.show()

    # Plot estimated tau for each filter setting
    filter_labels = ["baseline", "no pass", "weak", "medium", "strong"]
    tau_values = [
        tau_baseline,
        tau_vehicle_no_pass,
        tau_vehicle_weak_pass,
        tau_vehicle_med_pass,
        tau_vehicle_strong_pass,
    ]
    plt.bar(filter_labels, tau_values)
    plt.xlabel("filter setting")
    plt.ylabel("estimated tau (s)")
    plt.title("Estimated first-order LPF time constant per setting")
    plt.show()


##############

compute_angle_difference()
plot_stats_and_do_regression()
