import pandas as pd
import matplotlib.pyplot as plt

import numpy as np
from scipy.optimize import curve_fit

UPPER_BOUND_REGION_1 = 1.5  # m/s
UPPER_BOUND_REGION_2 = 5  # m/s
UPPER_BOUND_REGION_3 = 8  # m/s
COEFF_REGION_1 = 27 * 1.25
COEFF_REGION_2 = 24 * 2.3
COEFF_REGION_3 = 26 * 4
WHEELBASE = 0.406

# Read the CSV file
df = pd.read_csv("manually_written_csv.csv")

# plt.plot(df["mean_window_speed"])
# plt.show()

# df_1 = df[df["mean_window_speed"] < UPPER_BOUND_REGION_1]
# df_2 = df[(df["mean_window_speed"] >= UPPER_BOUND_REGION_1) & (df["mean_window_speed"] < UPPER_BOUND_REGION_2)]
# df_3 = df[(df["mean_window_speed"] >= UPPER_BOUND_REGION_2) & (df["mean_window_speed"] < UPPER_BOUND_REGION_3)]
# df_4 = df[(df["mean_window_speed"] >= UPPER_BOUND_REGION_3)]

# Extract the relevant columns
radius_df = df["mean_window_radius_from_velocity"]
steering_angle_diff_df = df["mean_window_steering_angle_deviation_from_center"]
steering_angle_df = df["mean_window_steering_angle"]
speed_df = df["mean_window_speed"]
acceleration_df = df["mean_window_acceleration_from_velocity"]

# Plot steering angle vs radius, colored by speed
plt.scatter(
    steering_angle_df,
    radius_df,
    c=speed_df,
    cmap="viridis",
    alpha=0.8,
    label="Colored by speed",
    vmin=2.5,
    vmax=8,
)
plt.colorbar(label="Speed (m/s)")
plt.xlabel("Steering Angle")
plt.ylabel("Radius (m)")
plt.title("Radius vs Steering Angle (Colored by Speed)")
plt.grid(True)
plt.show()

# Plot steering angle vs radius, colored by acceleration
plt.scatter(
    steering_angle_df,
    radius_df,
    c=acceleration_df,
    cmap="viridis",
    alpha=0.8,
    label="Colored by acceleration",
    vmin=2.5,
    vmax=8,
)
plt.colorbar(label="Mean Window Acceleration (m/s²)")
plt.xlabel("Steering Angle")
plt.ylabel("Mean Window Radius from Velocity (m)")
plt.title("Radius vs Steering Angle (Colored by Acceleration)")
plt.grid(True)
plt.show()

# Plot steering angle diff vs radius, colored by speed
plt.scatter(
    steering_angle_diff_df,
    radius_df,
    c=speed_df,
    cmap="viridis",
    alpha=0.8,
    label="Colored by speed",
    vmin=2.5,
    vmax=8,
)
plt.colorbar(label="Speed (m/s)")
plt.xlabel("Steering diff")
plt.ylabel("Radius (m)")
plt.title("Radius vs Steering diff (Colored by Speed)")
plt.grid(True)
plt.show()

# Plot steering angle diff vs radius, colored by acceleration
plt.scatter(
    steering_angle_diff_df,
    radius_df,
    c=acceleration_df,
    cmap="viridis",
    alpha=0.8,
    label="Colored by acceleration",
    vmin=2.5,
    vmax=8,
)
plt.colorbar(label="Mean Window Acceleration (m/s²)")
plt.xlabel("Mean Window Steering Angle Deviation from Center")
plt.ylabel("Mean Window Radius from Velocity (m)")
plt.title("Radius vs Steering Angle Deviation (Colored by Acceleration)")
plt.grid(True)
plt.show()


# radius_1 = df_1["mean_window_radius_from_velocity"]
# steering_deviation_1 = df_1["mean_window_steering_angle_deviation_from_center"]
# radius_2 = df_2["mean_window_radius_from_velocity"]
# steering_deviation_2 = df_2["mean_window_steering_angle_deviation_from_center"]
# radius_3 = df_3["mean_window_radius_from_velocity"]
# steering_deviation_3 = df_3["mean_window_steering_angle_deviation_from_center"]
# radius_4 = df_4["mean_window_radius_from_velocity"]
# steering_deviation_4 = df_4["mean_window_steering_angle_deviation_from_center"]


def radius_calc_antoine_model(steering_diff, speed):

    if speed <= UPPER_BOUND_REGION_1:
        coeff = 1 / COEFF_REGION_1
    elif speed > UPPER_BOUND_REGION_1 and speed <= UPPER_BOUND_REGION_2:
        coeff = 1 / (
            COEFF_REGION_1
            + (speed - UPPER_BOUND_REGION_1)
            * (COEFF_REGION_2 - COEFF_REGION_1)
            / (UPPER_BOUND_REGION_2 - UPPER_BOUND_REGION_1)
        )
    elif speed > UPPER_BOUND_REGION_2 and speed <= UPPER_BOUND_REGION_3:
        coeff = 1 / (
            COEFF_REGION_2
            + (speed - UPPER_BOUND_REGION_2)
            * (COEFF_REGION_3 - COEFF_REGION_2)
            / (UPPER_BOUND_REGION_3 - UPPER_BOUND_REGION_2)
        )
    elif speed > UPPER_BOUND_REGION_3:
        coeff = 1 / COEFF_REGION_3
    else:
        raise ValueError()

    return 1 / (steering_diff * coeff)


def radius_calc_bicyle_model(steering_diff):
    return WHEELBASE / np.tan((0.523599 / 27) * steering_diff)


# steering_diff_linspace = np.linspace(5, 25, 100)
# radii_at_region_1_upper_bound = radius_calc_antoine_model(steering_diff_linspace, UPPER_BOUND_REGION_1)
# radii_at_region_2_upper_bound = radius_calc_antoine_model(steering_diff_linspace, UPPER_BOUND_REGION_2)
# radii_at_region_3_upper_bound = radius_calc_antoine_model(steering_diff_linspace, UPPER_BOUND_REGION_3)

# # Plot
# plt.figure()
# plt.scatter(steering_deviation_1, radius_1)
# plt.plot(steering_diff_linspace, radii_1)
# plt.scatter(steering_deviation_2, radius_2)
# plt.scatter(steering_deviation_3, radius_3)
# plt.scatter(steering_deviation_4, radius_4)
# plt.xlabel("Mean Window Steering Angle Deviation from Center")
# plt.ylabel("Mean Window Radius from Velocity (m)")
# plt.title("Radius vs Steering Angle Deviation")
# plt.grid(True)
# plt.show()

plt.figure()


for steering_deviation, radius, speed in zip(
    steering_angle_diff_df, radius_df, speed_df
):
    computed_radius_antoine_model = radius_calc_antoine_model(steering_deviation, speed)
    computed_radius_bicycle_model = radius_calc_bicyle_model(steering_deviation)

    plt.scatter(
        steering_deviation,
        radius,
        c=speed,
        cmap="viridis",
        marker="o",
        vmin=2.5,
        vmax=8,
    )
    plt.scatter(
        steering_deviation,
        computed_radius_antoine_model,
        c=speed,
        cmap="viridis",
        marker="x",
        vmin=2.5,
        vmax=8,
    )
    plt.scatter(
        steering_deviation, computed_radius_bicycle_model, c="black", marker="+"
    )
    plt.plot(
        [steering_deviation, steering_deviation],
        [radius, computed_radius_antoine_model],
        color=plt.cm.viridis((speed - 2.5) / (8 - 2.5)),
    )


# Plot with color depending on radius
# sc = plt.scatter(
#     steering_angle_diff_df,
#     radius_df,
#     c=speed_df,
#     cmap='viridis',
#     alpha=0.8,
#     label='Colored by speed',
#     vmin=2.5, vmax=8
# )
# plt.plot(steering_diff_linspace, radii_at_region_1_upper_bound, label='Region 1 Upper Bound', color='blue')
# plt.plot(steering_diff_linspace, radii_at_region_2_upper_bound, label='Region 1 Upper Bound', color='blue')
# plt.plot(steering_diff_linspace, radii_at_region_3_upper_bound, label='Region 1 Upper Bound', color='blue')


#####

UPPER_BOUND_REGION_1_NEW = 4  # m/s
UPPER_BOUND_REGION_2_NEW = 6.1  # m/s
UPPER_BOUND_REGION_3_NEW = 7.1  # m/s
UPPER_BOUND_REGION_4_NEW = 10.0  # m/s

df_1 = df[df["mean_window_speed"] < UPPER_BOUND_REGION_1_NEW]
df_2 = df[
    (df["mean_window_speed"] >= UPPER_BOUND_REGION_1_NEW)
    & (df["mean_window_speed"] < UPPER_BOUND_REGION_2_NEW)
]
df_3 = df[
    (df["mean_window_speed"] >= UPPER_BOUND_REGION_2_NEW)
    & (df["mean_window_speed"] < UPPER_BOUND_REGION_3_NEW)
]
df_4 = df[
    (df["mean_window_speed"] >= UPPER_BOUND_REGION_3_NEW)
    & (df["mean_window_speed"] < UPPER_BOUND_REGION_4_NEW)
]

print(len(df_1), len(df_2), len(df_3), len(df_4))

plt.figure()
sc = plt.scatter(
    df_1["mean_window_steering_angle_deviation_from_center"],
    df_1["mean_window_radius_from_velocity"],
    alpha=0.8,
)
sc = plt.scatter(
    df_2["mean_window_steering_angle_deviation_from_center"],
    df_2["mean_window_radius_from_velocity"],
    alpha=0.8,
)
sc = plt.scatter(
    df_3["mean_window_steering_angle_deviation_from_center"],
    df_3["mean_window_radius_from_velocity"],
    alpha=0.8,
)
sc = plt.scatter(
    df_4["mean_window_steering_angle_deviation_from_center"],
    df_4["mean_window_radius_from_velocity"],
    alpha=0.8,
)
plt.show()

# radius_df = df["mean_window_radius_from_velocity"]
# steering_angle_diff_df = df["mean_window_steering_angle_deviation_from_center"]
# steering_angle_df = df["mean_window_steering_angle"]
# speed_df = df["mean_window_speed"]
# acceleration_df = df["mean_window_acceleration_from_velocity"]

# BUILDING A MODEL OF THE RADIUS


def model_func(x, a, b):  # y is the radius, x is the steering angle deviation
    return (WHEELBASE / x) * a + b  # The b is a hack


x_data_1 = df_1["mean_window_steering_angle_deviation_from_center"].values
y_data_1 = df_1["mean_window_radius_from_velocity"].values
popt_1, pcov_1 = curve_fit(model_func, x_data_1, y_data_1, p0=[1, 1])
print(popt_1)

x_data_2 = df_2["mean_window_steering_angle_deviation_from_center"].values
y_data_2 = df_2["mean_window_radius_from_velocity"].values
popt_2, pcov_2 = curve_fit(model_func, x_data_2, y_data_2, p0=[1, 1])
print(popt_2)

x_data_3 = df_3["mean_window_steering_angle_deviation_from_center"].values
y_data_3 = df_3["mean_window_radius_from_velocity"].values
popt_3, pcov_3 = curve_fit(model_func, x_data_3, y_data_3, p0=[1, 1])
print(popt_3)

x_data_4 = df_4["mean_window_steering_angle_deviation_from_center"].values
y_data_4 = df_4["mean_window_radius_from_velocity"].values
popt_4, pcov_4 = curve_fit(model_func, x_data_4, y_data_4, p0=[1, 1])
print(popt_4)

# Plot the fit
x_fit = np.linspace(5, 28, 100)
y_fit_1 = model_func(x_fit, *popt_1)
y_fit_2 = model_func(x_fit, *popt_2)
y_fit_3 = model_func(x_fit, *popt_3)
y_fit_4 = model_func(x_fit, *popt_4)

plt.figure()
plt.scatter(x_data_1, y_data_1, label="Data")
plt.plot(x_fit, y_fit_1, color="red", label="Fit: y = (WHEELBASE / x) * a + b")
plt.scatter(x_data_2, y_data_2, label="Data")
plt.plot(x_fit, y_fit_2, color="red", label="Fit: y = (WHEELBASE / x) * a + b")
plt.scatter(x_data_3, y_data_3, label="Data")
plt.plot(x_fit, y_fit_3, color="red", label="Fit: y = (WHEELBASE / x) * a + b")
plt.scatter(x_data_4, y_data_4, label="Data")
plt.plot(x_fit, y_fit_4, color="red", label="Fit: y = (WHEELBASE / x) * a + b")
plt.xlabel("Mean Window Steering Angle Deviation from Center")
plt.ylabel("Mean Window Radius from Velocity (m)")
plt.legend()
plt.show()


# # BUILDING A MODEL OF THE COEFFS
# # coeff = (radius * steering_diff) / WHEELBASE
# coeffs_1 = (df_1["mean_window_radius_from_velocity"].values * df_1["mean_window_steering_angle_deviation_from_center"].values) / WHEELBASE
# coeffs_2 = (df_2["mean_window_radius_from_velocity"].values * df_2["mean_window_steering_angle_deviation_from_center"].values) / WHEELBASE
# coeffs_3 = (df_3["mean_window_radius_from_velocity"].values * df_3["mean_window_steering_angle_deviation_from_center"].values) / WHEELBASE
# coeffs_4 = (df_4["mean_window_radius_from_velocity"].values * df_4["mean_window_steering_angle_deviation_from_center"].values) / WHEELBASE

# print("Coeffs 1:", coeffs_1)
# print("Coeffs 2:", coeffs_2)
# print("Coeffs 3:", coeffs_3)
# print("Coeffs 4:", coeffs_4)

# BUILDING A MODEL OF THE COEFFS
# coeff = (radius * steering_diff) / WHEELBASE
coeffs_1 = (
    df_1["mean_window_radius_from_velocity"].values
    * df_1["mean_window_steering_angle_deviation_from_center"].values
)
coeffs_2 = (
    df_2["mean_window_radius_from_velocity"].values
    * df_2["mean_window_steering_angle_deviation_from_center"].values
)
coeffs_3 = (
    df_3["mean_window_radius_from_velocity"].values
    * df_3["mean_window_steering_angle_deviation_from_center"].values
)
coeffs_4 = (
    df_4["mean_window_radius_from_velocity"].values
    * df_4["mean_window_steering_angle_deviation_from_center"].values
)

print("Coeffs 1:", coeffs_1.mean())
print("Coeffs 2:", coeffs_2.mean())
print("Coeffs 3:", coeffs_3.mean())
print("Coeffs 4:", coeffs_4.mean())
