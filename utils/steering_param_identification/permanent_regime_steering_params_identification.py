import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# TODO more work needed

df = pd.read_csv(
    "/Users/antoineroux/Downloads/20250824_canada_college/open_loop_steering/fitted_circles.csv"
)


# Extract the relevant columns
radius_df = df["radius"]
steering_angle_df = df["mean_steering_cmd"]
steering_angle_diff_df = abs(df["mean_steering_cmd"] - 90)
speed_df = df["mean_speed"]


# Plot speed vs radius
r = np.linspace(0, 10, 100)
v = np.sqrt(9.81 * r)
plt.plot(r, v, label="v = sqrt(g * r)", color="orange")
plt.scatter(radius_df, speed_df)
plt.xlabel("Radius (m)")
plt.ylabel("Speed (m/s)")
plt.title("Speed vs Radius")
plt.grid(True)
plt.show()

# Plot steering angle vs radius, colored by speed
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
plt.xlabel("Steering Diff (|cmd - 90|)")
plt.ylabel("Radius (m)")
plt.title("Radius vs Steering Diff (Colored by Speed)")
plt.grid(True)
plt.show()
