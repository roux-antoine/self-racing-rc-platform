import pandas as pd
import plotly.graph_objects as go
import numpy as np
from scipy.optimize import curve_fit
from plotly.subplots import make_subplots
import matplotlib.pyplot as plt

SPEED_REGION_WIDTH = 0.5  # m/s
COLORS = [
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
]

df = pd.read_csv("/Users/antoineroux/Downloads/san_mateo_2025-07-20/fitted_circles.csv")


# Extract the relevant columns
radius_df = df["radius"]
steering_angle_df = df["mean_steering_cmd"]
steering_angle_diff_df = abs(df["mean_steering_cmd"] - 90)
speed_df = df["mean_speed"]
steering_fbk_df = df["mean_steering_fbk"]

print(len(radius_df), len(steering_angle_diff_df), len(speed_df))

PLOT_BASICS = False

if PLOT_BASICS:
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

    # Plot steering feedback vs radius, colored by speed
    plt.scatter(
        steering_fbk_df,
        radius_df,
        c=speed_df,
        cmap="viridis",
        alpha=0.8,
        label="Colored by speed",
        vmin=2.5,
        vmax=8,
    )
    plt.colorbar(label="Speed (m/s)")
    plt.xlabel("Steering feedback")
    plt.ylabel("Radius (m)")
    plt.title("Radius vs Steering Feedback (Colored by Speed)")
    plt.grid(True)
    plt.show()

    STEERING_FBK_IDLE = 335  # measured by eye

    # Plot steering feedback diff vs radius, colored by speed
    plt.scatter(
        abs(steering_fbk_df - STEERING_FBK_IDLE),
        radius_df,
        c=speed_df,
        cmap="viridis",
        alpha=0.8,
        label="Colored by speed",
        vmin=2.5,
        vmax=8,
    )
    plt.colorbar(label="Speed (m/s)")
    plt.xlabel("Steering feedback diff (|fbk - idle|)")
    plt.ylabel("Radius (m)")
    plt.title("Radius vs Steering Feedback diff (Colored by Speed)")
    plt.grid(True)
    plt.show()


# Define the function to fit
def model_function(x_data, coeff):
    # radius = f(steering_diff, speed)
    steering_diff, speed = x_data
    return coeff * speed / (steering_diff)


x_data = (steering_angle_diff_df, speed_df)

# Perform the curve fitting
params, covariance = curve_fit(model_function, x_data, radius_df)

# Extract the fitted parameters
a = params
print(f"Fitted parameters on all data: a={a}")

# Generate fitted values for plotting
y_fit_real_data = model_function(x_data, a)
residuals_model_on_all_data = radius_df - y_fit_real_data
print(
    f"Mean residual on all data: {np.mean(abs(residuals_model_on_all_data)):.2f}m"  # noqa: E231
)
print(
    f"Mean relative residual on all data: {np.mean(abs(residuals_model_on_all_data) / radius_df):.2f}"  # noqa: E231
)


# Plot fitted model vs actual data using plotly
fig = go.Figure()

# Actual data points colored by speed
fig.add_trace(
    go.Scatter(
        x=steering_angle_diff_df,
        y=radius_df,
        mode="markers",
        marker=dict(
            color=speed_df,
            colorscale="Viridis",
            cmin=2.5,
            cmax=8,
            colorbar=dict(title="Speed (m/s)"),
            opacity=0.8,
        ),
        name="Actual Data",
    )
)

# Fitted data points
fig.add_trace(
    go.Scatter(
        x=steering_angle_diff_df,
        y=y_fit_real_data,
        mode="markers",
        marker=dict(
            color=speed_df,
            colorscale="Viridis",
            cmin=2.5,
            cmax=8,
            symbol="x",
            opacity=0.8,
        ),
        name="Fitted Data",
    )
)


fig.update_layout(
    title="Fitted Model: Radius vs Steering Feedback diff",
    xaxis_title="Steering feedback diff (|fbk - idle|)",
    yaxis_title="Radius (m)",
    showlegend=True,
)
fig.show()


# experiment with sweep

considered_speeds = [2, 3, 4, 5, 6, 7]
residual_model_on_data_at_speed = {}

for considered_speed in considered_speeds:
    speed_mask = (df["mean_speed"] >= considered_speed - SPEED_REGION_WIDTH) & (
        df["mean_speed"] <= considered_speed + SPEED_REGION_WIDTH
    )
    df_at_speed = df[speed_mask]

    steering_angle_diff_df_at_speed = abs(df_at_speed["mean_steering_cmd"] - 90)
    speed_df_at_speed = df_at_speed["mean_speed"]
    radius_df_at_speed = df_at_speed["radius"]

    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=steering_angle_diff_df,
            y=radius_df,
            mode="markers",
            marker=dict(
                color="gray",
                cmin=2.5,
                cmax=8,
                colorbar=dict(title="Speed (m/s)"),
                opacity=0.3,
            ),
            name="Actual Data",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=steering_angle_diff_df_at_speed,
            y=radius_df_at_speed,
            mode="markers",
            marker=dict(
                color=speed_df_at_speed,
                colorscale="Viridis",
                cmin=2.5,
                cmax=8,
                colorbar=dict(title="Speed (m/s)"),
                opacity=1,
            ),
            name="Actual Data",
        )
    )

    x_data_at_speed = (steering_angle_diff_df_at_speed, speed_df_at_speed)

    # Perform the curve fitting
    params, covariance = curve_fit(model_function, x_data_at_speed, radius_df_at_speed)

    # Extract the fitted parameters
    a = params
    print()
    print(f"Fitted parameters for speed {considered_speed} m/s: a={a}")

    residual_model_on_data_at_speed[
        considered_speed
    ] = radius_df_at_speed - model_function(x_data_at_speed, a)
    print(
        f"Mean residual at {considered_speed} m/s: {np.mean(abs(residual_model_on_data_at_speed[considered_speed])):.2f}m"  # noqa: E231
    )
    print(
        f"Mean relative residual at {considered_speed} m/s: {np.mean(abs(residual_model_on_data_at_speed[considered_speed] / radius_df_at_speed)):.2f}"  # noqa: E231 E501
    )

    steering_diff_linspace = np.linspace(2, 25, 100)
    speeds_at_N_m_per_s = np.ones(100) * considered_speed
    y_fit_at_N_m_per_s = model_function(
        (steering_diff_linspace, speeds_at_N_m_per_s), a
    )

    PLOT_INDIVIDUAL_FITS = False
    if PLOT_INDIVIDUAL_FITS:
        # Model prediction at N m/s
        fig.add_trace(
            go.Scatter(
                x=steering_diff_linspace,
                y=y_fit_at_N_m_per_s,
                mode="markers",
                marker=dict(color="red", symbol="x", opacity=0.8),
                name="Model at N m/s",
            )
        )

        fig.update_xaxes(range=[0, 25])
        fig.show()


# Create subplots for residuals analysis

fig = make_subplots(
    rows=4,
    cols=1,
)

# Errors vs Radius
fig.add_trace(
    go.Scatter(
        x=radius_df,
        y=residuals_model_on_all_data,
        mode="markers",
        name="Errors vs Radius",
        marker=dict(color="blue", opacity=0.7),
    ),
    row=1,
    col=1,
)

# Errors (relative) vs Radius
fig.add_trace(
    go.Scatter(
        x=radius_df,
        y=residuals_model_on_all_data / radius_df,
        mode="markers",
        name="Errors (relative) vs Radius",
        marker=dict(color="blue", opacity=0.7),
    ),
    row=2,
    col=1,
)

# Errors vs Speed
fig.add_trace(
    go.Scatter(
        x=speed_df,
        y=residuals_model_on_all_data,
        mode="markers",
        name="Errors vs Speed",
        marker=dict(color="red", opacity=0.7),
    ),
    row=3,
    col=1,
)
for i, (considered_speed, residuals) in enumerate(
    residual_model_on_data_at_speed.items()
):
    speed_mask = (df["mean_speed"] >= considered_speed - SPEED_REGION_WIDTH) & (
        df["mean_speed"] <= considered_speed + SPEED_REGION_WIDTH
    )
    df_at_speed = df[speed_mask]

    color = COLORS[i % len(COLORS)]
    fig.add_trace(
        go.Scatter(
            x=df_at_speed["mean_speed"],
            y=residuals,
            mode="markers",
            name=f"Errors vs Speed (at {considered_speed} m/s)",
            marker=dict(color=color, opacity=0.7),
        ),
        row=3,
        col=1,
    )


# Errors (relative) vs Speed
fig.add_trace(
    go.Scatter(
        x=speed_df,
        y=residuals_model_on_all_data / radius_df,
        mode="markers",
        name="Errors (relative) vs Speed",
        marker=dict(color="red", opacity=0.7),
    ),
    row=4,
    col=1,
)
for i, (considered_speed, residuals) in enumerate(
    residual_model_on_data_at_speed.items()
):
    speed_mask = (df["mean_speed"] >= considered_speed - SPEED_REGION_WIDTH) & (
        df["mean_speed"] <= considered_speed + SPEED_REGION_WIDTH
    )
    df_at_speed = df[speed_mask]

    color = COLORS[i % len(COLORS)]
    fig.add_trace(
        go.Scatter(
            x=df_at_speed["mean_speed"],
            y=residuals / df_at_speed["radius"],
            mode="markers",
            name=f"Errors (relative) vs Speed (at {considered_speed} m/s)",
            marker=dict(color=color, opacity=0.7),
        ),
        row=4,
        col=1,
    )


fig.update_xaxes(title_text="Radius (m)", row=1, col=1)
fig.update_yaxes(title_text="Errors", row=1, col=1)
fig.update_xaxes(title_text="Speed (m/s)", row=2, col=1)
fig.update_yaxes(title_text="Errors (relative)", row=2, col=1)
fig.update_xaxes(title_text="Speed (m/s)", row=3, col=1)
fig.update_yaxes(title_text="Errors", row=3, col=1)
fig.update_xaxes(title_text="Speed (m/s)", row=4, col=1)
fig.update_yaxes(title_text="Errors (relative)", row=4, col=1)

fig.update_layout(
    title="Errors Analysis",
    showlegend=True,
)
fig.show()
