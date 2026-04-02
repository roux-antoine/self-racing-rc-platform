#!/usr/bin/python3
import rosbag
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import numpy as np
import math
import utm
from tqdm import tqdm

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
future_poses_v4 = []
future_poses_v5 = []
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

            # ########### V1 ###########
            # very simple model: future position is the current one + the velocity vector
            future_pose_x = x_utm + speed_mps * math.cos(yaw_rad) * DELTA_TIME
            future_pose_y = y_utm + speed_mps * math.sin(yaw_rad) * DELTA_TIME
            future_poses_v1.append([future_pose_x, future_pose_y, yaw_rad])

            # ########### V2 ###########
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

            #  ########### V3 ###########
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

            # ########### V4 ###########
            # with the model using different regions

            STEERING_IDLE_PWM = 98  # unitless
            STEERING_MAX_PWM = 123  # unitless
            STEERING_MIN_PWM = 68  # unitless
            UPPER_BOUND_REGION_1 = 1.5  # m/s
            UPPER_BOUND_REGION_2 = 5  # m/s
            UPPER_BOUND_REGION_3 = 8  # m/s
            COEFF_REGION_1 = 27 * 1.25  # max steering_diff * radius of circle at max lateral acceleration
            COEFF_REGION_2 = 24 * 2.3
            COEFF_REGION_3 = 26 * 4
            
            steering_diff = last_steering_cmd - 98

            if speed_mps == 0:
                R = 10000  # big number 
            else:
                if speed_mps > 0 and speed_mps <= UPPER_BOUND_REGION_1:
                    coeff = COEFF_REGION_1
                    
                elif speed_mps > UPPER_BOUND_REGION_1 and speed_mps <= UPPER_BOUND_REGION_2:
                    coeff = COEFF_REGION_1 + (speed_mps - UPPER_BOUND_REGION_1) * (COEFF_REGION_2 - COEFF_REGION_1) / (UPPER_BOUND_REGION_2 - UPPER_BOUND_REGION_1)
                    
                elif speed_mps > UPPER_BOUND_REGION_2 and speed_mps <= UPPER_BOUND_REGION_3:
                    coeff = COEFF_REGION_2 + (speed_mps - UPPER_BOUND_REGION_2) * (COEFF_REGION_3 - COEFF_REGION_2) / (UPPER_BOUND_REGION_3 - UPPER_BOUND_REGION_2)
                    
                elif speed_mps > UPPER_BOUND_REGION_3:
                    coeff = COEFF_REGION_3

                R = coeff / steering_diff

            # computing the x and y coordinates of the future pose
            theta = speed_mps / abs(R) * DELTA_TIME
            phi = yaw_rad
            alpha = phi - np.pi / 2

            if R > 0:
                theta = -theta  # TODO wtf
                
            future_pose_x = x_utm + np.sign(R) * abs(R) * (np.cos(alpha) - np.cos(alpha + theta))
            future_pose_y = y_utm + np.sign(R) * abs(R) * (np.sin(alpha) - np.sin(alpha + theta))
            angle = theta + alpha + np.pi / 2

            future_poses_v4.append([future_pose_x, future_pose_y, angle])


            # ########### V4 ###########
            # with the model using different regions, with updated params

            STEERING_IDLE_PWM = 98  # unitless
            STEERING_MAX_PWM = 123  # unitless
            STEERING_MIN_PWM = 68  # unitless
            UPPER_BOUND_REGION_1_NEW = 4  # m/s
            UPPER_BOUND_REGION_2_NEW = 6.1  # m/s
            UPPER_BOUND_REGION_3_NEW = 7.1  # m/s
            UPPER_BOUND_REGION_4_NEW = 10.0  # m/s
            COEFF_REGION_1 = 32.8125
            COEFF_REGION_2 = 57.08
            COEFF_REGION_3 = 79.8
            COEFF_REGION_4 = 101.9875
            
            steering_diff = last_steering_cmd - 98

            if speed_mps == 0:
                R = 10000  # big number 
            else:
                if speed_mps > 0 and speed_mps <= UPPER_BOUND_REGION_1:
                    coeff = COEFF_REGION_1
                    
                elif speed_mps > UPPER_BOUND_REGION_1_NEW and speed_mps <= UPPER_BOUND_REGION_2_NEW:
                    coeff = COEFF_REGION_1 + (speed_mps - UPPER_BOUND_REGION_1_NEW) * (COEFF_REGION_2 - COEFF_REGION_1) / (UPPER_BOUND_REGION_2_NEW - UPPER_BOUND_REGION_1_NEW)
                    
                elif speed_mps > UPPER_BOUND_REGION_2_NEW and speed_mps <= UPPER_BOUND_REGION_3_NEW:
                    coeff = COEFF_REGION_2 + (speed_mps - UPPER_BOUND_REGION_2_NEW) * (COEFF_REGION_3 - COEFF_REGION_2) / (UPPER_BOUND_REGION_3_NEW - UPPER_BOUND_REGION_2_NEW)
                    
                elif speed_mps > UPPER_BOUND_REGION_3_NEW and speed_mps <= UPPER_BOUND_REGION_4_NEW:
                    coeff = COEFF_REGION_3 + (speed_mps - UPPER_BOUND_REGION_3_NEW) * (COEFF_REGION_4 - COEFF_REGION_3) / (UPPER_BOUND_REGION_4_NEW - UPPER_BOUND_REGION_3_NEW)
                elif speed_mps > UPPER_BOUND_REGION_4_NEW:
                    coeff = COEFF_REGION_4

                R = coeff / steering_diff

            # computing the x and y coordinates of the future pose
            theta = speed_mps / abs(R) * DELTA_TIME
            phi = yaw_rad
            alpha = phi - np.pi / 2

            if R > 0:
                theta = -theta  # TODO wtf
                
            future_pose_x = x_utm + np.sign(R) * abs(R) * (np.cos(alpha) - np.cos(alpha + theta))
            future_pose_y = y_utm + np.sign(R) * abs(R) * (np.sin(alpha) - np.sin(alpha + theta))
            angle = theta + alpha + np.pi / 2


            future_poses_v5.append([future_pose_x, future_pose_y, angle])

if PLOT_DEBUG:
    plt.axis("equal")
    plt.show()


# Just plotting the real positions to figure out if the track angle is weird
fig = go.Figure()

for current_pose in tqdm(current_poses):
    # Add position scatter point
    fig.add_trace(go.Scatter(x=[current_pose[0]], y=[current_pose[1]], 
                            mode='markers', marker=dict(color='green', size=6),
                            name='Position', showlegend=False))
    
    # Add orientation arrow
    arrow_end_x = current_pose[0] + 0.5 * np.cos(current_pose[2])
    arrow_end_y = current_pose[1] + 0.5 * np.sin(current_pose[2])
    fig.add_annotation(x=arrow_end_x, y=arrow_end_y, ax=current_pose[0], ay=current_pose[1],
                      xref='x', yref='y', axref='x', ayref='y',
                      arrowhead=2, arrowsize=1, arrowwidth=2, arrowcolor='blue')

fig.update_layout(title="Real Positions and Orientations", xaxis_title="UTM X", yaxis_title="UTM Y")
fig.update_yaxes(scaleanchor="x", scaleratio=1)
fig.show()

# plotting the predicted collisions and computing metrics

print(len(current_poses))

errors_xy_v1 = []
errors_angle_v1 = []
errors_xy_v2 = []
errors_angle_v2 = []
errors_xy_v3 = []
errors_angle_v3 = []
errors_xy_v4 = []
errors_angle_v4 = []
errors_xy_v5 = []
errors_angle_v5 = []

# Plotting the predicted positions and computing metrics
fig = go.Figure()

for i in tqdm(range(len(current_poses) - 1)):
    # Current positions (green)
    fig.add_trace(go.Scatter(x=[current_poses[i][0]], y=[current_poses[i][1]], 
                            mode='markers', marker=dict(color='green', size=6),
                            name='Current Position', showlegend=i==0))
    
    # Current orientation arrows (green)
    arrow_end_x = current_poses[i][0] + 0.5 * np.cos(current_poses[i][2])
    arrow_end_y = current_poses[i][1] + 0.5 * np.sin(current_poses[i][2])
    fig.add_annotation(x=arrow_end_x, y=arrow_end_y, 
                      ax=current_poses[i][0], ay=current_poses[i][1],
                      xref='x', yref='y', axref='x', ayref='y',
                      arrowhead=2, arrowsize=1, arrowwidth=2, arrowcolor='green')
    
    # Future positions V1 (pink)
    fig.add_trace(go.Scatter(x=[future_poses_v1[i][0]], y=[future_poses_v1[i][1]], 
                            mode='markers', marker=dict(color='pink', size=6),
                            name='Future Position V1', showlegend=i==0))

    # Future positions V2 (orange)
    fig.add_trace(go.Scatter(x=[future_poses_v2[i][0]], y=[future_poses_v2[i][1]], 
                            mode='markers', marker=dict(color='orange', size=6),
                            name='Future Position V2', showlegend=i==0))


    # Future orientation arrows V2 (orange)
    future_arrow_end_x = future_poses_v2[i][0] + 0.5 * np.cos(future_poses_v2[i][2])
    future_arrow_end_y = future_poses_v2[i][1] + 0.5 * np.sin(future_poses_v2[i][2])
    fig.add_annotation(x=future_arrow_end_x, y=future_arrow_end_y,
                      ax=future_poses_v2[i][0], ay=future_poses_v2[i][1],
                      xref='x', yref='y', axref='x', ayref='y',
                      arrowhead=2, arrowsize=1, arrowwidth=2, arrowcolor='orange')

    # Future positions V4 (blue)
    fig.add_trace(go.Scatter(x=[future_poses_v4[i][0]], y=[future_poses_v4[i][1]], 
                            mode='markers', marker=dict(color='blue', size=6),
                            name='Future Position V4', showlegend=i==0))
    
    # Future positions V5 (black)
    fig.add_trace(go.Scatter(x=[future_poses_v5[i][0]], y=[future_poses_v5[i][1]], 
                            mode='markers', marker=dict(color='black', size=6),
                            name='Future Position V5', showlegend=i==0))

    # Connection lines (gray)
    fig.add_trace(go.Scatter(x=[current_poses[i+1][0], future_poses_v1[i][0]],
                            y=[current_poses[i+1][1], future_poses_v1[i][1]], 
                            mode='lines', line=dict(color='gray', width=1),
                            name='Prediction Error', showlegend=i==0))
    fig.add_trace(go.Scatter(x=[current_poses[i+1][0], future_poses_v2[i][0]],
                            y=[current_poses[i+1][1], future_poses_v2[i][1]], 
                            mode='lines', line=dict(color='gray', width=1),
                            name='Prediction Error', showlegend=i==0))
    fig.add_trace(go.Scatter(x=[current_poses[i+1][0], future_poses_v4[i][0]],
                            y=[current_poses[i+1][1], future_poses_v4[i][1]], 
                            mode='lines', line=dict(color='gray', width=1),
                            name='Prediction Error', showlegend=i==0))
    fig.add_trace(go.Scatter(x=[current_poses[i+1][0], future_poses_v5[i][0]],
                            y=[current_poses[i+1][1], future_poses_v5[i][1]], 
                            mode='lines', line=dict(color='gray', width=1),
                            name='Prediction Error', showlegend=i==0))

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

    error_xy_v4 = np.linalg.norm(
        [
            future_poses_v4[i][0] - current_poses[i + 1][0],
            future_poses_v4[i][1] - current_poses[i + 1][1],
        ]
    )
    errors_xy_v4.append(error_xy_v4)
    error_angle_v4 = np.abs(current_poses[i + 1][2] - future_poses_v4[i][2]) % 2 * np.pi
    errors_angle_v4.append(error_angle_v4)

    error_xy_v5 = np.linalg.norm(
        [
            future_poses_v5[i][0] - current_poses[i + 1][0],
            future_poses_v5[i][1] - current_poses[i + 1][1],
        ]
    )
    errors_xy_v5.append(error_xy_v5)
    error_angle_v5 = np.abs(current_poses[i + 1][2] - future_poses_v5[i][2]) % 2 * np.pi
    errors_angle_v5.append(error_angle_v5)

fig.update_layout(title="Future State Prediction Comparison", 
                  xaxis_title="UTM X", yaxis_title="UTM Y")
fig.update_yaxes(scaleanchor="x", scaleratio=1)
fig.show()

print(f"errors_xy_v1: {np.mean(errors_xy_v1)}")
print(f"errors_xy_v2: {np.mean(errors_xy_v2)}")
print(f"errors_xy_v3: {np.mean(errors_xy_v3)}")
print(f"errors_xy_v4: {np.mean(errors_xy_v4)}")
print(f"errors_xy_v5: {np.mean(errors_xy_v5)}")

print(f"errors_angle_v1: {np.mean(errors_angle_v1)}")
print(f"errors_angle_v2: {np.mean(errors_angle_v2)}")
print(f"errors_angle_v3: {np.mean(errors_angle_v3)}")
print(f"errors_angle_v4: {np.mean(errors_angle_v4)}")
print(f"errors_angle_v5: {np.mean(errors_angle_v5)}")

plt.hist(errors_xy_v1, bins=50, alpha=0.5, label="errors_xy_v1")
plt.hist(errors_xy_v2, bins=50, alpha=0.5, label="errors_xy_v2")
plt.hist(errors_xy_v3, bins=50, alpha=0.5, label="errors_xy_v3")
plt.hist(errors_xy_v4, bins=50, alpha=0.5, label="errors_xy_v4")
plt.hist(errors_xy_v5, bins=50, alpha=0.5, label="errors_xy_v5")
plt.legend()
plt.show()

plt.hist(errors_angle_v1, bins=50, alpha=0.5, label="errors_angle_v1")
plt.hist(errors_angle_v2, bins=50, alpha=0.5, label="errors_angle_v2")
plt.hist(errors_angle_v3, bins=50, alpha=0.5, label="errors_angle_v3")
plt.hist(errors_angle_v4, bins=50, alpha=0.5, label="errors_angle_v4")
plt.hist(errors_angle_v5, bins=50, alpha=0.5, label="errors_angle_v5")
plt.legend()
plt.show()


# NOTE
# there is no point trying to predict the heading until we use the "bad" heading from the gps
# we need a better way of determining the heading of the vehicle
# in fact it could even make sense of trying the current V2 algo, maybe the angle is correct enough
# the fact that the heading from the gps is so incorrect might also negtively impact the pure pursuit calculations (e.g. the circle we compute is far from the real circle)
# end NOTE
