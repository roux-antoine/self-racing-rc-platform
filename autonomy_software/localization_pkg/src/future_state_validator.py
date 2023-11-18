#!/usr/bin/python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np

bag = rosbag.Bag("/home/antoine/bags/future_validator_2023-11-17-05-53-09.bag")


current_poses = []
future_poses = []

for topic, msg, t in bag.read_messages(topics=["/current_pose", "/future_pose"]):
    if topic == "/current_pose":
        current_poses.append(msg)
    elif topic == "/future_pose":
        future_poses.append(msg)

errors = []
fig = plt.Figure()
for current_pose, future_pose in zip(current_poses[2:-1], future_poses[1:-2]):
    plt.scatter(current_pose.pose.position.x, current_pose.pose.position.y, c="green")
    plt.scatter(future_pose.pose.position.x, future_pose.pose.position.y, c="red")
    plt.plot(
        [current_pose.pose.position.x, future_pose.pose.position.x],
        [current_pose.pose.position.y, future_pose.pose.position.y],
        color="black",
    )

    error = np.linalg.norm(
        [
            future_pose.pose.position.x - current_pose.pose.position.x,
            future_pose.pose.position.y - current_pose.pose.position.y,
        ]
    )
    errors.append(error)
plt.show()

print(np.mean(errors))
plt.hist(errors, bins=50)
plt.show()
