import glob
import os

import matplotlib.pyplot as plt
import numpy as np

flist = glob.glob(os.path.join("x_y_files", "*.txt"))
print("Available txt: ")
for path in flist:
    print(path)
print()

input_name_waypoints = input("Enter name for the files with waypoints: ")
input_name_edges = input("Enter name for the files with edges: ")

# Reading the waypoints
with open(input_name_waypoints) as waypoints_file:
    waypoints_xs_ys = [
        [float(line.split()[0]), float(line.split()[1])]
        for line in waypoints_file.readlines()
    ]

waypoints_xs = np.array(np.array(waypoints_xs_ys)[:, 0])
waypoints_ys = np.array(np.array(waypoints_xs_ys)[:, 1])

# Reading the edges
# Given that there might be multiple blocs of edges in the txt, we will store each block in a list
edges_xs_list = []  # this will be a list of list of x positions
edges_ys_list = []  # this will be a list of list of y positions
tmp_xs = []
tmp_ys = []

with open(input_name_edges) as edges_file:
    for line in edges_file:
        if line != "\n":
            tmp_xs.append(float(line.split()[0]))
            tmp_ys.append(float(line.split()[1]))
        else:
            edges_xs_list.append(tmp_xs)
            edges_ys_list.append(tmp_ys)
            tmp_xs = []
            tmp_ys = []
        edges_xs_list.append(tmp_xs)
        edges_ys_list.append(tmp_ys)

# Plotting things
plt.scatter(waypoints_xs, waypoints_ys)
for id, x, y in zip(range(len(waypoints_xs)), waypoints_xs, waypoints_ys):
    plt.annotate(id, (x, y))
for edges_xs, edges_ys in zip(edges_xs_list, edges_ys_list):
    plt.plot(edges_xs, edges_ys)

plt.axis("equal")
plt.show()
