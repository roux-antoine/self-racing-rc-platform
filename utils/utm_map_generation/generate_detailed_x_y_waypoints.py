#!/usr/bin/python3
import sys

import matplotlib.pyplot as plt
import numpy as np

THRESHOLD = 3  # meters

wp_file = str(sys.argv[1])
# print( wp_file)
# wp_file = "/home/mattia/ws/src/utils/utm_map_generation/x_y_files/grattan_street_waypoints.txt"
# wp_file = "/home/mattia/Downloads/laguna_seca_track_waypoints.txt"


def load_waypoints2(file):

    with open(file) as waypoints_file:
        waypoints_xs_ys = [
            [float(line.split()[0]), float(line.split()[1])]
            for line in waypoints_file.readlines()
        ]
    return waypoints_xs_ys


def distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


wp_list = load_waypoints2(wp_file)

# print("In: ", wp_list)

# plot
for wp in wp_list:
    plt.plot(wp[0], wp[1], "o")
plt.grid()
plt.axis("equal")
plt.show()


id_wp = 0

while id_wp != len(wp_list) - 1:

    x, y, x_n, y_n = (
        wp_list[id_wp][0],
        wp_list[id_wp][1],
        wp_list[id_wp + 1][0],
        wp_list[id_wp + 1][1],
    )

    if distance(x, y, x_n, y_n) > THRESHOLD:

        x_mid = (x + x_n) / 2
        y_mid = (y + y_n) / 2

        wp_list.insert(id_wp + 1, [x_mid, y_mid])

    else:
        id_wp += 1

# plot
for wp in wp_list:
    plt.plot(wp[0], wp[1], "o")

plt.grid()
plt.axis("equal")
plt.show()

print(wp_file.split(".")[0])

file_out = open(str(wp_file.split(".")[0] + "_detail_" + str(THRESHOLD) + "m.txt"), "w")

for wp in wp_list:
    file_out.writelines(str(wp[0]) + " " + str(wp[1]) + "\n")
