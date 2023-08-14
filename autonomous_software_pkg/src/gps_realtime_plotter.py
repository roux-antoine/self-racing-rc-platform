#!/usr/bin/python3

import numpy as np
import rospy
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from self_racing_car_msgs.msg import VehicleState

# WAYPOINTS_FILEPATH = "/home/antoine/workspace/catkin_ws/src/utils/utm_map_generation/x_y_files/rex_manor_parking_lot_waypoints.txt"
WAYPOINTS_FILEPATH = "/home/antoine/workspace/catkin_ws/src/utils/utm_map_generation/x_y_files/laguna_seca_track_waypoints_detail_3m.txt"
EDGES_FILEPATH = "/home/antoine/workspace/catkin_ws/src/utils/utm_map_generation/x_y_files/laguna_seca_cold_pit_all_edges.txt"


class RealTimePlotter:
    def __init__(self):
        self.last_lat = None
        self.last_lon = None
        self.last_point = [None, None]
        self.previous_points = []
        self.waypoints_xs, self.waypoints_ys = self.load_waypoints(WAYPOINTS_FILEPATH)
        self.edges_xs_list, self.edges_ys_list = self.load_edges(EDGES_FILEPATH)

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

    def load_waypoints(self, filename):

        with open(filename) as waypoints_file:
            waypoints_xs_ys = [
                [float(line.split()[0]), float(line.split()[1])]
                for line in waypoints_file.readlines()
            ]

        waypoints_xs = np.array(np.array(waypoints_xs_ys)[:, 0])
        waypoints_ys = np.array(np.array(waypoints_xs_ys)[:, 1])

        return waypoints_xs, waypoints_ys

    def load_edges(self, filename):
        edges_xs_list = []
        edges_ys_list = []
        tmp_xs = []
        tmp_ys = []

        with open(filename) as edges_file:

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

        return edges_xs_list, edges_ys_list

    def prepare_map(self):

        self.ax.scatter(self.waypoints_xs, self.waypoints_ys, color="k")

        # for id, x, y in zip(
        #     list(range(len(self.waypoints_xs))), self.waypoints_xs, self.waypoints_ys
        # ):
        # self.ax.annotate(id, (x, y))

        for edges_xs, edges_ys in zip(self.edges_xs_list, self.edges_ys_list):
            self.ax.plot(edges_xs, edges_ys)

        self.ax.grid()
        self.ax.axis("equal")

    def plotting_callback(self, msg):
        self.last_point = [msg.x, msg.y]

    def animate(self, i):
        if self.last_point not in self.previous_points:
            self.previous_points.append(self.last_point)
            plt.scatter(self.last_point[0], self.last_point[1])


if __name__ == "__main__":
    real_time_plotter = RealTimePlotter()
    real_time_plotter.prepare_map()
    rospy.init_node("real_time_plotter")
    rospy.Subscriber("vehicle_state", VehicleState, real_time_plotter.plotting_callback)
    ani = FuncAnimation(plt.gcf(), real_time_plotter.animate, interval=20)
    plt.show()
