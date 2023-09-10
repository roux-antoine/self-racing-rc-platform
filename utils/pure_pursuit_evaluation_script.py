import math
import matplotlib.pyplot as plt
import rosbag


def distance_to_line(point, line_start, line_end):
    # Calculate the vector from the line start to the point
    vector1 = [point[0] - line_start[0], point[1] - line_start[1]]
    # Calculate the vector along the line
    vector2 = [line_end[0] - line_start[0], line_end[1] - line_start[1]]

    # Calculate the dot product of the two vectors
    dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]

    # Calculate the length of the line segment
    line_length = math.sqrt(vector2[0] ** 2 + vector2[1] ** 2)

    # Calculate the projection of vector1 onto vector2
    projection = dot_product / line_length

    if projection < 0:
        # If the projection is before the line start, distance is to line_start
        return math.sqrt(
            (point[0] - line_start[0]) ** 2 + (point[1] - line_start[1]) ** 2
        )
    elif projection > line_length:
        # If the projection is after the line end, distance is to line_end
        return math.sqrt((point[0] - line_end[0]) ** 2 + (point[1] - line_end[1]) ** 2)
    else:
        # If the projection is on the line segment, distance is perpendicular
        return abs(vector1[0] * vector2[1] - vector1[1] * vector2[0]) / line_length


def evaluate_pure_pursuit(waypoints, vehicle_locations):
    distances = []

    for loc in vehicle_locations:
        min_distance = float("inf")

        for i in range(len(waypoints) - 1):
            waypoint_start = waypoints[i]
            waypoint_end = waypoints[i + 1]

            dist = distance_to_line(loc, waypoint_start, waypoint_end)

            if dist < min_distance:
                min_distance = dist

        distances.append(min_distance)

    return distances


def load_waypoints(file):
    with open(file) as waypoints_file:
        waypoints_xs_ys = [
            [float(line.split()[0]), float(line.split()[1])]
            for line in waypoints_file.readlines()
        ]

    return waypoints_xs_ys


if __name__ == "__main__":

    waypoints = load_waypoints(
        "/home/antoine/workspace/self_racing_rc_platform_ws/src/utils/utm_map_generation/x_y_files/rex_manor_parking_lot_waypoints.txt"
    )

    vehicle_locations = []
    velocities = []
    with rosbag.Bag(
        "/home/antoine/bags/2023-09-09/antoine_and_mattia_branches_merged_parking_lot_2023-09-09-20-49-22.bag",
        "r",
    ) as bag:
        for topic, msg, _ in bag.read_messages():
            if topic == "/gps_pose":
                vehicle_locations.append([msg.pose.position.x, msg.pose.position.y])
            if topic == "/gps_info":
                velocities.append(msg.speed)

    distances = evaluate_pure_pursuit(waypoints, vehicle_locations)

    fig, axes = plt.subplots(nrows=3)
    axes[0].plot(distances)
    axes[1].plot(velocities)
    for waypoint in waypoints:
        axes[2].scatter(waypoint[0], waypoint[1], color="k")
    for i, vehicle_location in enumerate(vehicle_locations):
        if i % 100 == 0:
            plt.text(
                vehicle_location[0] + 0.5, vehicle_location[1] + 0.5, i, fontsize=12
            )
            axes[2].scatter(vehicle_location[0], vehicle_location[1], color="r")
        else:
            axes[2].scatter(vehicle_location[0], vehicle_location[1], color="b")

    plt.tight_layout()
    plt.show()
