import math
import numpy as np


MIN_CURVATURE = 0.3
MAX_CURVATURE = 100000


class State:
    def __init__(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, angle=0):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.angle = angle


def compute_curvature(current_state: State, target_state: State, wheel_base: float):
    """
    Compute curvature of the circle passing by the current point and target point, tangent to the car direction

    Args:
        current_state: State reprenting the current state of the vehicle, with at least x, y and angle
        target state: State representing the target point, with at least x and y

    Returns:
        float: the curvature
    """

    car_vector = np.array(
        [
            wheel_base * np.cos(current_state.angle + np.pi / 2),
            wheel_base * np.sin(current_state.angle + np.pi / 2),
        ]
    )

    car_to_target_vector = np.array(
        [(target_state.x - current_state.x), target_state.y - current_state.y]
    )

    sign = np.sign(np.cross(car_vector, car_to_target_vector))

    numerator = sign * 2 * abs(compute_relative_x_offset(target_state, current_state))

    denominator = plane_distance(target_state, current_state) ** 2

    if denominator > 0.0001:
        return numerator / denominator
    else:
        if numerator > 0:
            return MIN_CURVATURE
        else:
            return MAX_CURVATURE


def compute_relative_x_offset(current_state: State, target_state: State):
    """
    TODO

    Args:
        current_state: State reprenting the current state of the vehicle, with at least x, y and angle
        target state: State representing the target point, with at least x and y

    Returns:
        float: TODO
    """
    diff_x = target_state.x - current_state.x
    diff_y = target_state.y - current_state.y
    yaw = current_state.angle

    relative_x = math.cos(yaw) * diff_x + math.sin(yaw) * diff_y

    return relative_x


def plane_distance(current_state: State, target_state: State):
    """
    Returns the distance between two points on the xy plane

    Args:
        current_state: State reprenting the current state of the vehicle, with at least x, y and angle
        target state: State representing the target point, with at least x and y

    Returns:
        float: the distance
    """
    return math.sqrt(
        (target_state.x - current_state.x) ** 2
        + (target_state.y - current_state.y) ** 2
    )


def compute_steering_angle_from_curvature(curvature: float, wheel_base: float):
    """
    Compute the steering angle required to drive on a circle of a given curvature, assuming a bicycle model

    Args:
        curvature: the curvature of the circle to drive on
        wheel_base: wheel base of the vehicle

    Returns:
        float: the steering angle
    """

    return np.arctan(curvature * wheel_base)
