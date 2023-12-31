import math
import numpy as np
from typing import List

MIN_CURVATURE = 0.3  # m
MAX_CURVATURE = 100000  # m


class State:
    def __init__(
        self,
        x: float = 0,
        y: float = 0,
        z: float = 0,
        vx: float = 0,
        vy: float = 0,
        vz: float = 0,
        angle: float = 0,
    ):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.angle = angle


def compute_curvature(current_state: State, target_state: State):
    """
    Compute the curvature of the circle passing through the current_state and the target_state, tangent to the direction of the car
    See the document pure_pursuit.pdf for the derivation

    Args:
        current_state: State representing the current state of the vehicle, with at least x, y and angle
        line_vector: List of the components of the vector defining the line direction

    Returns:
        float: the curvature
    """
    # Computing the norm of the curvature
    orthogonal_vector = [
        -np.sin(current_state.angle),
        np.cos(current_state.angle),
    ]
    d = distance_point_to_line(current_state, orthogonal_vector, target_state)
    D = plane_distance(current_state, target_state)

    curvature_norm = 2 * np.sqrt(D**2 - d**2) / D**2

    # Computing the sign of the curvature
    car_direction_vector = np.array(
        [
            np.cos(current_state.angle),
            np.sin(current_state.angle),
        ]
    )
    target_direction_vector = np.array(
        [target_state.x - current_state.x, target_state.y - current_state.y]
    )
    sign = np.sign(np.cross(car_direction_vector, target_direction_vector))

    return sign * curvature_norm


def distance_point_to_line(
    current_state: State, line_vector: List[int], target_state: State
):
    """
    Compute the distance between the target state and the line defined by the current state and the line_vector

    Args:
        current_state: State representing the current state of the vehicle, with at least x and y
        line_vector: List of the components of the vector defining the line direction
        target state: State representing the target point, with at least x and y

    Returns:
        float: the distance
    """
    current_state_point = np.array([current_state.x, current_state.y])
    target_state_point = np.array([target_state.x, target_state.y])
    line_vector = np.array(line_vector)

    target_to_current_vector = target_state_point - current_state_point

    distance = np.linalg.norm(
        (target_to_current_vector)
        - np.dot((target_to_current_vector), line_vector) * line_vector
    )
    return distance


def plane_distance(current_state: State, target_state: State):
    """
    Returns the distance between two points on the xy plane

    Args:
        current_state: State representing the current state of the vehicle, with at least x, y and angle
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


def circle_line_segment_intersection(
    circle_center: List[float],
    circle_radius: float,
    pt1: List[float],
    pt2: List[float],
    full_line: bool = True,
    tangent_tol: float = 1e-9,
):
    """Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

    :param circle_center: The (x, y) location of the circle center
    :param circle_radius: The radius of the circle
    :param pt1: The (x, y) location of the first point of the segment
    :param pt2: The (x, y) location of the second point of the segment
    :param full_line: True to find intersections along full line - not just in the segment.  False will just return intersections within the segment.
    :param tangent_tol: Numerical tolerance at which we decide the intersections are close enough to consider it a tangent
    :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at which the circle intercepts a line segment.

    Note: We follow: http://mathworld.wolfram.com/Circle-LineIntersection.html
    """

    (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx**2 + dy**2) ** 0.5
    big_d = x1 * y2 - x2 * y1
    discriminant = circle_radius**2 * dr**2 - big_d**2

    if discriminant < 0:  # No intersection between circle and line
        return []
    else:  # There may be 0, 1, or 2 intersections with the segment
        intersections = [
            (
                cx
                + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**0.5)
                / dr**2,
                cy + (-big_d * dx + sign * abs(dy) * discriminant**0.5) / dr**2,
            )
            for sign in ((1, -1) if dy < 0 else (-1, 1))
        ]
        # This makes sure the order along the segment is correct
        if not full_line:
            # If only considering the segment, filter out intersections that do not fall within the segment
            fraction_along_segment = [
                (xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy
                for xi, yi in intersections
            ]
            intersections = [
                pt
                for pt, frac in zip(intersections, fraction_along_segment)
                if 0 <= frac <= 1
            ]
        if len(intersections) == 2 and abs(discriminant) <= tangent_tol:
            # If line is tangent to circle, return just one point (as both intersections have same location)
            return [intersections[0]]
        else:
            return intersections
