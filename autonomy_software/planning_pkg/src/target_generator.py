#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from self_racing_car_msgs.msg import WaypointArray
from visualization_msgs.msg import Marker

from geometry_utils_pkg.geometry_utils import (
    State,
    plane_distance,
    circle_line_segment_intersection,
    compute_curvature,
)
from std_msgs.msg import Float64
import tf


from dynamic_reconfigure.server import Server

from dynamic_reconfigure_pkg.cfg import (
    target_generatorConfig,
)
from enum import Enum


"""
NOTE:
- Publish some additional things? Lookahead circle?
- Here we don't have the logic when the nextWaypoint is close to the starting line
"""


class VelocityModeEnum(Enum):
    MANUAL_INPUT_SPEED = 0
    WAYPOINT_FOLLOWING = 1


class TargetGenerator:
    def __init__(self):

        """Initialization"""
        self.current_state = State()
        self.waypoints = None
        self.user_input_speed_mps = 0
        self.id_closest_wp = 0

        """ Subscribers """
        rospy.Subscriber(
            "waypoints",
            WaypointArray,
            self.callback_waypoints,
            queue_size=10,
        )
        rospy.Subscriber(
            "current_pose",
            PoseStamped,
            self.callback_current_pose,
            queue_size=10,
        )
        rospy.Subscriber(
            "current_velocity",
            TwistStamped,
            self.callback_current_velocity,
            queue_size=10,
        )
        rospy.Subscriber(
            "manual_input_speed",
            TwistStamped,
            self.callback_manual_input_speed,
            queue_size=10,
        )

        """ Publishers """
        self.target_curvature_pub = rospy.Publisher(
            "target_curvature",
            Float64,
            queue_size=10,
        )
        self.target_velocity_pub = rospy.Publisher(
            "target_velocity",
            TwistStamped,
            queue_size=10,
        )
        # self.target_gen_debug_pub = rospy.Publisher("target_generator_debug", TBD, queue_size=10)
        self.target_point_marker_pub = rospy.Publisher(
            "target_point_marker",
            Marker,
            queue_size=10,
        )

        """ Dynamic reconfigure setup """
        self.dynamic_reconfigure_server = Server(
            target_generatorConfig,
            self.dynamic_reconfigure_callback,
        )

    def dynamic_reconfigure_callback(self, config, level):
        self.lookahead_distance = config["lookahead_distance"]
        self.curvature_min = config["curvature_min"]
        self.speed_scale_factor = config["speed_scale_factor"]
        self.loop_over_waypoints = config["loop_over_waypoints"]
        if config["velocity_mode"] == VelocityModeEnum.MANUAL_INPUT_SPEED.value:
            self.velocity_mode = VelocityModeEnum.MANUAL_INPUT_SPEED
        elif config["velocity_mode"] == VelocityModeEnum.WAYPOINT_FOLLOWING.value:
            self.velocity_mode = VelocityModeEnum.WAYPOINT_FOLLOWING
        else:
            raise ValueError("Invalid value for velocity_mode.")

        return config

    def callback_waypoints(self, wp_msg: WaypointArray):
        """
        Populates the waypoints attribute with the received waypoints

        Args:
            wp_msg [WaypointArray]: Array of waypoints to follow. [id, pose, speed_mps]
        """

        self.waypoints = wp_msg

    def callback_current_pose(self, pose_msg: PoseStamped):
        """
        Most important part of the node. Receives new pose and outputs target point and target velocity.

        Args:
            pose_msg [PoseStamped]: Current position and orientation of vehicle
        """

        if self.waypoints is None:

            rospy.logwarn(
                "Waypoints haven't been acquired yet. Can't process new pose."
            )

        else:

            """Read subscribed message"""
            self.current_state.x = pose_msg.pose.position.x
            self.current_state.y = pose_msg.pose.position.y

            orientation_list = [
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            ]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                orientation_list
            )

            self.current_state.angle = yaw

            """ Get Next waypoint - First waypoint further than the lookahead distance """
            nextWaypointId = self.getNextWaypoint()

            # If next waypoint is the last one and we are not looping over, no need to
            # try and find the target point. If we do, the target point will be behind us
            # and this is a potential hazard. Instead, set target point to last waypoint.
            # NOTE: This could theoretically be better if getNextWaypoint would publish something else when
            # all the waypoints are within the lookahead
            if nextWaypointId is None and self.loop_over_waypoints is False:

                rospy.logwarn("Next waypoint is the last one of the list. ")

                # Get target point - just for visualization
                targetPoint = State(
                    x=self.waypoints.waypoints[-1].pose.position.x,
                    y=self.waypoints.waypoints[-1].pose.position.y,
                )

                # Set curvature to minimum, so the steering is frozen to zero
                curvature = self.curvature_min

            # Nominal situation
            else:

                """Get target point"""
                targetPoint = self.getTargetPoint(nextWaypointId)

                """ Compute target curvature """
                curvature = compute_curvature(
                    current_state=self.current_state, target_state=targetPoint
                )

            """ Publish target point marker """
            self.publish_target_point_marker(targetPoint)

            """ Publish target curvature """
            self.publish_target_curvature(curvature)

            """ Compute target speed """
            if self.velocity_mode == VelocityModeEnum.MANUAL_INPUT_SPEED:
                target_speed = self.user_input_speed_mps
            elif self.velocity_mode == VelocityModeEnum.WAYPOINT_FOLLOWING:
                try:
                    if (
                        self.loop_over_waypoints is False
                        and self.id_closest_wp == self.waypoints.waypoints[-1].id
                    ):
                        target_speed = 0
                    else:
                        target_speed = (
                            self.speed_scale_factor
                            * self.waypoints.waypoints[self.id_closest_wp].speed_mps
                        )
                except IndexError:
                    rospy.logwarn(
                        "An error occured while retrieving speed from waypoint"
                    )
                    target_speed = 0

            """ Publish target speed """
            self.publish_target_speed(target_speed)

    def callback_current_velocity(self, twist_msg: TwistStamped):
        """ """
        pass

    def callback_manual_input_speed(self, twist_msg: TwistStamped):
        """
        Callback for manually generated input speed
        NOTE: Use a timer that we reset to make sure we receive it regularly?
        """
        self.user_input_speed_mps = twist_msg.twist.linear.x

    def getNextWaypoint(self):
        """
        Returns:
            The id of the first waypoint further than the lookahead distance and in the direction of the path
        """

        """ Get closest waypoint """
        d_min = 10000
        id_closest_wp = None

        for wp in self.waypoints.waypoints:
            distance = plane_distance(
                State(x=wp.pose.position.x, y=wp.pose.position.y), self.current_state
            )
            if distance < d_min:
                d_min = distance
                id_closest_wp = wp.id

        # Set attribute to use it for retrieving the speed of the closest waypoint
        # NOTE: Is this ideal?
        if id_closest_wp is not None:
            self.id_closest_wp = id_closest_wp

        if id_closest_wp is None:
            rospy.logfatal("No closest waypoint found")
            # TODO: Improve the logic
            return self.waypoints.waypoints[-1].id

        """ Iterate through all the waypoints starting from the closest + 1. Return the first further than lookahead """
        for wp in self.waypoints.waypoints[id_closest_wp + 1 :]:
            if (
                plane_distance(
                    State(x=wp.pose.position.x, y=wp.pose.position.y),
                    self.current_state,
                )
                > self.lookahead_distance
            ):
                return wp.id

        if self.loop_over_waypoints is True:
            rospy.logwarn("Looping over from beginning.")
            # If we are here, it means that all the waypoints behind the closest one are within the lookahead distance
            # We have to loop over the waypoints starting from the beginning
            for wp in self.waypoints.waypoints[:id_closest_wp]:
                if (
                    plane_distance(
                        State(x=wp.pose.position.x, y=wp.pose.position.y),
                        self.current_state,
                    )
                    > self.lookahead_distance
                ):
                    return wp.id

        rospy.logwarn("All waypoints considered are within the lookahead distance")
        return None

    def getTargetPoint(self, nextWaypointId: int):
        """
        Args:
            - nextWaypointId [int]: Id of first waypoint further than lookahead distance
        Returns:
            - target_point [State]: Point of intersection between lookahead circle and line that runs through nextWaypoint and 9nextWaypointId - 1)
        """

        """ Find intersections between lookahead circle and segment """
        intersections = circle_line_segment_intersection(
            circle_center=(self.current_state.x, self.current_state.y),
            circle_radius=self.lookahead_distance,
            pt1=(
                self.waypoints.waypoints[nextWaypointId].pose.position.x,
                self.waypoints.waypoints[nextWaypointId].pose.position.y,
            ),
            pt2=(
                self.waypoints.waypoints[nextWaypointId - 1].pose.position.x,
                self.waypoints.waypoints[nextWaypointId - 1].pose.position.y,
            ),
        )

        # 2 intersections
        if len(intersections) == 2:
            # Distance between NextWaypoint and first intersection
            d1 = plane_distance(
                State(
                    x=self.waypoints.waypoints[nextWaypointId].pose.position.x,
                    y=self.waypoints.waypoints[nextWaypointId].pose.position.y,
                ),
                State(x=intersections[0][0], y=intersections[0][1]),
            )
            # Distance between NextWaypoint and second intersection
            d2 = plane_distance(
                State(
                    x=self.waypoints.waypoints[nextWaypointId].pose.position.x,
                    y=self.waypoints.waypoints[nextWaypointId].pose.position.y,
                ),
                State(x=intersections[1][0], y=intersections[1][1]),
            )

            # We choose the intersection that is closer to the NextWaypoint
            if d1 < d2:
                target_point = State(x=intersections[0][0], y=intersections[0][1])
            else:
                target_point = State(x=intersections[1][0], y=intersections[1][1])

        # One intersection, the target point is the NextWaypoint
        elif len(intersections) == 1:
            target_point = State(
                x=self.waypoints.waypoints[nextWaypointId].pose.position.x,
                y=self.waypoints.waypoints[nextWaypointId].pose.position.y,
            )

        # No intersection, the target point is the NextWaypoint
        else:
            target_point = State(
                x=self.waypoints.waypoints[nextWaypointId].pose.position.x,
                y=self.waypoints.waypoints[nextWaypointId].pose.position.y,
            )

        """ Target point speed it NextWaypoint's speed """
        target_point.vx = self.waypoints.waypoints[nextWaypointId].speed_mps

        return target_point

    def publish_target_point_marker(self, targetPoint: State):
        """
        Publishes the targetPoint marker for visualization
        """

        marker_msg = Marker()

        marker_msg.header.frame_id = "world"
        marker_msg.ns = "map"
        marker_msg.type = 2
        marker_msg.action = 0
        marker_msg.id = 0

        # marker scale
        marker_msg.scale.x = 1.5
        marker_msg.scale.y = 1.5

        # marker color
        marker_msg.color.r = 1
        marker_msg.color.g = 0.1
        marker_msg.color.b = 0.1
        marker_msg.color.a = 1

        marker_msg.pose.position.x = targetPoint.x
        marker_msg.pose.position.y = targetPoint.y

        marker_msg.pose.orientation.w = 1.0

        self.target_point_marker_pub.publish(marker_msg)

    def publish_target_speed(self, target_speed: float):
        """
        Publishes the TwistStamped target speed
        """

        twist_msg = TwistStamped()

        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "world"

        twist_msg.twist.linear.x = target_speed

        self.target_velocity_pub.publish(twist_msg)

    def publish_target_curvature(self, curvature: float):
        """
        Publishes the Float64 target curvature
        """

        curvature_msg = Float64()
        curvature_msg.data = curvature

        self.target_curvature_pub.publish(curvature_msg)


if __name__ == "__main__":
    try:
        rospy.init_node("target_generator")
        target_gen = TargetGenerator()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
