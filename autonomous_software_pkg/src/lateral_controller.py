from geometry_msgs.msg import PoseStamped
import numpy as np
import rospy
from std_msgs.msg import Float32, Float64
import tf


from geometry_utils.geometry_utils import (
    State,
    compute_steering_angle_from_curvature,
)


class LateralController:
    def __init__(self):

        # Constants
        # TODO these values will be shared by many nodes -> use a rosparam
        self.STEERING_IDLE_PWM = 98  # unitless
        self.STEERING_MAX_PWM = 123  # unitless
        self.STEERING_MIN_PWM = 68  # unitless
        self.EFFECTIVE_MAX_STEERING_ANGLE = 0.3  # rad
        self.PWM_DIFFERENCE_AT_EFFECTIVE_MAX_STEERING_ANGLE = 27  # unitless
        # NOTE because the diff between IDLE and MAX (25) is not the same as IDLE and MIN (30),
        # we pick this 'average' value of 27. It would be smarter to have 2 values of the width
        # to the MAX and MIN.
        self.CAR_MIN_TURN_RADIUS = 1.25  # m
        self.WHEEL_BASE = 0.406  # m
        self.STEERING_REVERSE = (
            -1
        )  # unitless, here because the car turns left for a steering angle smaller than the IDLE

        # Variables
        target_curvature_topic_name = rospy.get_param(
            "~target_curvature_topic_name", "target_curvature"
        )
        current_pose_topic_name = rospy.get_param(
            "~current_pose_topic_name", "current_pose"
        )
        steering_pwm_cmd_topic_name = rospy.get_param(
            "~steering_pwm_cmd_topic_name", "steering_pwm_cmd"
        )
        self.current_state = State()

        # Subscribers
        rospy.Subscriber(
            target_curvature_topic_name,
            Float64,
            self.target_curvature_callback,
        )
        rospy.Subscriber(
            current_pose_topic_name,
            PoseStamped,
            self.current_pose_callback,
        )

        # Publishers
        self.steering_cmd_pub = rospy.Publisher(
            steering_pwm_cmd_topic_name,
            Float32,
            queue_size=10,
        )

    def current_pose_callback(self, msg: PoseStamped):
        """
        Retrieve the data from the /current_pose topic and stores it in the class variable
        """
        self.current_state.x = msg.pose.position.x
        self.current_state.y = msg.pose.position.y
        self.current_state.z = msg.pose.position.z

        orientation_list = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        if yaw <= 0 and yaw >= -np.pi:  # yaw in [-pi, 0]
            self.current_state.angle = yaw
        elif yaw > 0 and yaw <= np.pi:  # yaw in ]0, pi]
            self.current_state.angle = yaw - 2 * np.pi

    def target_curvature_callback(self, msg: Float64):
        """
        Computes the steering_pwm_cmd based on the target curvature and publishes it to the topic

        Using a very simple model, where the car (wheelbase=40.6cm) turns on a circle of diameter 2.5m when at 27 PWM units away from neutral,
        hence has a 'effective' angle of 0.3 rad at 27 PWM units away from neutral
        """

        # Compute the corresponding steering angle
        steering_angle = compute_steering_angle_from_curvature(
            curvature=msg.data, wheel_base=self.WHEEL_BASE
        )

        # Compute the steering_pwn_cmd based on the steering angle and our steering model
        if steering_angle > self.EFFECTIVE_MAX_STEERING_ANGLE:
            steering_pwn_cmd = self.STEERING_MIN_PWM
        elif steering_angle < -self.EFFECTIVE_MAX_STEERING_ANGLE:
            steering_pwn_cmd = self.STEERING_MAX_PWM
        else:
            steering_pwn_cmd = (
                self.STEERING_IDLE_PWM
                + self.STEERING_REVERSE
                * steering_angle
                * self.PWM_DIFFERENCE_AT_EFFECTIVE_MAX_STEERING_ANGLE
                / self.EFFECTIVE_MAX_STEERING_ANGLE
            )

        self.steering_cmd_pub.publish(steering_pwn_cmd)


if __name__ == "__main__":
    try:
        rospy.init_node("lateral_controller")
        lateral_controller = LateralController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
