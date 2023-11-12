from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy
from std_msgs.msg import Float32, Float64
import tf


from geometry_utils_pkg.geometry_utils import (
    State,
)


class LateralController:
    def __init__(self):

        # Constants
        # TODO these values will be shared by many nodes -> use a rosparam
        rate = rospy.get_param("~rate", 10.0)
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
        self.target_curvature = 0
        self.rate = rospy.Rate(rate)
        self.current_velocity = 0

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
        rospy.Subscriber(
            "current_velocity",
            TwistStamped,
            self.callback_current_velocity,
            queue_size=10,
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

        self.current_state.angle = yaw

    def target_curvature_callback(self, msg: Float64):
        """
        Callback function to retrieve the target curvature
        """

        self.target_curvature = msg.data

    def callback_current_velocity(self, twist_msg: TwistStamped):
        """ """
        self.current_velocity = twist_msg.twist.linear.x

    def loop(self):
        """
        Main loop of the lateral controller
        Computes the steering_pwm_cmd based on the target curvature and publishes it to the topic

        Using a very simple model, where the car (wheelbase=40.6cm) turns on a circle of diameter 2.5m when at 27 PWM units away from neutral,
        hence has a 'effective' angle of 0.3 rad at 27 PWM units away from neutral
        """

        while not rospy.is_shutdown():

            # Compute the steering pwm command using a data-derived model of the steering
            # our model: steering_diff = curvature / coeff
            # coeff = 1 / (max steering_diff * radius of circle at max lateral acceleration)
            # coeff at 1.5 m/s = 27 * 1.25
            # coeff at 4.5 m/s = 24 * 2.3
            # coeff at 8 ms = 26 * 6
            # we linearly interpolate in between

            if self.current_velocity == 0:
                # not too sure what to do here, figure out
                coeff = 1000

            elif self.current_velocity > 0 and self.current_velocity <= 1.5:
                coeff = 1 / (
                    27 * 1.25
                )  # max steering_diff * radius of circle at max lateral acceleration
            elif self.current_velocity > 1.5 and self.current_velocity <= 4.5:
                coeff = 1 / (
                    27 * 1.25
                    + (self.current_velocity - 1.5)
                    * (24 * 2.3 - 27 * 1.25)
                    / (4.5 - 1.5)
                )
            elif self.current_velocity > 4.5 and self.current_velocity <= 8:
                coeff = 1 / (
                    24 * 2.3
                    + (self.current_velocity - 4.5) * (26 * 4 - 24 * 2.3) / (8 - 4.5)
                )
            elif self.current_velocity > 8:
                coeff = 1 / (26 * 4)

            steering_pwn_cmd = (
                self.STEERING_IDLE_PWM - self.target_curvature / coeff
            )  # TODO maybe we need a minus here

            if steering_pwn_cmd > self.STEERING_MAX_PWM:
                steering_pwn_cmd = self.STEERING_MAX_PWM
            elif steering_pwn_cmd < self.STEERING_MIN_PWM:
                steering_pwn_cmd = self.STEERING_MIN_PWM

            self.steering_cmd_pub.publish(steering_pwn_cmd)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("lateral_controller")
        lateral_controller = LateralController()
        # rospy.spin()
        lateral_controller.loop()
    except rospy.ROSInterruptException:
        pass
