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
        self.UPPER_BOUND_REGION_1 = 1.5  # m/s
        self.UPPER_BOUND_REGION_2 = 5  # m/s
        self.UPPER_BOUND_REGION_3 = 8  # m/s
        self.COEFF_REGION_1 = 27 * 1.25  # max steering_diff * radius of circle at max lateral acceleration
        self.COEFF_REGION_2 = 24 * 2.3
        self.COEFF_REGION_3 = 26 * 4

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
            if self.current_velocity == 0:
                coeff = 1000  # just so that we get a small number later on
            elif self.current_velocity > 0 and self.current_velocity <= self.UPPER_BOUND_REGION_1:
                coeff = 1 / (
                    self.COEFF_REGION_1
                )
            elif self.current_velocity > self.UPPER_BOUND_REGION_1 and self.current_velocity <= self.UPPER_BOUND_REGION_2:
                coeff = 1 / (
                    self.COEFF_REGION_1
                    + (self.current_velocity - self.UPPER_BOUND_REGION_1)
                    * (self.COEFF_REGION_2 - self.COEFF_REGION_1)
                    / (self.UPPER_BOUND_REGION_2 - self.UPPER_BOUND_REGION_1)
                )
            elif self.current_velocity > self.UPPER_BOUND_REGION_2 and self.current_velocity <= self.UPPER_BOUND_REGION_3:
                coeff = 1 / (
                    self.COEFF_REGION_2
                    + (self.current_velocity - self.UPPER_BOUND_REGION_2) * (self.COEFF_REGION_3 - self.COEFF_REGION_2) / (self.UPPER_BOUND_REGION_3 - self.UPPER_BOUND_REGION_2)
                )
            elif self.current_velocity > self.UPPER_BOUND_REGION_3:
                coeff = 1 / self.COEFF_REGION_3

            steering_pwn_cmd = (
                self.STEERING_IDLE_PWM - self.target_curvature / coeff
            )

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
