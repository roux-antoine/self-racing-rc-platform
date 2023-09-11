from geometry_msgs.msg import TwistStamped
import rospy
from std_msgs.msg import Float32


class LongitudinalController:
    def __init__(self):

        # Constants
        # TODO these values will be shared by many nodes -> use a rosparam
        self.THROTTLE_IDLE_PWM = 90  # unitless
        self.THROTTLE_MAX_AUTONOMOUS_PWM = 102  # unitless
        self.THROTTLE_MIN_AUTONOMOUS_PWM = 70  # unitless

        # Variables
        target_velocity_topic_name = rospy.get_param(
            "~target_velocity_topic_name", "target_velocity"
        )
        throttle_pwm_cmd_topic_name = rospy.get_param(
            "~throttle_pwm_cmd_topic_name", "throttle_pwm_cmd"
        )

        # Subscribers
        rospy.Subscriber(
            target_velocity_topic_name,
            TwistStamped,
            self.target_point_callback,
        )

        # Publishers
        self.throttle_cmd_pub = rospy.Publisher(
            throttle_pwm_cmd_topic_name,
            Float32,
            queue_size=10,
        )

    def target_point_callback(self, msg: Float32):
        """
        The most basic controller for now: send a constant value
        """
        constant_throttle_pwm_cmd = self.THROTTLE_MAX_AUTONOMOUS_PWM
        self.throttle_cmd_pub.publish(constant_throttle_pwm_cmd)


if __name__ == "__main__":
    try:
        rospy.init_node("longitudinal_controller")
        longitudinal_controller = LongitudinalController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
