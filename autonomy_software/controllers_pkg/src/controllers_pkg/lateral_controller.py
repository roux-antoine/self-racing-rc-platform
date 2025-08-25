from geometry_msgs.msg import PoseStamped, TwistStamped
from enum import Enum, auto
import rospy
from std_msgs.msg import Float32, Float64
import tf


from geometry_utils_pkg.geometry_utils import State

from dynamic_reconfigure.server import Server

from dynamic_reconfigure_pkg.cfg import lateral_controllerConfig
from vehicle_models_pkg.vehicle_models import CarModelBicycleV1, CarModelBicycleV2


class LateralControllerType(Enum):
    CarModelBicycleV1 = auto()
    CarModelBicycleV2 = auto()


class LateralController:
    def __init__(self):

        # Constants
        rate = rospy.get_param("~rate", 10.0)

        self.vehicle_model_type = LateralControllerType.CarModelBicycleV2

        if self.vehicle_model_type == LateralControllerType.CarModelBicycleV1:
            self.vehicle_model = CarModelBicycleV1()
        elif self.vehicle_model_type == LateralControllerType.CarModelBicycleV2:
            self.vehicle_model = CarModelBicycleV2()
        else:
            raise ValueError(f"Unknown controller type: {self.controller_type}")

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

        self.dynamic_reconfigure_server = Server(
            lateral_controllerConfig,
            self.dynamic_reconfigure_callback,
        )

        # Publishers
        self.steering_cmd_pub = rospy.Publisher(
            steering_pwm_cmd_topic_name,
            Float32,
            queue_size=10,
        )

    def dynamic_reconfigure_callback(self, config, level):
        self.STEERING_IDLE_PWM = config["steering_idle_pwm"]
        self.STEERING_MAX_PWM = config["steering_max_pwm"]
        self.STEERING_MIN_PWM = config["steering_min_pwm"]
        return config

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
        Computes the steering_pwm_cmd based on the chosen car model and publishes it to the topic
        """

        while not rospy.is_shutdown():

            # Compute the commands based on the controller
            if self.target_curvature == 0:
                steering_command_pwm = self.STEERING_IDLE_PWM
            else:
                steering_command_pwm = (
                    self.vehicle_model.compute_steering_command_from_radius(
                        radius=1 / self.target_curvature,
                        speed=self.current_velocity,
                    )
                )

            # Sanity checking
            if steering_command_pwm > self.STEERING_MAX_PWM:
                steering_command_pwm = self.STEERING_MAX_PWM
            elif steering_command_pwm < self.STEERING_MIN_PWM:
                steering_command_pwm = self.STEERING_MIN_PWM

            # Publish steering output
            self.steering_cmd_pub.publish(steering_command_pwm)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("lateral_controller")
        lateral_controller = LateralController()
        # rospy.spin()
        lateral_controller.loop()
    except rospy.ROSInterruptException:
        pass
