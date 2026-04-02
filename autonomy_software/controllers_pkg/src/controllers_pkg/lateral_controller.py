from enum import Enum, auto

import rospy
import tf
from dynamic_reconfigure.server import Server
from dynamic_reconfigure_pkg.cfg import lateral_controllerConfig
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from geometry_utils_pkg.geometry_utils import State, compute_curvature
from std_msgs.msg import Float32, Float64
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
            raise ValueError(f"Unknown controller type: {self.vehicle_model_type}")

        # Variables
        target_point_topic_name = rospy.get_param(
            "~target_point_topic_name", "target_point"
        )
        current_pose_topic_name = rospy.get_param(
            "~current_pose_topic_name", "current_pose"
        )
        steering_pwm_cmd_topic_name = rospy.get_param(
            "~steering_pwm_cmd_topic_name", "steering_pwm_cmd"
        )
        self.current_state = State()
        self.target_point_state = None
        self.rate = rospy.Rate(rate)
        self.current_velocity = 0

        # Subscribers
        rospy.Subscriber(
            target_point_topic_name,
            PointStamped,
            self.target_point_callback,
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
        self.target_curvature_pub = rospy.Publisher(
            "target_curvature",
            Float64,
            queue_size=10,
        )

    def dynamic_reconfigure_callback(
        self, config, level
    ):  # pylint: disable=unused-argument
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

    def target_point_callback(self, msg: PointStamped):
        """
        Callback to receive the target point from the target generator
        """
        self.target_point_state = State(x=msg.point.x, y=msg.point.y)

    def callback_current_velocity(self, twist_msg: TwistStamped):
        """
        Store current velocity
        """
        self.current_velocity = twist_msg.twist.linear.x

    def loop(self):
        """
        Main loop of the lateral controller
        Computes the steering_pwm_cmd based on the chosen car model and publishes it to the topic
        """

        while not rospy.is_shutdown():

            if self.target_point_state is None:
                print("No target point received yet, waiting...")
                self.rate.sleep()
                continue

            # Compute curvature from current pose and target point
            curvature = compute_curvature(
                current_state=self.current_state,
                target_state=self.target_point_state,
            )

            # Publish curvature for debug / introspection
            curvature_msg = Float64()
            curvature_msg.data = curvature
            self.target_curvature_pub.publish(curvature_msg)

            # Compute the commands based on the controller
            if abs(curvature) < 1e-3:
                steering_command_pwm = self.STEERING_IDLE_PWM
            else:
                steering_command_pwm = (
                    self.vehicle_model.compute_steering_command_from_radius(
                        radius=1 / curvature,
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
