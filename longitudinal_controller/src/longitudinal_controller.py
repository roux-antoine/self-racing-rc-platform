#!/usr/bin/python3

import numpy as np
import rospy
import time

from pid import PID
from geometry_msgs.msg import TwistStamped
from self_racing_car_msgs.msg import ArduinoLogging
from std_msgs.msg import Float32


class LongitudinalController:
    def __init__(self):

        # Constants
        rate = rospy.get_param("~rate", 10.0)
        self.throttle_idle_autonomous_pwm = rospy.get_param(
            "~throttle_idle_autonomous_pwm", 90
        )
        self.throttle_max_autonomous_pwm = rospy.get_param(
            "~throttle_max_autonomous_pwm", 102
        )
        self.throttle_min_autonomous_pwm = rospy.get_param(
            "~throttle_min_autonomous_pwm", 70
        )
        gain_p = rospy.get_param("~speed_control_gain_p", 0.5)
        gain_i = rospy.get_param("~speed_control_gain_i", 0)
        gain_d = rospy.get_param("~speed_control_gain_d", 0)

        # Subscribers
        rospy.Subscriber(
            "target_velocity",
            TwistStamped,
            self.target_velocity_callback,
        )
        rospy.Subscriber(
            "current_velocity",
            TwistStamped,
            self.current_velocity_callback,
        )
        rospy.Subscriber(
            "arduino_logging",
            ArduinoLogging,
            self.firmware_log_callback,
        )

        # Publishers
        self.throttle_cmd_pub = rospy.Publisher(
            "throttle_pwm_cmd",
            Float32,
            queue_size=10,
        )

        # Variables
        self.desired_velocity = 0
        self.current_velocity = 0
        self.anti_windup_enabled = True
        self.pid_controller = PID(gain_p, gain_i, gain_d)
        self.speed_controller_enabled = False
        self.previous_t = time.time()
        self.rate = rospy.Rate(rate)

    def current_velocity_callback(self, msg: TwistStamped):
        """
        Callback function for current velocity message
        """
        self.current_velocity = msg.twist.linear.x

    def firmware_log_callback(self, msg: ArduinoLogging):
        """
        Callback of log message from low level controller.
        """
        self.speed_controller_enabled = msg.engaged_mode
        # TODO: Add timer for safety?

    def target_velocity_callback(self, msg: TwistStamped):
        """
        Callback function for target velocity message. Main function of the controller.
        """
        self.desired_velocity = msg.twist.linear.x

    def loop(self):
        """
        Main loop of the longitudinal controller
        """

        while not rospy.is_shutdown():

            if self.speed_controller_enabled:

                # Compute sample time
                dt = time.time() - self.previous_t

                # Compute velocity error
                error = self.desired_velocity - self.current_velocity

                # Call the PID controller
                throttle_diff = self.pid_controller.update(
                    error, self.anti_windup_enabled, dt
                )

                # Add the controller's output value to the idle position
                throttle_value = self.throttle_idle_autonomous_pwm + throttle_diff

                #  --- Anti windup strategy ---

                # Check if throttle output is already saturating the actuator
                if (
                    throttle_value < self.throttle_min_autonomous_pwm
                    or throttle_value > self.throttle_max_autonomous_pwm
                ):
                    throttle_saturating = True
                else:
                    throttle_saturating = False

                # Check if controller is trying to saturate the actuator even more
                if np.sign(error) == np.sign(throttle_value):
                    controller_saturating = True
                else:
                    controller_saturating = False

                # If both above are true, then stop integrating.
                if throttle_saturating and controller_saturating:
                    self.anti_windup_enabled = True
                else:
                    self.anti_windup_enabled = False

                #  Publish result
                self.publish_throttle_cmd(throttle_value)

            else:
                # If the controller is not enabled,
                self.pid_controller.reset()
                self.previous_t = time.time()

            self.rate.sleep()

    def publish_throttle_cmd(self, throttle_value: float):
        """
        Function to publish on throttle topic
        """

        throttle_msg = Float32()
        throttle_msg.data = throttle_value

        self.throttle_cmd_pub.publish(throttle_msg)


if __name__ == "__main__":
    try:
        rospy.init_node("longitudinal_controller")
        longitudinal_controller = LongitudinalController()
        longitudinal_controller.loop()
    except rospy.ROSInterruptException:
        pass
