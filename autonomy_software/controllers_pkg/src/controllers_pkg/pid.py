#!/usr/bin/python3
import rospy
from typing import Optional


class PID:
    def __init__(
        self,
        gain_p: Optional[float] = None,
        gain_i: Optional[float] = None,
        gain_d: Optional[float] = None,
    ):

        # two modes to get the parameters:
        # - if they are passed as arguments, set them so and don't allow to change them after the fact
        # - if not, they are set to 0 and will be retrieved using dynamic reconfigure
        if gain_p or gain_i or gain_d:
            self.allow_dynamic_reconfigure = False
            self._kp = gain_p or 0.0
            self._ki = gain_i or 0.0
            self._kd = gain_d or 0.0
        else:
            self.allow_dynamic_reconfigure = True
            self._kp = 0.0
            self._ki = 0.0
            self._kd = 0.0

        self.min_sampling_time_sec = 0.01  # 10 ms
        self.output = 0
        self.i_value = 0
        self.previous_error = 0

    @property
    def kp(self):
        return self._kp

    @kp.setter
    def kp(self, value):
        if self.allow_dynamic_reconfigure:
            self._kp = value
        else:
            raise Exception("You are not allowed to modify the value of kp")

    @property
    def ki(self):
        return self._ki

    @ki.setter
    def ki(self, value):
        if self.allow_dynamic_reconfigure:
            self._ki = value
        else:
            raise Exception("You are not allowed to modify the value of ki")

    @property
    def kd(self):
        return self._kd

    @kd.setter
    def kd(self, value):
        if self.allow_dynamic_reconfigure:
            self._kd = value
        else:
            raise Exception("You are not allowed to modify the value of kd")

    def update(self, error: float, activate_anti_windup: bool, dt: float):
        """
        Function to calculate the controller's output given a set error.
        An anti-windup strategy is implemented to avoid increasing the integral value if needed
        Arguments:
        - error [m/s]: Desired value - current value
        - anti_windup: Boolean to enable the anti-windup strategy
        - dt [s]: Time since last call
        """

        if dt <= self.min_sampling_time_sec:
            rospy.logwarn(
                "Input sampling time is {}, which is smaller than min of {}.".format(
                    dt, self.min_sampling_time_sec
                )
            )

        # Proportional component
        output_p = error

        # Integral component - only increase if anti_windup not enabled
        if activate_anti_windup:
            output_i = self.i_value
        else:
            # Rectangular integration
            output_i = self.i_value + error * dt

        # Derivative component
        output_d = (error - self.previous_error) / dt

        # Sum all the components
        output = self._kp * output_p + self._ki * output_i + self._kd * output_d

        self.i_value = output_i
        self.previous_error = error

        return output, self._kp * output_p, self._ki * output_i, self._kd * output_d

    def reset(self):
        """
        Function to reset the integral value
        """
        self.i_value = 0
