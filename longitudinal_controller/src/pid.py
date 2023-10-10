#!/usr/bin/python3


class PID:
    def __init__(
        self,
        gain_p: float = 0.0,
        gain_i: float = 0.0,
        gain_d: float = 0.0,
    ):

        self.kp = gain_p
        self.ki = gain_i
        self.kd = gain_d

        self.output = 0
        self.i_value = 0
        self.previous_error = 0

    def update(self, error: float, activate_anti_windup: bool, dt: float):
        """
        Function to calculate the controller's output given a set error.
        An anti-windup strategy is implemented to avoid increasing the integral value if needed
        Arguments:
        - error [m/s]: Desired value - current value
        - anti_windup: Boolean to enable the anti-windup strategy
        - dt [s]: Time since last call
        """

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
        output = self.kp * output_p + self.ki * output_i + self.kd * output_d

        self.i_value = output_i
        self.previous_error = error

        return output

    def reset(self):
        """
        Function to reset the integral value
        """
        self.i_value = 0
