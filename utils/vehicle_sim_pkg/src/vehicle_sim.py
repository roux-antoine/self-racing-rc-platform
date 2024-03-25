#!/usr/bin/python3
from geometry_utils_pkg.geometry_utils import State
import numpy as np


class LongitudinalControlModel:
    def __init__(
        self, tau_delay: float = 0.0, noise: float = 0.0, max_speed_mps: float = 10
    ):
        self.tau_delay = tau_delay
        self.noise = noise
        self.max_speed_mps = max_speed_mps

    def throttle_to_speed(self, throttle_pwm_cmd: int):
        """
        Function that returns the speed the vehicle reaches given a throttle pwm input.
        This is an big assumption of course, not close to what happens in real life.
        Arguments:
            - throttle_pwm_cmd [pwm]: Throttle input
        """
        speed = 0.54 * throttle_pwm_cmd - 51.1

        if speed < 0:
            speed = 0

        return speed

    def update_speed(self, current_speed, throttle_pwm_cmd, dt):
        """
        Function that updates the simulated vehicle's speed
        Arguments:
            - current_speed [m/s]: Simulated vehicle's current speed
            - throttle_pwm_cmd [pwm]: Throttle input
            - dt [s]: Timestamp difference between two simulated speeds
        """

        speed_feedforward = self.throttle_to_speed(throttle_pwm_cmd)

        # Add delay
        if self.tau_delay != 0:
            next_speed = current_speed + (dt / self.tau_delay) * (
                speed_feedforward - current_speed
            )
        else:
            next_speed = speed_feedforward

        # Add noise
        if self.noise != 0:
            pass

        return next_speed

    def update_tau_delay(self, input_tau):
        self.tau_delay = input_tau

    def update_noise(self, input_noise):
        self.noise = input_noise


class LateralControlModel:
    def __init__(self, wheelbase: float = 0.406):
        self.wheelbase = wheelbase

    def update_position(self, current_state: State, steering_input, dt):
        """
        Function that updates the simulated vehicle's position and orientation using a bicycle model.

        Arguments:
            - current_state: Simulate vehicle's current state
            - steering_input [rad]: Steering input to apply to simulated lateral control model
            - dt [s]: Timestamp difference between two simulated positions
        """

        new_x = current_state.x + current_state.vx * np.cos(current_state.angle) * dt

        new_y = current_state.y + current_state.vx * np.sin(current_state.angle) * dt

        new_angle = (
            current_state.angle
            + (current_state.vx / self.wheelbase) * np.tan(steering_input) * dt
        )

        return new_x, new_y, new_angle

    def update_wheelbase(self, input_wheelbase):
        self.wheelbase = input_wheelbase


class VehicleSim:
    def __init__(
        self,
        lateral_model: LateralControlModel,
        longitudinal_model: LongitudinalControlModel,
        initial_state: State = State(),
    ):
        self.lateral_model = lateral_model
        self.longitudinal_model = longitudinal_model
        self.current_state = initial_state

    def update_position(self, steering_input: float, dt: float) -> None:
        """
        Function that updates the simulated vehicle's position and orientation using a bicycle model.
        Arguments:
            - steering_input [rad]: Steering input to apply to simulated lateral control model
            - dt [s]: Timestamp difference between two simulated positions
        """

        (
            self.current_state.x,
            self.current_state.y,
            self.current_state.angle,
        ) = self.lateral_model.update_position(self.current_state, steering_input, dt)

    def update_speed(self, throttle_pwm_cmd: int, dt: float):
        """
        Function to update the simulated vehicle's current speed.
        Arguments:
            - throttle_pwm_cmd [pwm]: Throttle input
            - dt [s]: Timestamp difference between two simulated speeds
        """

        self.current_state.vx = self.longitudinal_model.update_speed(
            self.current_state.vx, throttle_pwm_cmd, dt
        )

    def stop(self):
        """
        Function that updates the simulated vehicle's state to simulate a stop
        """

        self.current_state.vx = 0
