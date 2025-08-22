#!/usr/bin/python3
import constants
import numpy as np
from enum import Enum
from geometry_utils_pkg.geometry_utils import State


# NOTE: I think we can get rid of this file altogether, now that we have the vehicle models implemented.
# We just need to port some of the logic in the from here into the vehicle models.


class LateralModel(Enum):
    PERFECT = 1
    DELAY = 2


class LongitudinalModel(Enum):
    PERFECT = 1
    DELAY = 2
    ACC_P_CONTROl = 3


class VehicleSim:
    def __init__(
        self,
        initial_state: State = State(),
        wheelbase_m: float = constants.WHEELBASE_M,
        lateral_model: LateralModel = LateralModel.PERFECT,
        longitudinal_model: LongitudinalModel = LongitudinalModel.ACC_P_CONTROl,
        max_steering_rad: float = constants.MAX_STEERING_ANGLE_RAD,
        desired_acc_gain_p: float = constants.DESIRED_ACC_GAIN_P,
    ):
        self.current_state = initial_state
        self.wheelbase_m = wheelbase_m
        self.lateral_model = lateral_model
        self.longitudinal_model = longitudinal_model
        self.max_steering_rad = max_steering_rad
        self.desired_acc_gain_p = desired_acc_gain_p

    def predict_next_pose(self, steering_input: float, dt: float) -> None:
        """
        Function that updates the simulated vehicle's position and orientation using a bicycle model.

        Args:
            - steering_input [rad]: Steering input to apply to simulated lateral control model
            - dt [s]: Timestamp difference between two simulated positions
        """

        if self.lateral_model == LateralModel.PERFECT:

            self.current_state.x = (
                self.current_state.x
                + self.current_state.vx * np.cos(self.current_state.angle) * dt
            )
            self.current_state.y = (
                self.current_state.y
                + self.current_state.vx * np.sin(self.current_state.angle) * dt
            )

            self.current_state.angle = (
                self.current_state.angle
                + (self.current_state.vx / self.wheelbase_m)
                * np.tan(steering_input)
                * dt
            )

        else:
            # TODO
            pass

    def predict_next_pose_from_steering_pwm_cmd(
        self, steering_pwm_cmd: float, dt: float
    ) -> None:
        """
        Function that updates the simulated vehicle's position and orientation using the current steering pwm command and the current velocity.
        Args:
            - steering_pwm_cmd [float]: Input steering pwm, coming from the lateral controller
            - dt [s]: Timestamp difference between two simulated positions
        """

        # Convert steering pwm command to
        steering_rad = self.steering_pwm_to_steering_rad(steering_pwm_cmd)

        self.current_state.x = (
            self.current_state.x
            + self.current_state.vx * np.cos(self.current_state.angle) * dt
        )
        self.current_state.y = (
            self.current_state.y
            + self.current_state.vx * np.sin(self.current_state.angle) * dt
        )

        self.current_state.angle = (
            self.current_state.angle
            + (self.current_state.vx / self.wheelbase_m) * np.tan(steering_rad) * dt
        )

    def predict_next_velocity_from_throttle_pwm_cmd(
        self, throttle_pwm_cmd: float, dt: float
    ) -> None:
        """
        Function that updates the simulated vehicle's velocity, based on:
            - A simplified PWM to target velocity model, assuming a flat surface
            - A first order lag behavior with a proportional controller, where the vehicle cannot instantaneously match the
            target velocity. Instead, it accelerates proportionally to the velocity error.
        Args:
            - throttle_pwm_cmd [float]: Input throttle pwm, coming from the speed controller
            - dt [s]: Timestamp difference between two simulated positions
        """

        target_velocity = self.throttle_pwm_to_velocity(throttle_pwm_cmd)

        if self.longitudinal_model == LongitudinalModel.ACC_P_CONTROl:

            desired_acc = self.desired_acc_gain_p * (
                target_velocity - self.current_state.vx
            )

            updated_speed = self.current_state.vx + desired_acc * dt

        elif self.longitudinal_model == LongitudinalModel.PERFECT:

            updated_speed = target_velocity

        # Lower bound
        if updated_speed < 0:
            updated_speed = 0

        self.current_state.vx = updated_speed

    def throttle_pwm_to_velocity(self, throttle_pwm_cmd: float):
        """
        Function that converts a given throttle pwm command into a target velocity. Assuming a flat surface.
        This is in theory the speed the vehicle will reach given this throttle input.
        Args:
            - throttle_pwm_cmd [float]: Throttle input coming from the speed controller
        """

        return (
            constants.THROTTLE_PWM_TO_VELOCITY_MPS_SLOPE * throttle_pwm_cmd
            + constants.THROTTLE_PWM_TO_VELOCITY_MPS_OFFSET
        )

    def predict_next_velocity(self, target_velocity: float, dt: float):
        """
        Function that updates the simulated vehicle's velocity, based on a simulated upper-level proportional controller
        (that calculates the desired acceleration) and a perfect lower-level controller (the actual acceleration is the desired acceleration)

        Arguments:
            - target_velocity [m/s]: Velocity the vehicle needs to follow
            - dt [s]: Timestamp difference between two simulated positions
        """

        if self.longitudinal_model == LongitudinalModel.ACC_P_CONTROl:

            desired_acc = self.desired_acc_gain_p * (
                target_velocity - self.current_state.vx
            )

            self.current_state.vx = self.current_state.vx + desired_acc * dt

        elif self.longitudinal_model == LongitudinalModel.PERFECT:
            self.current_state.vx = target_velocity

        else:
            # TODO
            pass

    def steering_pwm_to_steering_rad(self, steering_pwm: float):
        """
        Function that converts a given steering pwm command into an actual steering angle from the wheels.
        This is simplified because the left and right wheels have different angles, and because of the way this is calculated.
        This is calculated by setting the steering pwm command to a certain value, then measure the curvature and use it to
        calculate the steering angle using a simple bicycle model.
        """

        b = (
            constants.EFFECTIVE_MAX_STEERING_ANGLE_RAD_V1
            * constants.STEERING_PWM_IDLE
            / (constants.STEERING_PWM_IDLE - constants.STEERING_PWM_MIN)
        )

        return (
            -constants.EFFECTIVE_MAX_STEERING_ANGLE_RAD_V1
            / (constants.STEERING_PWM_IDLE - constants.STEERING_PWM_MIN)
        ) * steering_pwm + b
