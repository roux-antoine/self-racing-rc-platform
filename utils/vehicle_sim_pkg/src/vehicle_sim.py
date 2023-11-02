#!/usr/bin/python3
import numpy as np
from enum import Enum
from geometry_utils_pkg.geometry_utils import State


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
        wheelbase_m: float = 0.406,
        lateral_model: LateralModel = LateralModel.PERFECT,
        longitudinal_model: LongitudinalModel = LongitudinalModel.ACC_P_CONTROl,
        max_steering_rad: float = np.pi / 4,
        desired_acc_gain_p: float = 0.5,
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

        Arguments:
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
