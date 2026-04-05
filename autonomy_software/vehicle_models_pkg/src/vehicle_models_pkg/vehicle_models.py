from typing import List

import numpy as np
from geometry_utils_pkg.geometry_utils import State
from vehicle_models_pkg.vehicle_models_constants import (
    EFFECTIVE_MAX_STEERING_ANGLE_RAD_V1,
    PHYSICAL_MAX_STEERING_ANGLE_RAD,
    PWM_DIFF_AT_MAX_STEER_ANGLE,
    STEERING_DIRECTION_FACTOR,
    STEERING_IDLE_PWM,
    WHEELBASE,
)


class CarModelBase:
    """Base class for car models.

    Child classes shall implement the `step` method to update the vehicle state based on commands etc.
    """

    # radius to return when steering is at the idle position (in m)
    VERY_LARGE_RADIUS = 100000
    VERY_LARGE_COEFF = 100000

    def __init__(self) -> None:
        self.states: List[State] = []

    def init(self, x: float, y: float, vx: float, angle: float) -> None:
        self.states.append(
            State(
                x=x,
                y=y,
                z=None,
                vx=vx,
                vy=None,
                vz=None,
                angle=angle,
            )
        )

    def step(
        self,
        dt: float,
        cmd_steering: float,
    ) -> None:
        """Make one step of the vehicle model state, based on commands.

        This method needs to be implemented in the child classes.
        """
        raise NotImplementedError

    @property
    def name(self) -> str:
        return self.__class__.__name__


class CarModelBicyclePure(CarModelBase):
    """Models the car behavior as a kinematic bicycle model.

    The model assumes that the car is moving in a circular path, with a constant radius determined by the steering command.
    """

    def compute_radius_from_steering_command(
        self, cmd_steering: float, speed: float
    ) -> float:
        """Formula to compute the turning radius from the steering command.

        Has to be implemented in the child class."""
        raise NotImplementedError

    def compute_steering_command_from_radius(
        self, radius: float, speed: float
    ) -> float:
        """Has to be implemented in the child class.

        Obtained by inversing the compute_radius_from_steering_command function.
        """
        raise NotImplementedError

    def step(
        self,
        dt: float,
        cmd_steering: float,
    ) -> None:

        x = self.states[-1].x
        y = self.states[-1].y
        yaw = self.states[-1].angle
        v = self.states[-1].vx

        radius = self.compute_radius_from_steering_command(cmd_steering, v)

        updated_x = x + radius * (-np.sin(yaw) + np.sin(v * dt / radius + yaw))
        updated_y = y + radius * (np.cos(yaw) - np.cos(v * dt / radius + yaw))
        updated_yaw = yaw + v * dt / radius

        self.states.append(
            State(
                x=updated_x,
                y=updated_y,
                z=None,
                vx=self.states[-1].vx,
                vy=None,
                vz=None,
                angle=updated_yaw,
            )
        )


class CarModelBicycleV0(CarModelBicyclePure):
    """Models the car behavior as a kinematic bicycle model.

    The model assumes that the car is moving in a circular path, with a constant radius determined by the steering command, using geometric measurements,
    i.e. the V0 model.
    """

    def compute_radius_from_steering_command(
        self, cmd_steering: float, speed: float
    ) -> float:
        """Formula to compute the turning radius from the steering command."""
        if cmd_steering == STEERING_IDLE_PWM:
            return self.VERY_LARGE_RADIUS
        else:
            return WHEELBASE / (
                np.tan(
                    STEERING_DIRECTION_FACTOR
                    * (PHYSICAL_MAX_STEERING_ANGLE_RAD / PWM_DIFF_AT_MAX_STEER_ANGLE)
                    * (cmd_steering - STEERING_IDLE_PWM)
                )
            )

    def compute_steering_command_from_radius(
        self, radius: float, speed: float
    ) -> float:
        """Obtained by inversing the compute_radius_from_steering_command function."""
        return STEERING_IDLE_PWM + STEERING_DIRECTION_FACTOR * (
            PWM_DIFF_AT_MAX_STEER_ANGLE / PHYSICAL_MAX_STEERING_ANGLE_RAD
        ) * np.arctan(WHEELBASE / radius)


class CarModelBicycleV1(CarModelBicyclePure):
    """Models the car behavior as a kinematic bicycle model.

    The model assumes that the car is moving in a circular path, with a constant radius determined by the steering command, using measurements at low speed,
    i.e. the V1 model.
    """

    def compute_radius_from_steering_command(
        self, cmd_steering: float, speed: float
    ) -> float:
        """Compute the radius of the turn that the car will follow, based on the steering command."""
        if cmd_steering == STEERING_IDLE_PWM:
            return self.VERY_LARGE_RADIUS
        else:
            return WHEELBASE / (
                np.tan(
                    STEERING_DIRECTION_FACTOR
                    * (
                        EFFECTIVE_MAX_STEERING_ANGLE_RAD_V1
                        / PWM_DIFF_AT_MAX_STEER_ANGLE
                    )
                    * (cmd_steering - STEERING_IDLE_PWM)
                )
            )

    def compute_steering_command_from_radius(
        self, radius: float, speed: float
    ) -> float:
        """Obtained by inversing the compute_radius_from_steering_command function."""
        return STEERING_IDLE_PWM + STEERING_DIRECTION_FACTOR * (
            PWM_DIFF_AT_MAX_STEER_ANGLE / EFFECTIVE_MAX_STEERING_ANGLE_RAD_V1
        ) * np.arctan(WHEELBASE / radius)


class CarModelBicycleSpeedToParam(CarModelBicyclePure):
    """Base class for models that assume that the car is moving in a circular path, with a constant radius determined by the steering command,

    Meaning models of the form: radius = coeff / steering_diff , where coeff is obtained by interpolating the SPEEDS_TO_PARAM_MAPPING
    dict based on the current speed.
    """

    SPEEDS_TO_PARAM_MAPPING: dict = {}

    def _compute_coefficient(self, speed: float) -> float:
        """Compute the coefficient by interpolating the speed-to-param mapping and multiplying by the speed.

        Meaning: coeff = param(speed) * speed, where param(speed) is obtained by interpolating the SPEEDS_TO_PARAM_MAPPING dict based on the current speed.

        """
        if not self.SPEEDS_TO_PARAM_MAPPING:
            raise ValueError("SPEEDS_TO_PARAM_MAPPING cannot be empty!")

        if speed < 0:
            raise ValueError("Speed cannot be negative!")
        if speed == 0:
            return self.VERY_LARGE_COEFF

        sorted_speeds = sorted(self.SPEEDS_TO_PARAM_MAPPING.keys())

        if speed <= sorted_speeds[0]:
            param_at_speed = self.SPEEDS_TO_PARAM_MAPPING[sorted_speeds[0]]
        elif speed >= sorted_speeds[-1]:
            param_at_speed = self.SPEEDS_TO_PARAM_MAPPING[sorted_speeds[-1]]
        else:
            for i in range(len(sorted_speeds) - 1):
                if sorted_speeds[i] <= speed <= sorted_speeds[i + 1]:
                    lower_speed = sorted_speeds[i]
                    upper_speed = sorted_speeds[i + 1]
                    lower_param = self.SPEEDS_TO_PARAM_MAPPING[lower_speed]
                    upper_param = self.SPEEDS_TO_PARAM_MAPPING[upper_speed]
                    weight = (speed - lower_speed) / (upper_speed - lower_speed)
                    param_at_speed = lower_param * (1 - weight) + upper_param * weight
                    break

        return param_at_speed * speed

    def compute_radius_from_steering_command(
        self, cmd_steering: float, speed: float
    ) -> float:
        """Formula to compute the turning radius from the steering command."""
        if cmd_steering == STEERING_IDLE_PWM:
            return self.VERY_LARGE_RADIUS
        else:
            coeff = self._compute_coefficient(speed)
            steering_diff = cmd_steering - STEERING_IDLE_PWM
            return STEERING_DIRECTION_FACTOR * coeff / steering_diff

    def compute_steering_command_from_radius(
        self, radius: float, speed: float
    ) -> float:
        """Obtained by inversing the compute_radius_from_steering_command function."""
        if abs(radius) < 0.1:
            return STEERING_IDLE_PWM
        else:
            coeff = self._compute_coefficient(speed)
            return STEERING_IDLE_PWM + STEERING_DIRECTION_FACTOR * (coeff / radius)


class CarModelBicycleV2(CarModelBicycleSpeedToParam):
    """
    These are the params from the first attempt at parameter fitting (2023-2024), i.e. the V2 model.
    They were estimated from steady-state circle fitting, using scripts in the folder steering_param_identification_old
    """

    SPEEDS_TO_PARAM_MAPPING: dict = {  # here, params were computed as: steering_diff * radius of circle at the given speed
        1.5: 27 * 1.25,
        5: 24 * 2.3,
        8: 26 * 4,
    }


class CarModelBicycleV3(CarModelBicycleSpeedToParam):
    """
    Similar to V2, but with updated params based on the 2025-07-20 San Mateo bagfiles (still steady-state circle fitting)
    """

    SPEEDS_TO_PARAM_MAPPING: dict = {
        2: 12.67852795,
        3: 12.29078091,
        4: 10.12400495,
        5: 9.71443423,
        6: 9.55109679,
        7: 10.15111903,
    }
