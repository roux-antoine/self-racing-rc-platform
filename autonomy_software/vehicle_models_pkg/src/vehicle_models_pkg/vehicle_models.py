import numpy as np
from geometry_utils_pkg.geometry_utils import State
from vehicle_models_pkg.vehicle_models_constants import (
    STEERING_PWM_IDLE,
    STEERING_DIRECTION_FACTOR,
    WHEELBASE,
    EFFECTIVE_MAX_STEERING_ANGLE_RAD_V1,
    PWM_DIFF_AT_MAX_STEER_ANGLE,
    PHYSICAL_MAX_STEERING_ANGLE_RAD,
)


from typing import List


class CarModelBase:
    """Base class for car models.

    Child classes shall implement the `step` method to update the vehicle state based on commands etc.
    """

    VERY_LARGE_RADIUS = 100000  # radius to return when steering is at the idle position (in m)

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

    def compute_radius_from_steering_command(self, cmd_steering: float, speed: float) -> float:
        """Formula to compute the turning radius from the steering command.

        Has to be implemented in the child class."""
        raise NotImplementedError

    def compute_steering_command_from_radius(self, radius: float, speed: float) -> float:
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

    def compute_radius_from_steering_command(self, cmd_steering: float, speed: float) -> float:
        """Formula to compute the turning radius from the steering command."""
        if cmd_steering == STEERING_PWM_IDLE:
            return self.VERY_LARGE_RADIUS
        else:
            return WHEELBASE / (
                np.tan(
                    STEERING_DIRECTION_FACTOR
                    * (PHYSICAL_MAX_STEERING_ANGLE_RAD / PWM_DIFF_AT_MAX_STEER_ANGLE)
                    * (cmd_steering - STEERING_PWM_IDLE)
                )
            )

    def compute_steering_command_from_radius(self, radius: float, speed: float) -> float:
        """Obtained by inversing the compute_radius_from_steering_command function."""
        return STEERING_PWM_IDLE + STEERING_DIRECTION_FACTOR * (
            PWM_DIFF_AT_MAX_STEER_ANGLE / PHYSICAL_MAX_STEERING_ANGLE_RAD
        ) * np.arctan(WHEELBASE / radius)


class CarModelBicycleV1(CarModelBicyclePure):
    """Models the car behavior as a kinematic bicycle model.

    The model assumes that the car is moving in a circular path, with a constant radius determined by the steering command, using measurements at low speed,
    i.e. the V1 model.
    """

    def compute_radius_from_steering_command(self, cmd_steering: float, speed: float) -> float:
        """TODO"""
        if cmd_steering == STEERING_PWM_IDLE:
            return self.VERY_LARGE_RADIUS
        else:
            return WHEELBASE / (
                np.tan(
                    STEERING_DIRECTION_FACTOR
                    * (
                        EFFECTIVE_MAX_STEERING_ANGLE_RAD_V1
                        / PWM_DIFF_AT_MAX_STEER_ANGLE
                    )
                    * (cmd_steering - STEERING_PWM_IDLE)
                )
            )

    def compute_steering_command_from_radius(self, radius: float, speed: float) -> float:
        """Obtained by inversing the compute_radius_from_steering_command function."""
        return STEERING_PWM_IDLE + STEERING_DIRECTION_FACTOR * (
            PWM_DIFF_AT_MAX_STEER_ANGLE / EFFECTIVE_MAX_STEERING_ANGLE_RAD_V1
        ) * np.arctan(WHEELBASE / radius)


class CarModelBicycleV2(CarModelBicyclePure):
    """Models the car behavior as a kinematic bicycle model.

    The model assumes that the car is moving in a circular path, with a constant radius determined by the steering command, using measurements at low speed,
    i.e. the V2 model.
    """

    UPPER_BOUND_REGION_1: float = 1.5  # m/s
    UPPER_BOUND_REGION_2: float = 5  # m/s
    UPPER_BOUND_REGION_3: float = 8  # m/s
    # Coeffs are computed as: steering_diff * radius of circle at the given speed
    COEFF_REGION_1: float = 27 * 1.25
    COEFF_REGION_2: float = 24 * 2.3
    COEFF_REGION_3: float = 26 * 4

    def _compute_coefficient(self, speed: float) -> float:
        """Compute the coefficient used to compute the required turning radius, see model description for more details."""

        if speed < 0:
            raise ValueError("Speed cannot be negative!")
        if speed == 0:
            coeff = self.VERY_LARGE_RADIUS
        elif speed > 0 and speed <= self.UPPER_BOUND_REGION_1:
            coeff = self.COEFF_REGION_1
        elif speed > self.UPPER_BOUND_REGION_1 and speed <= self.UPPER_BOUND_REGION_2:
            coeff = self.COEFF_REGION_1 + (speed - self.UPPER_BOUND_REGION_1) * (
                self.COEFF_REGION_2 - self.COEFF_REGION_1
            ) / (self.UPPER_BOUND_REGION_2 - self.UPPER_BOUND_REGION_1)
        elif speed > self.UPPER_BOUND_REGION_2 and speed <= self.UPPER_BOUND_REGION_3:
            coeff = self.COEFF_REGION_2 + (speed - self.UPPER_BOUND_REGION_2) * (
                self.COEFF_REGION_3 - self.COEFF_REGION_2
            ) / (self.UPPER_BOUND_REGION_3 - self.UPPER_BOUND_REGION_2)
        elif speed > self.UPPER_BOUND_REGION_3:
            coeff = self.COEFF_REGION_3
        else:
            raise ValueError("This should never happen!")

        return coeff

    def compute_radius_from_steering_command(self, cmd_steering: float, speed: float) -> float:
        """Formula to compute the turning radius from the steering command."""
        if cmd_steering == STEERING_PWM_IDLE:
            return self.VERY_LARGE_RADIUS
        else:
            coeff = self._compute_coefficient(speed)
            radius = (
                STEERING_DIRECTION_FACTOR * coeff * (cmd_steering - STEERING_PWM_IDLE)
            )
            return radius

    def compute_steering_command_from_radius(self, radius: float, speed: float) -> float:
        """Obtained by inversing the compute_radius_from_steering_command function."""
        if radius == 0:
            return STEERING_PWM_IDLE
        else:
            coeff = self._compute_coefficient(speed)
            return STEERING_PWM_IDLE + STEERING_DIRECTION_FACTOR * (coeff / radius)
