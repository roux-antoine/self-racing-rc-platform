from typing import List, Optional

import numpy as np


class State:
    def __init__(
        self,
        x: float,
        y: float,
        yaw: float,
        v: float,
    ) -> None:
        self.x: float = x
        self.y: float = y
        self.yaw: float = yaw
        self.v: float = v


class StampedState:
    def __init__(self, t: float, state: State) -> None:
        self.t = t
        self.state = state


class CarModelBase:

    LOOKBACK: int = -1  # needs to be overwritten in the child classes

    def step(
        self,
        stamped_states: List[StampedState],
        dt: float,
        cmd_steer: float,
        cmd_throttle: Optional[float] = None,
        v_ground_truth: Optional[float] = None,
    ):
        """Either pass a cmd_throttle or a v_ground_truth"""
        pass

    @property
    def name(self):
        return self.__class__.__name__


class CarModelBicyclePure(CarModelBase):

    WHEELBASE = 0.406  # m
    STEER_IDLE_PWM = 98
    PWM_DIFF_AT_MAX_STEER_ANGLE = 27
    MAX_STEER_ANGLE = 30 * np.pi / 180
    LOOKBACK: int = 1

    def compute_R(self, cmd: float, v: float):
        if cmd == self.STEER_IDLE_PWM:
            return 100000000  # TODO what about negative cases?
        else:
            return self.WHEELBASE / (
                np.tan(
                    (self.MAX_STEER_ANGLE / self.PWM_DIFF_AT_MAX_STEER_ANGLE)
                    * (self.STEER_IDLE_PWM - cmd)
                )
            )

    def step(
        self,
        stamped_states: List[StampedState],
        dt: float,
        cmd_steer: float,
        cmd_throttle: Optional[float] = None,
        v_ground_truth: Optional[float] = None,
    ):

        # this model (at least right now) needs to be given a speed ground truth instead of the cmd_throttle
        assert len(stamped_states) == 1
        assert v_ground_truth
        assert not cmd_throttle

        t = stamped_states[0].t
        x = stamped_states[0].state.x
        y = stamped_states[0].state.y
        yaw = stamped_states[0].state.yaw
        v = stamped_states[
            0
        ].state.v  # reminder: in this model, the speed is hardcoded to the ground truth

        radius = self.compute_R(cmd_steer, v)

        # TODO I think there are still issues in the formula

        alpha = yaw - np.pi / 2
        theta = -v / radius * dt

        updated_x = x + radius * (np.cos(alpha) - np.cos(alpha + theta))
        updated_y = y + radius * (np.sin(alpha) - np.sin(alpha + theta))
        updated_yaw = theta + yaw

        # debugging
        # if v > 1:
        #     import matplotlib.pyplot as plt
        #     plt.scatter(x, y)
        #     plt.plot(
        #         [x, x + 0.1 * np.cos(yaw)],
        #         [y, y + 0.1 * np.sin(yaw)],
        #         color="blue",
        #     )
        #     plt.scatter(updated_x, updated_y)
        #     plt.plot(
        #         [updated_x, updated_x + 0.1 * np.cos(updated_yaw)],
        #         [updated_y, updated_y + 0.1 * np.sin(updated_yaw)],
        #         color="orange",
        #     )
        #     plt.show()

        updated_state = State(
            x=updated_x, y=updated_y, yaw=updated_yaw, v=v_ground_truth
        )
        updated_stamped_state = StampedState(t=t + dt, state=updated_state)

        return updated_stamped_state


class CarModelBicycleEffectiveSteerAngle(CarModelBicyclePure):
    MAX_STEER_ANGLE = 17 * np.pi / 180


class CarModelBicycleVariableR(CarModelBicyclePure):
    UPPER_BOUND_REGION_1 = 1.5  # m/s
    UPPER_BOUND_REGION_2 = 5  # m/s
    UPPER_BOUND_REGION_3 = 8  # m/s
    COEFF_REGION_1 = (
        27 * 1.25
    )  # max steering_diff * radius of circle at max lateral acceleration
    COEFF_REGION_2 = 24 * 2.3
    COEFF_REGION_3 = 26 * 4

    def compute_R(self, cmd, v):
        # see https://antoineroux.notion.site/Self-racing-RC-platform-Lateral-controller-design-part-1-2023-26adcb5ad6ce48fc89e0aa7f1e6030d2?pvs=4

        if self.STEER_IDLE_PWM == cmd:
            return 100000  # big number

        if v == 0:
            coeff = 100000  # just so that we get a small number later on
        elif v > 0 and v <= self.UPPER_BOUND_REGION_1:
            coeff = 1 / (self.COEFF_REGION_1)
        elif v > self.UPPER_BOUND_REGION_1 and v <= self.UPPER_BOUND_REGION_2:
            coeff = 1 / (
                self.COEFF_REGION_1
                + (v - self.UPPER_BOUND_REGION_1)
                * (self.COEFF_REGION_2 - self.COEFF_REGION_1)
                / (self.UPPER_BOUND_REGION_2 - self.UPPER_BOUND_REGION_1)
            )
        elif v > self.UPPER_BOUND_REGION_2 and v <= self.UPPER_BOUND_REGION_3:
            coeff = 1 / (
                self.COEFF_REGION_2
                + (v - self.UPPER_BOUND_REGION_2)
                * (self.COEFF_REGION_3 - self.COEFF_REGION_2)
                / (self.UPPER_BOUND_REGION_3 - self.UPPER_BOUND_REGION_2)
            )
        elif v > self.UPPER_BOUND_REGION_3:
            coeff = 1 / self.COEFF_REGION_3

        R = 1 / (coeff * (self.STEER_IDLE_PWM - cmd))
        return R
