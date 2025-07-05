import copy
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np
from bagfile_loader import BagfileRecord
from car_models import CarModelBase, StampedState


class PureOpenLoopSimulator:
    """Open loop simulator: given a start state and a list of commands, compute the successive states"""

    def __init__(
        self,
        name: str,
        initial_stamped_states: List[StampedState],
        car_model: CarModelBase,
    ) -> None:
        self.name = name
        self.initial_stamped_states = copy.deepcopy(initial_stamped_states)
        self.stamped_states: List[StampedState] = []
        self.car_model = car_model

        print(self.initial_stamped_states)

    def simulate(
        self,
        dts: List[float],
        cmd_steers: List[float],
        cmd_throttles: List[float],
        v_ground_truths: List[float],
    ):

        self.stamped_states = self.initial_stamped_states

        for dt, cmd_steer, cmd_throttle, v_ground_truth in zip(
            dts, cmd_steers, cmd_throttles, v_ground_truths
        ):
            needed_stamped_states = self.stamped_states[-self.car_model.LOOKBACK :]
            updated_stamped_state = self.car_model.step(
                stamped_states=needed_stamped_states,
                dt=dt,
                cmd_steer=cmd_steer,
                cmd_throttle=cmd_throttle,
                v_ground_truth=v_ground_truth,
            )
            self.stamped_states.append(updated_stamped_state)

    def plot(
        self,
        plot_direction: bool = True,
        plot_index: bool = True,
    ):
        for i, stamped_state in enumerate(self.stamped_states):
            state = stamped_state.state
            plt.scatter(state.x, state.y)
            if plot_direction:
                plt.plot(
                    [state.x, state.x + 0.1 * np.cos(state.yaw)],
                    [state.y, state.y + 0.1 * np.sin(state.yaw)],
                )
            if plot_index:
                plt.text(x=state.x, y=state.y, s=i)
        plt.axis("equal")
        plt.title(self.car_model.name)
        plt.grid()
        plt.show()


class PureOpenLoopModelsComparator:
    def __init__(self, simulators: List[PureOpenLoopSimulator]) -> None:
        self.simulators: List[PureOpenLoopSimulator] = simulators

    def plot(
        self,
        plot_direction: bool = True,
        plot_index: bool = True,
    ):

        # TODO need to make sure all simulators got the same commands

        color_cycle = plt.cm.tab10.colors
        if len(self.simulators) > len(color_cycle):
            print("More simulators that colors, there will be duplicates")

        for j, simulator in enumerate(self.simulators):
            for i, stamped_state in enumerate(simulator.stamped_states):
                state = stamped_state.state
                plt.scatter(state.x, state.y)
                if plot_direction:
                    plt.plot(
                        [state.x, state.x + 0.1 * np.cos(state.yaw)],
                        [state.y, state.y + 0.1 * np.sin(state.yaw)],
                        color=color_cycle[j % len(color_cycle)],
                    )
                if plot_index:
                    plt.text(
                        x=state.x,
                        y=state.y,
                        s=i,
                        color=color_cycle[j % len(color_cycle)],
                    )
        plt.axis("equal")
        plt.grid()
        plt.show()


class ClosedLoopSimulator:
    """Closed loop simulator: given a ground truth state and command at each timestamp, compute the next step"""

    def __init__(
        self,
        name: str,
        bagfile_records_dicts: List[Dict[float, BagfileRecord]],
        car_model: CarModelBase,
    ) -> None:
        self.name = name
        self.bagfile_records_dicts = bagfile_records_dicts
        self.car_model = car_model
        self.stamped_states_bag: List[StampedState] = []
        self.steering_cmds_bag: List[float] = []
        self.stamped_states_simulated: List[StampedState] = []

        self.timestamps = list(bagfile_records_dicts.keys())
        self.timestamps.sort()
        # We create the stamped states

        for timestamp in self.timestamps:
            stamped_state = StampedState(
                t=timestamp, state=bagfile_records_dicts[timestamp].state
            )
            self.stamped_states_bag.append(stamped_state)
            self.steering_cmds_bag.append(
                (bagfile_records_dicts[timestamp].steering_cmd)
            )

    def simulate(self):
        last_time = self.timestamps[0]
        for i in range(len(self.stamped_states_bag) - 2):
            updated_stamped_state = self.car_model.step(
                stamped_states=[self.stamped_states_bag[i]],
                dt=self.timestamps[i] - last_time,
                cmd_steer=self.steering_cmds_bag[i],
                v_ground_truth=self.stamped_states_bag[i].state.v,
            )
            self.stamped_states_simulated.append(updated_stamped_state)
            last_time = self.timestamps[i]

    def plot(
        self,
        plot_direction: bool = True,
        plot_index: bool = True,
        plot_connections: bool = True,
    ):
        for i, (stamped_state_bag, stamped_state_simulated) in enumerate(
            zip(self.stamped_states_bag[1:], self.stamped_states_simulated[:-1])
        ):

            state_bag = stamped_state_bag.state
            state_simulated = stamped_state_simulated.state
            if plot_connections:
                plt.plot(
                    [state_bag.x, state_simulated.x],
                    [state_bag.y, state_simulated.y],
                    color="black",
                )
            plt.scatter(state_bag.x, state_bag.y, color="blue")
            plt.scatter(state_simulated.x, state_simulated.y, color="orange")
            if plot_direction:
                plt.plot(
                    [state_bag.x, state_bag.x + 0.1 * np.cos(state_bag.yaw)],
                    [state_bag.y, state_bag.y + 0.1 * np.sin(state_bag.yaw)],
                    color="blue",
                )
                plt.plot(
                    [
                        state_simulated.x,
                        state_simulated.x + 0.1 * np.cos(state_simulated.yaw),
                    ],
                    [
                        state_simulated.y,
                        state_simulated.y + 0.1 * np.sin(state_simulated.yaw),
                    ],
                    color="orange",
                )
            if plot_index:
                plt.text(x=state_bag.x, y=state_bag.y, s=i)

        plt.axis("equal")
        plt.grid()
        plt.title(self.name)
        plt.show()

    def compute_metrics(self):

        all_speeds = []
        speeds_above_speed_threshold = []
        all_offsets = []
        offsets_above_speed_threshold = []
        SPEED_THRESHOLD = 1
        for stamped_state_bag, stamped_bag_simulated in zip(
            self.stamped_states_bag[1:], self.stamped_states_simulated[:-1]
        ):
            state_bag = stamped_state_bag.state
            state_simulated = stamped_bag_simulated.state

            offset = np.linalg.norm(
                [state_bag.x - state_simulated.x, state_bag.y - state_simulated.y]
            )
            all_offsets.append(offset)
            all_speeds.append(state_bag.v)
            if state_bag.v > SPEED_THRESHOLD:
                offsets_above_speed_threshold.append(offset)
                speeds_above_speed_threshold.append(state_bag.v)

        plt.plot(all_offsets, label="all pair-wise offsets")
        plt.plot(
            offsets_above_speed_threshold,
            label=f"pair-wise offsets above {SPEED_THRESHOLD} m/s",
        )
        plt.legend()
        plt.xlabel("index")
        plt.ylabel("pair-wise offset (m)")
        plt.title(f"{self.name}")
        plt.show()

        plt.hist(all_offsets, label="all pair-wise offsets")
        plt.hist(
            offsets_above_speed_threshold,
            label=f"pair-wise offsets above {SPEED_THRESHOLD} m/s",
        )
        plt.legend()
        plt.xlabel("pair-wise offset (m)")
        plt.ylabel("count")
        plt.title(
            f"{self.name} Average distance error: {np.mean(all_offsets)} , {np.mean(offsets_above_speed_threshold)}"
        )
        plt.show()

        plt.scatter(all_speeds, all_offsets)
        plt.xlabel("vehicle speed (m/s)")
        plt.ylabel("pair-wise offset")
        plt.title(f"{self.name}")
        plt.show()


class ClosedLoopModelsComparator:
    def __init__(self, simulators: List[ClosedLoopSimulator]) -> None:
        self.simulators: List[ClosedLoopSimulator] = simulators

    def plot(
        self,
        plot_direction: bool = True,
        plot_index: bool = True,
        plot_connections: bool = True,
    ):

        # TODO make sure they have used the same bag

        color_cycle = plt.cm.tab10.colors
        if len(self.simulators) > len(color_cycle):
            print("More simulators that colors, there will be duplicates")

        # plot points from the bag
        for (
            i,
            stamped_state_bag,
        ) in enumerate(self.simulators[0].stamped_states_bag):
            state_bag = stamped_state_bag.state

            plt.scatter(state_bag.x, state_bag.y, color="black")
            if plot_direction:
                plt.plot(
                    [state_bag.x, state_bag.x + 0.1 * np.cos(state_bag.yaw)],
                    [state_bag.y, state_bag.y + 0.1 * np.sin(state_bag.yaw)],
                    color="black",
                )

            if plot_index:
                plt.text(x=state_bag.x, y=state_bag.y, s=i)

        # plot the points from each simulator
        for j, simulator in enumerate(self.simulators):
            current_color = color_cycle[j % len(color_cycle)]
            for i, (stamped_state_bag, stamped_state_simulated) in enumerate(
                zip(
                    simulator.stamped_states_bag[1:],
                    simulator.stamped_states_simulated[:-1],
                )
            ):

                state_bag = stamped_state_bag.state
                state_simulated = stamped_state_simulated.state
                if plot_connections:
                    plt.plot(
                        [state_bag.x, state_simulated.x],
                        [state_bag.y, state_simulated.y],
                        color=current_color,
                    )
                plt.scatter(state_simulated.x, state_simulated.y, color=current_color)
                if plot_direction:
                    plt.plot(
                        [
                            state_simulated.x,
                            state_simulated.x + 0.1 * np.cos(state_simulated.yaw),
                        ],
                        [
                            state_simulated.y,
                            state_simulated.y + 0.1 * np.sin(state_simulated.yaw),
                        ],
                        color=current_color,
                    )

        plt.axis("equal")
        plt.grid()
        plt.show()
