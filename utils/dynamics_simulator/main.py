from bagfile_loader import BagfileSectionExtractor
from car_models import (
    CarModelBicycleEffectiveSteerAngle,
    CarModelBicyclePure,
    CarModelBicycleVariableR,
    StampedState,
    State,
)
from simulators import (
    ClosedLoopModelsComparator,
    ClosedLoopSimulator,
    PureOpenLoopModelsComparator,
    PureOpenLoopSimulator,
)

pure_bicycle_model = CarModelBicyclePure()
adapted_bicycle_model = CarModelBicycleEffectiveSteerAngle()
bicycle_model_variable_r = CarModelBicycleVariableR()


DO_OPEN_LOOP = True
DO_CLOSED_LOOP = True

# Open loop
if DO_OPEN_LOOP:
    NB_POINTS = 50
    dts = [0.1] * NB_POINTS
    cmd_steers = [98 - 5] * NB_POINTS
    cmd_throttles = [None] * NB_POINTS
    v_ground_truths = [1] * NB_POINTS

    initial_stamped_states = [StampedState(t=0, state=State(0, 0, 0, 0))]

    simulator_1 = PureOpenLoopSimulator(
        name=pure_bicycle_model.name,
        initial_stamped_states=initial_stamped_states,
        car_model=pure_bicycle_model,
    )
    simulator_1.simulate(
        dts=dts,
        cmd_steers=cmd_steers,
        cmd_throttles=cmd_throttles,
        v_ground_truths=v_ground_truths,
    )
    simulator_1.plot(plot_direction=True, plot_index=True)

    simulator_2 = PureOpenLoopSimulator(
        name=adapted_bicycle_model.name,
        initial_stamped_states=initial_stamped_states,
        car_model=adapted_bicycle_model,
    )
    simulator_2.simulate(
        dts=dts,
        cmd_steers=cmd_steers,
        cmd_throttles=cmd_throttles,
        v_ground_truths=v_ground_truths,
    )
    simulator_2.plot(plot_direction=True, plot_index=True)

    simulator_3 = PureOpenLoopSimulator(
        name=bicycle_model_variable_r.name,
        initial_stamped_states=initial_stamped_states,
        car_model=bicycle_model_variable_r,
    )
    simulator_3.simulate(
        dts=dts,
        cmd_steers=cmd_steers,
        cmd_throttles=cmd_throttles,
        v_ground_truths=v_ground_truths,
    )
    simulator_3.plot(plot_direction=True, plot_index=True)

    comparator = PureOpenLoopModelsComparator(
        simulators=[simulator_1, simulator_2, simulator_3]
    )
    comparator.plot()

# Closed loop
if DO_CLOSED_LOOP:
    bagfile_loader = BagfileSectionExtractor(
        bag_path="bags_2024-06-22/open_loop_rex_manor_2024-06-22-17-05-50.bag",
        debug=False,
    )

    closed_loop_simulator_pure_bicycle_model = ClosedLoopSimulator(
        name="pure_bicycle_model",
        bagfile_records_dicts=bagfile_loader.bagfile_records_dicts,
        car_model=pure_bicycle_model,
    )
    closed_loop_simulator_pure_bicycle_model.simulate()
    closed_loop_simulator_pure_bicycle_model.plot(
        plot_direction=True, plot_index=False, plot_connections=False
    )
    closed_loop_simulator_pure_bicycle_model.compute_metrics()

    closed_loop_simulator_bicycle_model_variable_r = ClosedLoopSimulator(
        name="bicycle_model_variable_r",
        bagfile_records_dicts=bagfile_loader.bagfile_records_dicts,
        car_model=bicycle_model_variable_r,
    )
    closed_loop_simulator_bicycle_model_variable_r.simulate()
    closed_loop_simulator_bicycle_model_variable_r.plot(
        plot_direction=False, plot_index=False, plot_connections=False
    )
    closed_loop_simulator_bicycle_model_variable_r.compute_metrics()

    closed_loop_model_comparator = ClosedLoopModelsComparator(
        simulators=[
            closed_loop_simulator_pure_bicycle_model,
            closed_loop_simulator_bicycle_model_variable_r,
        ]
    )
    closed_loop_model_comparator.plot(
        plot_direction=False, plot_index=False, plot_connections=False
    )
