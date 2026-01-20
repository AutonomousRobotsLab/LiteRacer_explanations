"""Run vehicle in specified environment until termination."""

import matplotlib.pyplot as plt
from resources.config import visualizer_config
# from components.simulation import Simulation
from components.simulation_extended import SimulationExtended as Simulation
# from falsification_demo.controllableSimulation import ControllableSimulation as Simulation
from components.vehicle import Vehicle
from utils.enums import Visualization, VehicleStatus
import argparse
from resources.config import vehicle_config
from resources.config import env_config
from counterexamples import counterexamples

# read inline arguments
parser = argparse.ArgumentParser()
parser.add_argument('number_of_runs', nargs='?', type=int, default=1)
parser.add_argument('ctx', nargs='?', type=int, default=-1)
parser.add_argument('model', nargs='?', type=str, default="2b")
args = parser.parse_args()
vehicle_config.controller_model_path = 'resources/controller_models/' + args.model


# run the requested amount of times
# simulation = Simulation(env_config=env_config, vehicle_config=vehicle_config)
for run in range(args.number_of_runs):
    # init simulation
    if args.ctx >= 0:
        current_counterexample = sorted(counterexamples[args.model][args.ctx], key=lambda x: x[0])
        current_counterexample = current_counterexample
        env_config.predefined_obstacles = current_counterexample#+current_counterexample[6:] # use counterexample
        env_config.number_of_random_obstacles_at_init = 0
    simulation = Simulation(env_config=env_config, vehicle_config=vehicle_config)
    # simulation.reset_simulation(env_config=env_config)

    # visualize run
    if visualizer_config.visualization == Visualization.ON_AND_BLOCK_ON_TERMINATION or \
         visualizer_config.visualization == Visualization.ON_AND_NO_BLOCK_ON_TERMINATION:
        simulation.open_visualizer(visualizer_config.window_position_x, visualizer_config.window_position_y, visualizer_config.open_observation_view)

    # run
    print(f"Run #{run+1}: running...")
    states, observations, actions = simulation.vehicle.run()

    # check and report vehicle status
    if simulation.vehicle.status == VehicleStatus.FINISH:
        print("Goal reached.\n")
    elif simulation.vehicle.status == VehicleStatus.UNSAFE:
        print("Vehicle unsafe.\n")
    else:
        print("Vehicle timed-out.\n")

    # visualize end of run and block
    if visualizer_config.visualization == Visualization.ON_AND_BLOCK_ON_TERMINATION:
        plt.show(block=True)
    if visualizer_config.visualization == Visualization.TERMINATION_ONLY:
        simulation.open_visualizer(visualizer_config.window_position_x, visualizer_config.window_position_y, visualizer_config.open_observation_view)
        plt.show(block=True)
    #plt.savefig(f"output/ctx_{0}_different_properties.png")
    simulation.kill() # close open figures and realease resources



    print(f"Control loops: {Vehicle.control_loop_counter}.")
    Vehicle.control_loop_counter = 0



