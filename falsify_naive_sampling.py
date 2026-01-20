# ### To be able to call files from parent folder
# import os, sys
#
# sys.path.insert(1, "/".join(os.path.realpath(__file__).split("/")[0:-2]))
# ###

import matplotlib.pyplot as plt
from resources.config import env_config, vehicle_config
from components.simulation import Simulation
from resources.config import visualizer_config
from components.vehicle import Vehicle
from utils.enums import VehicleStatus, Visualization
import sys
import os
import argparse
import hashlib

model = vehicle_config.controller_model_path.split("/")[-1]

parser = argparse.ArgumentParser()
parser.add_argument('--num_of_iterations', '-N', type=int, default=100)
parser.add_argument('--obstacles', '-O', type=int, default=10)
parser.add_argument('--radius_of_obstacles', '-R', type=float, default=0.1)
args = parser.parse_args()

num_of_iterations = args.num_of_iterations
env_config.number_of_random_obstacles_at_init = args.obstacles
env_config.radius_of_random_obstacles = args.radius_of_obstacles

run_counter = 0
specification_sat_counter = 0
specification_unsat_counter = 0

visualizer_config.visualization = Visualization.OFF
visualizer_config.draw_observation = Visualization.OFF

counter_examples_wf = []
counter_examples_relative = []

while specification_sat_counter + specification_unsat_counter < num_of_iterations:  # for trial in range(1,scene.TRIALS+1):
    run_counter = run_counter + 1
    world_simulator = Simulation(None, env_config, vehicle_config)

    states, observations, actions = world_simulator.vehicle.run(vehicle_config.run_timeout)

    # check vehicle status
    if world_simulator.vehicle.status == VehicleStatus.FINISH:
        print("#" + str(run_counter) + ": Goal reached.")
        specification_sat_counter = specification_sat_counter + 1
    elif world_simulator.vehicle.status == VehicleStatus.UNSAFE:
        specification_unsat_counter = specification_unsat_counter + 1
        print("#" + str(run_counter) + ": Vehicle unsafe. " + str(specification_unsat_counter) + " failures so far.")
        # print(f"{Vehicle.control_loop_counter} controller calls so far.\n")
        # print(world_simulator.obstacles)
        counter_examples_wf.append(world_simulator.obstacles_in_WF)
        counter_examples_relative.append(world_simulator.obstacles_relative_to_track)
        hash_code = hashlib.sha256(str(sorted(world_simulator.obstacles_relative_to_track)).encode('UTF-8')).hexdigest()
        # hash_code = hash(str(sorted(world_simulator.obstacles_relative_to_track)))
        world_simulator.open_visualizer(visualizer_config.window_position_x, visualizer_config.window_position_y,
                                        visualizer_config.open_observation_view)
        os.makedirs("output", exist_ok=True)
        plt.savefig(f"output/{model}_ctx_{hash_code}.png")
    else:
        print("#" + str(run_counter) + ": Vehicle timed-out.")

    world_simulator.kill()
    if run_counter % 10 == 0:
        sys.stdout.flush()

print("obstacles_in_WF:")
for example in counter_examples_wf:
    print(example, ",", sep="")
print("obstacles_relative_to_track:")
for example in counter_examples_relative:
    print(example, ",", sep="")
print(f"SAT runs: {specification_sat_counter}/{run_counter}")
print(f"{Vehicle.control_loop_counter} total controller calls.")