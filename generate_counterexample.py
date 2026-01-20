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
import random
import numpy as np
from math import pi

parser = argparse.ArgumentParser()
parser.add_argument('--seed', '-S', type=int, default=100)
parser.add_argument('--obstacles', '-O', type=int, default=6)
parser.add_argument('--track_range', '-T', type=int, default=5)
args = parser.parse_args()


def generate_counterexample(seed, model, obstacles, radius_of_obstacles, track_range, timeout, ctx_name):
    np.random.seed(seed)
    random.seed(seed)
    runs_completed = 0
    while timeout > runs_completed:
        vehicle_config.controller_model_path = 'resources/controller_models/' + model
        env_config.X_track_range = [0, track_range * pi]
        env_config.number_of_random_obstacles_at_init = obstacles
        env_config.radius_of_random_obstacles = radius_of_obstacles
        env_config.random_seed = seed
        world_simulator = Simulation(None, env_config, vehicle_config)

        states, observations, actions = world_simulator.vehicle.run(vehicle_config.run_timeout)

        # check vehicle status
        if world_simulator.vehicle.status == VehicleStatus.FINISH:
            print("#" + str(runs_completed) + ": Goal reached.")
        elif world_simulator.vehicle.status == VehicleStatus.UNSAFE:
            print("#" + str(runs_completed) + ": Vehicle unsafe. ")
            ctx_info = {
                'seed': seed,
                'model': model,
                'obstacles_num': obstacles,
                'radius_of_obstacles': radius_of_obstacles,
                'track_range': track_range,
                'obstacles': world_simulator.obstacles_relative_to_track
            }
            world_simulator.open_visualizer(visualizer_config.window_position_x, visualizer_config.window_position_y,
                                            visualizer_config.open_observation_view)
            os.makedirs("output", exist_ok=True)
            plt.savefig(f"output/{ctx_name}.png")
            # save ctx_info as pickle
            import pickle
            with open(f"output/{ctx_name}.pkl", "wb") as f:
                pickle.dump(ctx_info, f)
            break
        else:
            print("#" + str(runs_completed) + ": Vehicle timed-out.")

        world_simulator.kill()
        if runs_completed % 10 == 0:
            sys.stdout.flush()
        runs_completed += 1

if __name__ == "__main__":
    generate_counterexample(args.seed,
                            "2b",
                            args.obstacles,
                            0.1,
                            args.track_range,
                            1000,
                            str(args.seed))