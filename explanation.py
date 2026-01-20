import matplotlib.pyplot as plt
from resources.config import visualizer_config
from components.simulation import Simulation
from utils.enums import DrawObservationInVisualizer, VehicleStatus
from itertools import chain, combinations
import pandas as pd
from counterexamples import counterexamples
from responsibilities import responsibilities
import argparse
import sys
from resources.config import vehicle_config
import random
from copy import deepcopy
import numpy as np

model = vehicle_config.controller_model_path.split("/")[-1]


parser = argparse.ArgumentParser()
parser.add_argument('--ctx_idx', '-C', type=int, default=0)
args = parser.parse_args()

ctx_idx = args.ctx_idx

def powerset(iterable):
    "powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)"
    s = list(iterable)
    return chain.from_iterable(combinations(s, r) for r in range(len(s)+1))

def run(world_simulator):
    states, observations, actions = world_simulator.vehicle.run()

    # check and report vehicle status
    if world_simulator.vehicle.status == VehicleStatus.FINISH:
        print("Goal reached.")
        finished = True
    elif world_simulator.vehicle.status == VehicleStatus.UNSAFE:
        print("Vehicle unsafe.")
        finished = False
    else:
        print("Vehicle timed-out.")
        finished = True  # TODO: check if this is correct, not sure this is what we are looking for

    world_simulator.kill() # close open figures and realease resources
    return finished

def compute_cause(df):
    potential_causes = []
    # AC1 holds since when all obstacles are present, the vehicle is unsafe

    # checking for subsets where AC2 holds
    for index, row in df.iterrows():
        if index == 0:  # TODO: a cause cannot be an empty subset
            continue
        filtered_df = df.copy()
        obs_subset = set()

        # option 1: x' must be different than x
        filtered_df["disjunction"] = False  # column filled with False
        for i in range(num_of_obstacles):
            if row[f"obs{i}"] == 1:
                filtered_df["disjunction"] |= filtered_df[f"obs{i}"] == 0
                obs_subset.add(i)
        necessity_df = filtered_df[filtered_df["disjunction"]]
        sufficiency_df = filtered_df[~filtered_df["disjunction"]]

        # option 2: x' must be the exact opposite of x
        # for i in range(num_of_obstacles):
        #     if row[f"obs{i}"] == 1:
        #         filtered_df = filtered_df[filtered_df[f"obs{i}"] == 0]
        #         obs_subset.add(i)

        # if necessity_df["finished"].any() and not(sufficiency_df["finished"].any()):
        #     potential_causes.append(obs_subset)

        for necessity_index, necessity_row in necessity_df.iterrows():
            sufficiency_df_copy = sufficiency_df.copy()
            if necessity_row["finished"]:
                for i in range(num_of_obstacles):
                    if (i not in obs_subset) and necessity_row[f"obs{i}"] == 1:
                        sufficiency_df_copy = sufficiency_df_copy[sufficiency_df_copy[f"obs{i}"] == 1]
            if not (sufficiency_df_copy["finished"].any()):
                potential_causes.append(obs_subset)
                break

    # checking for minimal subsets (AC3)
    final_causes = []
    for cause in potential_causes:
        is_minimal = True
        for other_cause in potential_causes:
            if other_cause.issubset(cause) and other_cause != cause:
                is_minimal = False
                break
        if is_minimal:
            final_causes.append(cause)
    return final_causes




num_of_obstacles = len(counterexamples[model][ctx_idx])

current_counterexample = sorted(deepcopy(counterexamples[model][ctx_idx]), key=lambda x: x[0])

df = pd.DataFrame(columns=[f"obs{i}" for i in range(num_of_obstacles)] + ["finished"])

for obs_subset in powerset(range(num_of_obstacles)):
    obs_subset = list(obs_subset)
    print(f"Testing obstacle subset: {obs_subset}")

    # Create a new world simulator with the same obstacles
    from resources.config import env_config

    env_config.predefined_obstacles = [obs for i, obs in enumerate(current_counterexample) if i in obs_subset]
    env_config.number_of_random_obstacles_at_init = 0
    world_simulator_copy = Simulation(env_config=env_config)
    finished = run(world_simulator_copy)

    # Store the result in the DataFrame
    result_row = [1 if i in obs_subset else 0 for i in range(num_of_obstacles)] + [finished]
    df.loc[len(df)] = result_row




current_responsibility = deepcopy(responsibilities[model][ctx_idx])

aggregated_responsibility = [current_responsibility[i]+current_responsibility[i+1] for i in range(0, len(current_responsibility), 2)]
aggregated_responsibility = aggregated_responsibility + [0] * (num_of_obstacles - len(aggregated_responsibility))
aggregated_responsibility = np.argsort(aggregated_responsibility)[::-1]  # sort in descending order
print("aggregated responsibility:", aggregated_responsibility)

for i in range(len(aggregated_responsibility)):
    obs_subset = list(aggregated_responsibility[:i+1])
    print(f"Testing obstacle subset: {obs_subset}")

    # Create a new world simulator with the same obstacles
    from resources.config import env_config

    env_config.predefined_obstacles = [obs for i, obs in enumerate(current_counterexample) if i in obs_subset]
    env_config.number_of_random_obstacles_at_init = 0
    world_simulator_copy = Simulation(env_config=env_config)
    finished = run(world_simulator_copy)
    if not finished:
        greedy_df = pd.DataFrame(columns=[f"obs{i}" for i in range(num_of_obstacles)] + ["finished"])
        for obs_subset in powerset(list(aggregated_responsibility[:i+1])):
            obs_subset = list(obs_subset)
            print(f"Testing obstacle subset: {obs_subset}")

            # Create a new world simulator with the same obstacles
            from resources.config import env_config

            env_config.predefined_obstacles = [obs for i, obs in enumerate(current_counterexample) if i in obs_subset]
            env_config.number_of_random_obstacles_at_init = 0
            world_simulator_copy = Simulation(env_config=env_config)
            finished = run(world_simulator_copy)

            # Store the result in the DataFrame
            result_row = [1 if i in obs_subset else 0 for i in range(num_of_obstacles)] + [finished]
            greedy_df.loc[len(greedy_df)] = result_row
        final_causes_brute_force = compute_cause(df)
        # print("potential actual causes are:", potential_causes)
        print("brute force:", final_causes_brute_force)
        final_causes_greedy = compute_cause(greedy_df)
        print("greedy:", final_causes_greedy)
        print("are equal:", final_causes_brute_force == final_causes_greedy)
        break




