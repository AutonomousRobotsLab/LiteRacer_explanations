import matplotlib.pyplot as plt
from resources.config import visualizer_config
from components.simulation import Simulation
from utils.enums import DrawObservationInVisualizer, VehicleStatus
from itertools import chain, combinations
import pandas as pd
from counterexamples import counterexamples
import argparse
import sys
from resources.config import vehicle_config
import random

random.seed(0)

def powerset(iterable):
    "powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)"
    s = list(iterable)
    return chain.from_iterable(combinations(s, r) for r in range(len(s)+1))


model = vehicle_config.controller_model_path.split("/")[-1]


parser = argparse.ArgumentParser()
parser.add_argument('--ctx_idx', '-C', type=int, default=15)
parser.add_argument('--delta_min', '-M', type=float, default=-0.03)
parser.add_argument('--delta_max', '-X', type=float, default=0.04)
parser.add_argument('--delta_step', '-S', type=float, default=0.03)
parser.add_argument('--num_of_samples', '-N', type=int, default=10000)
parser.add_argument('--compute_responsibility', '-R', action='store_true')
parser.add_argument('--parallel', '-P', action='store_true')
args = parser.parse_args()

ctx_idx = args.ctx_idx
delta_min = args.delta_min
delta_max = args.delta_max
delta_step = args.delta_step
compute_responsibility = args.compute_responsibility
parallel = args.parallel


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


df = pd.read_csv(f"analysis_results/ctx_{model}_{ctx_idx}_analysis.csv")
num_of_obstacles = len(counterexamples[model][ctx_idx])
potential_causes = []
if compute_responsibility:
    responsibility = {}
# AC1 holds since when all obstacles are present, the vehicle is unsafe

# checking for subsets where AC2 holds
for index, row in df.iterrows():
    if index == 0:  # TODO: a cause cannot be an empty subset
        continue
    filtered_df = df.copy()
    obs_subset = set()

    # option 1: x' must be different than x
    filtered_df["disjunction"] = filtered_df[f"obs{0}"] < 0 # column filled with False
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

print("potential actual causes are:", potential_causes)
print("minimal actual causes are:", final_causes)

new_columns = []
for i in range(num_of_obstacles):
    new_columns.extend([f"obs{i}_x", f"obs{i}_y"])
runs = pd.DataFrame(columns=new_columns + ["finished"])
from itertools import product
from resources.config import env_config
import numpy as np
from copy import deepcopy

deltas_y_range = np.arange(delta_min, delta_max, delta_step)
deltas_x_range = np.arange(delta_min, delta_max, delta_step)

if parallel:
    from concurrent.futures import ThreadPoolExecutor
    import pickle
    def run_list(deltas_list):
        runs = pd.DataFrame(columns=new_columns + ["finished"])
        run_counter = 0
        list_of_obs_str = []
        list_of_obs_lst = []
        world_simulator_copy = Simulation(env_config=env_config)
        for deltas in deltas_list:
            current_counterexample = sorted(deepcopy(counterexamples[model][ctx_idx]), key=lambda x: x[0])
            for i, fc in enumerate(final_causes):
                for c in fc:
                    current_counterexample[c][0] += deltas[i]
                    current_counterexample[c][1] += deltas[len(deltas) // 2 + i]
            # rounded_list = [[round(q, 3) for q in obs[:-1]] + [obs[-1]] for obs in current_counterexample]  # TODO: verify
            rounded_list_str = ",".join([",".join(map(str, obs)) for obs in current_counterexample])
            list_of_obs_str.append(rounded_list_str)
            list_of_obs_lst.append(current_counterexample)
        from subprocess import Popen
        pipe = Popen(["python", "run_specific_configuration.py", "-O", ";".join(list_of_obs_str)])
        pipe.wait()
        # Store the result in the DataFrame
        for rounded_list, rounded_list_str in zip(list_of_obs_lst, list_of_obs_str):
            result_row = []
            for obs in rounded_list:
                result_row.extend([obs[0], obs[1]])
            with open(f"output/{rounded_list_str.replace(",", "_").replace(".", "_")}", "rb") as f:
                finished = pickle.load(f)
            result_row += [finished]
            runs.loc[len(runs)] = result_row
        return runs
    deltas_list = []
    for deltas in product(*[deltas_x_range for _ in final_causes]+[deltas_y_range for _ in final_causes]):
        deltas_list.append(deltas)
    threads = 20
    deltas_per_thread = len(deltas_list) // threads + 1
    with ThreadPoolExecutor(threads) as executor:
        processes = [executor.submit(run_list, deltas_list[i:i + deltas_per_thread]) for i in range(0, len(deltas_list), deltas_per_thread)]
        results = [p.result() for p in processes]
        for result_row in results:
            runs = pd.concat([runs, result_row], ignore_index=True)
else:
    run_counter = 0
    for deltas in product(*[deltas_x_range for _ in final_causes]+[deltas_y_range for _ in final_causes]):
        world_simulator_copy = Simulation(env_config=env_config)
        current_counterexample = sorted(deepcopy(counterexamples[model][ctx_idx]), key=lambda x: x[0])
        for i, fc in enumerate(final_causes):
            for c in fc:
                current_counterexample[c][0] += deltas[i]
                current_counterexample[c][1] += deltas[len(deltas)//2+i]
        # rounded_list = [[round(q, 3) for q in obs[:-1]] + [obs[-1]] for obs in current_counterexample]  # TODO: verify
        env_config.predefined_obstacles = current_counterexample
        env_config.number_of_random_obstacles_at_init = 0
        # before
        # world_simulator_copy = world_simulator.copy()
        # for i in range(num_of_obstacles-1,-1,-1): # remove obstacles in reverse order
        #     if i not in obs_subset:
        #         world_simulator_copy.remove_obstacle(i)

        # Run the simulation
        finished = run(world_simulator_copy)

        # Store the result in the DataFrame
        result_row = []
        for obs in env_config.predefined_obstacles:
            result_row.extend([obs[0], obs[1]])
        result_row += [finished]
        runs.loc[len(runs)] = result_row
        run_counter += 1
        if run_counter % 10 == 0:
            sys.stdout.flush()

runs.to_csv(f"analysis_results/ctx_{model}_{ctx_idx}_second_analysis.csv", index=False)
current_counterexample = sorted(deepcopy(counterexamples[model][ctx_idx]), key=lambda x: x[0])
d = {}
for i, fc in enumerate(final_causes):
    for c in fc:#f"obs{i}_x", f"obs{i}_y"
        d[f"obs{c}_x"] = current_counterexample[c][0]
        d[f"obs{c}_y"] = current_counterexample[c][1]
runs = pd.read_csv(f"analysis_results/ctx_{model}_{ctx_idx}_second_analysis.csv")
potential_causes_second = []
# get uniques columns from each column in runs
unique_values = dict()
for col in runs.columns:
    if col.startswith("obs"):
        unique_values[col] = runs[col].unique().tolist() + [None]
        if len(unique_values[col]) == 2:
            unique_values[col] = [None]
# iterate over all combinations of unique values for each obstacle
keys = list(unique_values.keys())
values = list(unique_values.values())

check = set()

for index, runs_row in runs.iterrows():
    flag = False
    for k, v in d.items():
        if runs_row[k] != v:
            flag = True
            break
    if flag:
        continue
    if runs_row["finished"]:
        continue
    for combination in product(*values):
        row = dict([(i,x) for i,x in zip(keys, combination) if x is not None])
        # print(row)
        if not (row.items() <= dict(runs_row).items()):
            continue
        filtered_df = runs.copy()
        obs_subset = set()

        # option 1: x' must be different than x
        filtered_df["disjunction"] = False
        for fc in final_causes:
            for c in fc:
                if f"obs{c}_x" in row:
                    filtered_df["disjunction"] |= filtered_df[f"obs{c}_x"] != row[f"obs{c}_x"]
                    obs_subset.add((f"obs{c}_x", row[f"obs{c}_x"]))
                if f"obs{c}_y" in row:
                    filtered_df["disjunction"] |= filtered_df[f"obs{c}_y"] != row[f"obs{c}_y"]
                    obs_subset.add((f"obs{c}_y", row[f"obs{c}_y"]))

        check.add(tuple(obs_subset))

        if (not compute_responsibility) and obs_subset in potential_causes_second:
            continue

        contingencies_max_distance = abs(runs.iloc[0, :-1] - runs_row.iloc[:-1]).sum()


        necessity_df = filtered_df[filtered_df["disjunction"]]
        sufficiency_df = filtered_df[~filtered_df["disjunction"]]

        for necessity_index, necessity_row in necessity_df.iterrows():
            if necessity_row["finished"]:
                contingencies = [(col_name, value) for col_name, value in necessity_row.items() if col_name.startswith("obs") and (col_name not in dict(obs_subset)) and runs_row[col_name] != value]
                sufficiency_df_copy = sufficiency_df.copy()
                sufficiency_df_copy["sufficiency_cond"] = False
                for comb in powerset(contingencies):
                    full_contingency = runs_row.to_dict()
                    full_contingency.pop("finished")
                    full_contingency.update(dict(comb))
                    sufficiency_df_copy.loc[sufficiency_df_copy[list(full_contingency)].eq(pd.Series(full_contingency)).all(axis=1), "sufficiency_cond"] = True
                if not (sufficiency_df_copy[sufficiency_df_copy["sufficiency_cond"]]["finished"].any()):
                    potential_causes_second.append(obs_subset)
                    if compute_responsibility:
                        contingencies_distance = sum([abs(runs_row[col_name] - value) for col_name, value in contingencies])
                        causes_distance = sum([abs(necessity_row[col_name] - value) for col_name, value in obs_subset])
                        scaled_distance = ((contingencies_distance + causes_distance) * 4) / contingencies_max_distance # TODO: verify this scaling
                        responsibility[tuple(obs_subset)] = max(responsibility.get(tuple(obs_subset), 0), 1/((scaled_distance) + 1))
                    break
print(len(potential_causes_second))
print(len(check))

print([x for x in check if set(x) not in potential_causes_second])

# checking for minimal subsets (AC3)
final_causes_second = []
for cause in potential_causes_second:
    is_minimal = True
    for other_cause in potential_causes_second:
        if other_cause.issubset(cause) and other_cause != cause:
            is_minimal = False
            break
    if is_minimal:
        final_causes_second.append(cause)

# print("potential actual second causes are:", potential_causes_second)
print("minimal causes:", final_causes_second)
if compute_responsibility:
    print("responsibility:", [responsibility[tuple(x)] for x in final_causes_second])

