import matplotlib.pyplot as plt
from literacer.resources.config import visualizer_config
from literacer.components.simulation import Simulation
from literacer.utils.enums import DrawObservationInVisualizer, VehicleStatus
from itertools import chain, combinations
import pandas as pd
from counterexamples import counterexamples
import argparse
import sys
from literacer.resources.config import vehicle_config
import random
import os
from math import pi, dist

random.seed(9)

def powerset(iterable):
    "powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)"
    s = list(iterable)
    return chain.from_iterable(combinations(s, r) for r in range(len(s)+1))


model = vehicle_config.controller_model_path.split("/")[-1]


parser = argparse.ArgumentParser()
parser.add_argument('--ctx_idx', '-C', type=int, default=2)
parser.add_argument('--track_range', '-T', type=int, default=5)
parser.add_argument('--method', '-A', type=str, default="exhaustive_observed", choices=["exhaustive", "exhaustive_observed", "responsibility", "responsibility_discrete"])
parser.add_argument('--delta_min', '-M', type=float, default=-0.05)
parser.add_argument('--delta_max', '-X', type=float, default=0.05)
parser.add_argument('--delta_step', '-S', type=float, default=0.01)
parser.add_argument('--num_of_samples', '-N', type=int, default=101)
args = parser.parse_args()


ctx_idx = args.ctx_idx
track_range = args.track_range
method = args.method
delta_min = args.delta_min
delta_max = args.delta_max
delta_step = args.delta_step
num_of_samples = args.num_of_samples


def finished_without_crashing_obs_i(world_simulator, i=None):
    if i == -1:
        return True
    returned_i = None
    states, observations, actions = world_simulator.vehicle.run()

    # check and report vehicle status
    if world_simulator.vehicle.status == VehicleStatus.FINISH:
        # print("Goal reached.")
        finished = True
    elif world_simulator.vehicle.status == VehicleStatus.UNSAFE:
        # print("Vehicle unsafe.")
        if i is not None:
            if world_simulator._obstacle_circles[i].intersection(world_simulator._vehicle_shape).is_empty:
                finished = True
            else:
                finished = False
        else:
            for j in range(len(world_simulator._obstacle_circles)):
                if not world_simulator._obstacle_circles[j].intersection(world_simulator._vehicle_shape).is_empty:
                    returned_i = j
                    break
            finished = False
    else:
        # print("Vehicle timed-out.")
        finished = True  # TODO: check if this is correct, not sure this is what we are looking for

    world_simulator.kill() # close open figures and realease resources
    if i is None:
        return finished, returned_i
    return finished

def distance_between_envs(self, other_simulation, num_of_obs):
    """Return an estimated distance between envs."""

    # distance estimated as explained in the paper
    N = 32  # number of points to sample for distance estimation

    # distance from this simulation to other simulation
    sum1 = 0
    for i in range(N):
        p_self = self._get_random_obstructed_point()

        dist_p_to_simulation_obstacles = [max([0, dist(obs[0:2], p_self) - obs[2]]) for obs in
                                          other_simulation.obstacles_in_WF]
        sum1 = sum1 + min(dist_p_to_simulation_obstacles)
    avg1 = sum1 / N

    # distance from other simulation to this simulation
    sum2 = 0
    for i in range(N):
        p_simulation = other_simulation._get_random_obstructed_point()

        dist_p_to_self_obstacles = [max([0, dist(obs[0:2], p_simulation) - obs[2]]) for obs in self.obstacles_in_WF]
        sum2 = sum2 + min(dist_p_to_self_obstacles)
    avg2 = sum2 / N

    # print("dists: "+str(avg1)+", "+str(avg2))
    # return symmetric distance
    return (avg1 + avg2) / 2 * num_of_obs  # number of obstacles   ###


num_of_obstacles = len(counterexamples[model][ctx_idx])


from itertools import product
from literacer.resources.config import env_config
from literacer.resources.config import vehicle_config
import numpy as np
from copy import deepcopy

current_counterexample = sorted(deepcopy(counterexamples[model][ctx_idx]), key=lambda x: x[0])
env_config.predefined_obstacles = current_counterexample
env_config.number_of_random_obstacles_at_init = 0
env_config.X_track_range = [0, track_range*pi]
world_simulator_copy = Simulation(env_config=env_config)
original_simulation = world_simulator_copy
finished, crashed_obstacle = finished_without_crashing_obs_i(world_simulator_copy, None)

observed_on_x_axis = world_simulator_copy._vehicle_state_history[-1][0] + vehicle_config.sensor_max_range
obstacles_observed = len([x for x in world_simulator_copy.obstacles_in_WF if x[0] <= observed_on_x_axis])
print(f"Number of obstacles observed: {obstacles_observed}")
print(f"obstacle crashed: {crashed_obstacle}")
if finished:
    print("The counterexample is true, nothing to do.")
    sys.exit(0)

samples_num_range = list(range(100, num_of_samples, 100))
os.makedirs("analysis_results", exist_ok=True)


if method == "responsibility_discrete":
    new_columns = []
    maybe_responsible = list(range(obstacles_observed))
    for i in maybe_responsible:
        new_columns.extend([f"obs{i}"])

    runs = pd.DataFrame(columns=new_columns + ["finished"])


    run_counter = 0
    for _ in range(num_of_samples):
        current_counterexample = sorted(deepcopy(counterexamples[model][ctx_idx]), key=lambda x: x[0])
        sampled_example = []
        obs_subset = []
        for c in range(len(current_counterexample)):
            if c not in maybe_responsible:
                continue
            if _ == 0:
                sampled_example.append(1)  # the first sample is the original counterexample
                obs_subset.append(c)
            elif random.random() < 0.5:
                sampled_example.append(0)
            else:
                sampled_example.append(1)
                obs_subset.append(c)
        # print(current_counterexample)
        # rounded_list = [[round(q, 3) for q in obs[:-1]] + [obs[-1]] for obs in current_counterexample]  # TODO: verify

        current_counterexample = [c for i,c in enumerate(current_counterexample) if i in maybe_responsible]
        if len(current_counterexample) != len(sampled_example):
            raise ValueError("current_counterexample does not match sampled_example")
        env_config.predefined_obstacles = [c for c, in_sim in zip(current_counterexample, sampled_example) if in_sim]
        subset_idx = [i for i, obs in enumerate(current_counterexample) if i in obs_subset]
        updated_crashed_obs_ids = subset_idx.index(crashed_obstacle) if crashed_obstacle in subset_idx else -1
        env_config.number_of_random_obstacles_at_init = 0
        env_config.X_track_range = [0, track_range * pi]
        # before
        # world_simulator_copy = world_simulator.copy()
        # for i in range(num_of_obstacles-1,-1,-1): # remove obstacles in reverse order
        #     if i not in obs_subset:
        #         world_simulator_copy.remove_obstacle(i)

        # Run the simulation
        world_simulator_copy = Simulation(env_config=env_config)
        finished = finished_without_crashing_obs_i(world_simulator_copy, updated_crashed_obs_ids)

        # Store the result in the DataFrame
        result_row = []
        for i in range(len(current_counterexample)):
            result_row.extend([sampled_example[i]])
        result_row += [finished]
        # print(runs.columns)
        # print(result_row)
        runs.loc[len(runs)] = result_row
        run_counter += 1
        if run_counter % 10 == 0:
            sys.stdout.flush()

    runs.to_csv(f"analysis_results/ctx_{model}_{ctx_idx}_second_analysis_discrete.csv", index=False)
    current_counterexample = sorted(deepcopy(counterexamples[model][ctx_idx]), key=lambda x: x[0])
    d = {}
    for c in range(obstacles_observed):
        d[f"obs{c}"] = 1

    # memoization of simulation runs
    memo = {}
    results_discrete_by_samples = {}
    results_discrete_by_samples_only_greedy = {}
    for samples_num in samples_num_range:
        potential_causes = []
        distance_responsibility = {}
        discrete_responsibility = {}
        runs = pd.read_csv(f"analysis_results/ctx_{model}_{ctx_idx}_second_analysis_discrete.csv")
        runs = runs.iloc[:samples_num]

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

        runs_row = runs.iloc[0]
        if runs_row["finished"]:
            print("The first row is already finished, skipping the analysis.")
            sys.exit(0)

        sufficiency_checks_len = []

        for var_key, var_value in dict(runs_row).items():
            print(f"Analyzing variable {var_key} with value {var_value}")
            sys.stdout.flush()
            if not var_key.startswith("obs"):
                continue
            row = {var_key: var_value}
            filtered_df = runs.copy()
            obs_subset = set()
            filtered_df["disjunction"] = False
            filtered_df["disjunction"] |= filtered_df[var_key] != var_value
            obs_subset.add((var_key, var_value))

            check.add(tuple(obs_subset))


            necessity_df = filtered_df[filtered_df["disjunction"]]
            sufficiency_df = filtered_df[~filtered_df["disjunction"]]

            for necessity_index, necessity_row in necessity_df.iterrows():

                if necessity_row["finished"]:
                    contingencies = [(col_name, value) for col_name, value in necessity_row.items() if
                                     col_name.startswith("obs") and (col_name not in dict(obs_subset)) and runs_row[
                                         col_name] != value]

                    discrete_distance = len(set([col_name for col_name, _ in contingencies]))
                    if discrete_responsibility.get(tuple(obs_subset), 0) > 1 / (discrete_distance + 1):
                        continue  # skip if the distance responsibility is already higher than the current one
                    sufficiency_df_copy = sufficiency_df.copy()
                    sufficiency_df_copy["sufficiency_cond"] = sufficiency_df_copy[var_key] == var_value
                    for col_name, value in necessity_row.items():
                        if not col_name.startswith("obs"):
                            continue
                        if (col_name, value) in contingencies:
                            sufficiency_df_copy["sufficiency_cond"] &= sufficiency_df_copy[col_name].isin([value, runs_row[col_name]])
                        else:
                            sufficiency_df_copy["sufficiency_cond"] &= sufficiency_df_copy[col_name] == runs_row[col_name]



                    # print("A")
                    # for comb in powerset(contingencies):
                    #     full_contingency = runs_row.to_dict()
                    #     full_contingency.pop("finished")
                    #     full_contingency.update(dict(comb))
                    #     sufficiency_df_copy.loc[sufficiency_df_copy[list(full_contingency)].eq(pd.Series(full_contingency)).all(
                    #         axis=1), "sufficiency_cond"] = True


                    if not (sufficiency_df_copy[sufficiency_df_copy["sufficiency_cond"]]["finished"].any()):
                        discrete_responsibility[tuple(obs_subset)] = max(discrete_responsibility.get(tuple(obs_subset), 0), 1 / (discrete_distance + 1))
                    else:
                        # print(sufficiency_df_copy[sufficiency_df_copy["sufficiency_cond"]])
                        pass


        for o in range(obstacles_observed):
            if o not in maybe_responsible:
                discrete_responsibility[((f"obs{o}", 1),)] = 0

        final_discrete_responsibility = [discrete_responsibility[x] for x in sorted([x for x in discrete_responsibility.keys()])]
        print("discrete responsibility:", final_discrete_responsibility)

        aggregated_discrete_responsibility = final_discrete_responsibility + [0] * (num_of_obstacles - len(final_discrete_responsibility))
        aggregated_discrete_responsibility = np.argsort((-1)*np.array(aggregated_discrete_responsibility))
        print("aggregated discrete responsibility:", aggregated_discrete_responsibility)

        simulation_greedy_discrete_responsibility = 0
        for i in range(len(aggregated_discrete_responsibility)):
            obs_subset = list(aggregated_discrete_responsibility[:i+1])
            # print(f"Testing obstacle subset: {obs_subset}")

            if tuple(sorted(obs_subset)) not in memo:
                # Create a new world simulator with the same obstacles
                from literacer.resources.config import env_config

                current_counterexample = sorted(deepcopy(counterexamples[model][ctx_idx]), key=lambda x: x[0])
                env_config.predefined_obstacles = [obs for i, obs in enumerate(current_counterexample) if i in obs_subset]
                subset_idx = [i for i, obs in enumerate(current_counterexample) if i in obs_subset]
                updated_crashed_obs_ids = subset_idx.index(crashed_obstacle) if crashed_obstacle in subset_idx else -1
                env_config.number_of_random_obstacles_at_init = 0
                env_config.X_track_range = [0, track_range * pi]
                world_simulator_copy = Simulation(env_config=env_config)
                memo[tuple(sorted(obs_subset))] = finished_without_crashing_obs_i(world_simulator_copy, updated_crashed_obs_ids)

            finished = memo[tuple(sorted(obs_subset))]

            simulation_greedy_discrete_responsibility += 1
            if not finished:
                explanation_greedy_discrete_responsibility = list(aggregated_discrete_responsibility[:i+1])
                results_discrete_by_samples_only_greedy[samples_num] = ";".join(
                    [str(explanation_greedy_discrete_responsibility),
                     str(len(explanation_greedy_discrete_responsibility)),
                     str(simulation_greedy_discrete_responsibility + samples_num),
                     str(aggregated_discrete_responsibility)])
                for obs_subset in powerset(list(aggregated_discrete_responsibility[:i+1])):
                    obs_subset = list(obs_subset)
                    # print(f"Testing obstacle subset: {obs_subset}")

                    if tuple(sorted(obs_subset)) not in memo:
                        # Create a new world simulator with the same obstacles
                        from literacer.resources.config import env_config

                        current_counterexample = sorted(deepcopy(counterexamples[model][ctx_idx]), key=lambda x: x[0])
                        env_config.predefined_obstacles = [obs for i, obs in enumerate(current_counterexample) if i in obs_subset]
                        subset_idx = [i for i, obs in enumerate(current_counterexample) if i in obs_subset]
                        updated_crashed_obs_ids = subset_idx.index(crashed_obstacle) if crashed_obstacle in subset_idx else -1
                        env_config.number_of_random_obstacles_at_init = 0
                        env_config.X_track_range = [0, track_range * pi]
                        world_simulator_copy = Simulation(env_config=env_config)
                        memo[tuple(sorted(obs_subset))] = finished_without_crashing_obs_i(world_simulator_copy, updated_crashed_obs_ids)

                    finished = memo[tuple(sorted(obs_subset))]
                    simulation_greedy_discrete_responsibility += 1

                    if not finished:
                        if len(obs_subset) < len(explanation_greedy_discrete_responsibility):
                            explanation_greedy_discrete_responsibility = obs_subset

                print("minimal set len:", len(explanation_greedy_discrete_responsibility))
                break


        results_discrete_by_samples[samples_num] = ";".join([str(explanation_greedy_discrete_responsibility),
                                                             str(len(explanation_greedy_discrete_responsibility)),
                                                             str(simulation_greedy_discrete_responsibility+samples_num),
                                                             str(aggregated_discrete_responsibility)])


    os.makedirs("output", exist_ok=True)
    with open(f"output/ctx_{ctx_idx}_{method}_discrete_sampling_results.csv", "w") as f:
        f.write(f"{ctx_idx};{num_of_obstacles};{';'.join([results_discrete_by_samples[k] for k in samples_num_range])}\n")
    with open(f"output/ctx_{ctx_idx}_{method}_discrete_sampling_results_only_greedy.csv", "w") as f:
        f.write(f"{ctx_idx};{num_of_obstacles};{';'.join([results_discrete_by_samples_only_greedy[k] for k in samples_num_range])}\n")

if method == "exhaustive":
    # memoization of simulation runs
    memo = {}
    explanation_brute_force_all = list(range(num_of_obstacles))
    simulation_brute_force_all = 0

    for obs_subset in powerset(range(num_of_obstacles)):
        obs_subset = list(obs_subset)
        # print(f"Testing obstacle subset: {obs_subset}")

        if tuple(sorted(obs_subset)) not in memo:
            # Create a new world simulator with the same obstacles
            from literacer.resources.config import env_config

            env_config.predefined_obstacles = [obs for i, obs in enumerate(current_counterexample) if i in obs_subset]
            subset_idx = [i for i, obs in enumerate(current_counterexample) if i in obs_subset]
            updated_crashed_obs_ids = subset_idx.index(crashed_obstacle) if crashed_obstacle in subset_idx else -1
            env_config.number_of_random_obstacles_at_init = 0
            env_config.X_track_range = [0, track_range * pi]
            world_simulator_copy = Simulation(env_config=env_config)
            memo[tuple(sorted(obs_subset))] = finished_without_crashing_obs_i(world_simulator_copy, updated_crashed_obs_ids)
        finished = memo[tuple(sorted(obs_subset))]
        simulation_brute_force_all += 1

        if not finished:
            if len(obs_subset) < len(explanation_brute_force_all):
                explanation_brute_force_all = obs_subset
    os.makedirs("output", exist_ok=True)
    with open(f"output/ctx_{ctx_idx}_{method}_results.csv", "w") as f:
        f.write(f"{ctx_idx};{num_of_obstacles};{explanation_brute_force_all};{len(explanation_brute_force_all)};{simulation_brute_force_all}\n")

if method == "exhaustive_observed":
    # memoization of simulation runs
    memo = {}
    explanation_brute_force_observed = list(range(obstacles_observed))
    simulation_brute_force_observed = 0

    for obs_subset in powerset(range(obstacles_observed)):
        obs_subset = list(obs_subset)
        # print(f"Testing obstacle subset: {obs_subset}")

        if tuple(sorted(obs_subset)) not in memo:
            # Create a new world simulator with the same obstacles
            from literacer.resources.config import env_config

            env_config.predefined_obstacles = [obs for i, obs in enumerate(current_counterexample) if i in obs_subset]
            subset_idx = [i for i, obs in enumerate(current_counterexample) if i in obs_subset]
            updated_crashed_obs_ids = subset_idx.index(crashed_obstacle) if crashed_obstacle in subset_idx else -1
            env_config.number_of_random_obstacles_at_init = 0
            env_config.X_track_range = [0, track_range * pi]
            world_simulator_copy = Simulation(env_config=env_config)
            memo[tuple(sorted(obs_subset))] = finished_without_crashing_obs_i(world_simulator_copy, updated_crashed_obs_ids)

        finished = memo[tuple(sorted(obs_subset))]
        simulation_brute_force_observed += 1

        if not finished:
            if len(obs_subset) < len(explanation_brute_force_observed):
                explanation_brute_force_observed = obs_subset
    print("minimal set len:", len(explanation_brute_force_observed))

    os.makedirs("output", exist_ok=True)
    with open(f"output/ctx_{ctx_idx}_{method}_results.csv", "w") as f:
        f.write(f"{ctx_idx};{num_of_obstacles};{explanation_brute_force_observed};{len(explanation_brute_force_observed)};{simulation_brute_force_observed}\n")

