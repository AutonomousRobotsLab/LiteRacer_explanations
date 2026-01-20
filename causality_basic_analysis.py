import matplotlib.pyplot as plt
from resources.config import visualizer_config
from components.simulation_extended import SimulationExtended
from utils.enums import DrawObservationInVisualizer, VehicleStatus
from itertools import chain, combinations
import pandas as pd
from counterexamples import counterexamples
from resources.config import vehicle_config

model = vehicle_config.controller_model_path.split("/")[-1]
ctx_idx = 0

for ctx_idx in range(0, len(counterexamples[model])):
    df = pd.read_csv(f"analysis_results/ctx_{model}_{ctx_idx}_analysis.csv")
    num_of_obstacles = len(counterexamples[model][ctx_idx])
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

    # print("potential actual causes are:", potential_causes)
    print(model, ctx_idx, num_of_obstacles, final_causes, sep=";")

# for index, row in df[::-1].iterrows():
#     # if row["finished"]:
#     #     continue
#     obs_subset = set()
#     for i in range(num_of_obstacles):
#         if row[f"obs{i}"] == 1:
#             obs_subset.add(i)
#     print(f"Testing obstacle subset: {obs_subset}")
#
#     # Create a new world simulator with the same obstacles
#     from resources.config import env_config, visualizer_config
#     from utils.enums import DrawObservation
#     visualizer_config.visualization = DrawObservation.ON
#     visualizer_config.draw_observation = DrawObservation.ON
#     obstacles = counterexamples[ctx_idx]
#     obstacles = sorted(obstacles, key=lambda x: x[0])
#     env_config.predefined_obstacles = [obs for i, obs in enumerate(obstacles) if i in obs_subset]
#     env_config.number_of_random_obstacles_at_init = 0
#     world_simulator_copy = WorldSimulatorExtended(env_config=env_config,visualizer_config=visualizer_config)
#
#
#
#     # visualize run
#     if visualizer_config.visualization == DrawObservation.ON:
#         world_simulator_copy.open_world_view(visualizer_config.window_position_x, visualizer_config.window_position_y,
#                                         visualizer_config.open_observation_view)
#
#
#     states, observations, actions = world_simulator_copy.vehicle.run()
#
#     # check and report vehicle status
#     if world_simulator_copy.vehicle.status == VehicleStatus.FINISH:
#         print("Goal reached.")
#     elif world_simulator_copy.vehicle.status == VehicleStatus.UNSAFE:
#         print("Vehicle unsafe.")
#     else:
#         print("Vehicle timed-out.")
#
#     # visualize end of run and block
#     if visualizer_config.visualization == DrawObservation.ON:
#         plt.show(block=True)
#     elif visualizer_config.visualization == DrawObservation.RUN_TERMINATION_ONLY:
#         world_simulator_copy.open_world_view(visualizer_config.window_position_x, visualizer_config.window_position_y,
#                                         visualizer_config.open_observation_view)
#         plt.show(block=True)
#
#     world_simulator_copy.kill()  # close open figures and realease resources
