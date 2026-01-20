from components.simulation import Simulation
from utils.enums import DrawObservationInVisualizer, VehicleStatus
from itertools import chain, combinations
import pandas as pd
from counterexamples import counterexamples
from resources.config import vehicle_config

model = vehicle_config.controller_model_path.split("/")[-1]

def powerset(iterable):
    "powerset([1,2,3]) --> () (1,) (2,) (3,) (1,2) (1,3) (2,3) (1,2,3)"
    s = list(iterable)
    return chain.from_iterable(combinations(s, r) for r in range(len(s)+1))

goal_reaching_subsets = []
failing_subsets = []

ctx_idx = 0

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



def find_counterexample():
    while True:

        from resources.config import env_config
        #env_config.predefined_obstacles = [[8.315821250046216, 1.2879584484118278, 0.06], [7.045081450575701, 0.12082184929216852, 0.06], [2.7618769208374463, 0.13536324375329908, 0.06], [3.904242673327906, -0.2425059245859565, 0.06]]
        env_config.predefined_obstacles = counterexamples[model][ctx_idx]#[[round(q, 3) for q in obs[:-1]] + [obs[-1]] for obs in counterexamples[ctx_idx]]
        env_config.predefined_obstacles = sorted(env_config.predefined_obstacles, key=lambda x: x[0])
        env_config.number_of_random_obstacles_at_init = 0
        world_simulator = Simulation(env_config=env_config)


        # before
        # world_simulator = WorldSimulatorExtended()
        finished = run(world_simulator)
        if not finished:
            print("Found a counterexample!")
            return world_simulator
        else:
            raise Exception("No counterexample found, please check the configuration or the counterexamples data.")

for ctx_idx in range(0, 30):#range(0, len(counterexamples[model])):
    world_simulator = find_counterexample()
    num_of_obstacles = len(world_simulator.obstacles_relative_to_track)

    df = pd.DataFrame(columns=[f"obs{i}" for i in range(num_of_obstacles)] + ["finished"])

    for obs_subset in powerset(range(num_of_obstacles)):
        obs_subset = list(obs_subset)
        print(f"Testing obstacle subset: {obs_subset}")

        # Create a new world simulator with the same obstacles
        from resources.config import env_config

        env_config.predefined_obstacles = [obs for i, obs in enumerate(world_simulator.obstacles_relative_to_track) if i in obs_subset]
        env_config.number_of_random_obstacles_at_init = 0
        world_simulator_copy = Simulation(env_config=env_config)
        # before
        # world_simulator_copy = world_simulator.copy()
        # for i in range(num_of_obstacles-1,-1,-1): # remove obstacles in reverse order
        #     if i not in obs_subset:
        #         world_simulator_copy.remove_obstacle(i)

        # Run the simulation
        finished = run(world_simulator_copy)

        if finished:
            goal_reaching_subsets.append(obs_subset)
        else:
            failing_subsets.append(obs_subset)

        # Store the result in the DataFrame
        result_row = [1 if i in obs_subset else 0 for i in range(num_of_obstacles)] + [finished]
        df.loc[len(df)] = result_row

    import os

    os.makedirs("analysis_results", exist_ok=True)
    df.to_csv(f"analysis_results/ctx_{model}_{ctx_idx}_analysis.csv", index=False)