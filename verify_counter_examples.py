# from components.simulation import Simulation
from components.simulation_extended import SimulationExtended as Simulation
#from falsification_demo.controllableSimulation import ControllableSimulation as Simulation
from utils.enums import DrawObservationInVisualizer, VehicleStatus
from itertools import chain, combinations
import pandas as pd
from counterexamples import counterexamples
from resources.config import vehicle_config
import hashlib

model = vehicle_config.controller_model_path.split("/")[-1]

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

l_idx = []
import time
start_time = time.time()
# world_simulator = Simulation()
# run(world_simulator)  # run once to initialize the world simulator
for ctx_idx in range(24):#range(len(counterexamples[model])):

    from resources.config import env_config
    #env_config.predefined_obstacles = [[8.315821250046216, 1.2879584484118278, 0.06], [7.045081450575701, 0.12082184929216852, 0.06], [2.7618769208374463, 0.13536324375329908, 0.06], [3.904242673327906, -0.2425059245859565, 0.06]]
    env_config.predefined_obstacles = counterexamples[model][ctx_idx]#[[round(q, 5) for q in obs[:-1]] + [obs[-1]] for obs in counterexamples[model][ctx_idx]]
    env_config.predefined_obstacles = sorted(env_config.predefined_obstacles, key=lambda x: x[0])
    env_config.number_of_random_obstacles_at_init = 0
    # world_simulator.reset_simulation(env_config=env_config)
    world_simulator = Simulation(env_config=env_config)


    # before
    # world_simulator = WorldSimulatorExtended()
    finished = run(world_simulator)
    import matplotlib.pyplot as plt
    from resources.config import visualizer_config
    from components.simulation import Simulation
    from components.vehicle import Vehicle
    from utils.enums import Visualization, VehicleStatus
    if finished:
        import os
        hash_code = hashlib.sha256(str(sorted(world_simulator.obstacles_relative_to_track)).encode('UTF-8')).hexdigest()
        # hash_code = hash(str(sorted(world_simulator.obstacles_relative_to_track)))
        world_simulator.open_visualizer(visualizer_config.window_position_x, visualizer_config.window_position_y,
                                        visualizer_config.open_observation_view)
        os.makedirs("output", exist_ok=True)
        plt.savefig(f"output/{model}_ctx_{hash_code}.png")
    world_simulator.kill()
    if finished:
        l_idx.append(ctx_idx)
        print(f"counterexample {ctx_idx} not true")

end_time = time.time()    # Record the end time

elapsed_time = end_time - start_time
print(f"Elapsed time: {elapsed_time} seconds")

valid_counterexamples = [counterexamples[model][i] for i in range(len(counterexamples[model])) if i not in l_idx]

for ex in valid_counterexamples[:10]:
    print(ex, ",")