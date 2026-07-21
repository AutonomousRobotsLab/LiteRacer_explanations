
from components.simulation import Simulation
from components.vehicle import Vehicle
import shapely.geometry as sg
import random

class SimulationExtended(Simulation):
    def reset_simulation(self, env_config=None):
        if env_config is None:
            from resources.config import env_config
        self.env_config = env_config
        # init vehicle
        self._vehicle_state_history = []
        self._vehicle_shape_history = []
        self._observation_wedges_history = []
        self._initial_vehicle_state = self.vehicle_config.initial_state
        vehicle = Vehicle(self, self.vehicle_config, self._initial_vehicle_state)
        self.reset_vehicle(vehicle, call_sense_after_reset=False)
        # init obstacles (after vehicle, to be able to veify that random obstacles do not overlap with its inital position)

        while len(self.obstacles_in_WF) > 0:
            self.remove_obstacles([0])
        self.obstacles_relative_to_track = []
        self.add_obstacles_relative_to_track(self.env_config.predefined_obstacles)
        if self.env_config.number_of_random_obstacles_at_init > 0:
            self.add_random_obstacles(self.env_config.number_of_random_obstacles_at_init)

        # sense to  observe new obstacles
        self.vehicle.sensor.sense()  # only need to call "sense" the first time. It is later automatically triggered by the state change
        self._observation_wedges_history.append(self._latest_sensing_wedge)
