import numpy as np
from traffic_areas import *

class Worldmodel():
    def __init__(self):
        self.areas = {}

    def set_KG(self, g):
        self.g = g

    def add_area(self, uri, area):
        self.areas.update({uri: area})

    def print_areas(self):
        print(self.areas)

    def plot_areas(self):
        for x in self.areas.values():
            plt.fill(*x['polygon'].exterior.xy)
        
    def update_sit(self, sit):
        self.situation = sit

    def config_sit(self, road_width, road_yaw):
        self.road_width = road_width
        self.road_yaw = road_yaw

    def set_behaviour_map(self, robot):
        l = 20
        self.behaviour_map = OneLaneRoad(np.array([0,l+robot.pos[1]]), 0.5*math.pi, 2*l+2*robot.length, self.road_width)