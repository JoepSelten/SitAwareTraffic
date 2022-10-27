import numpy as np
from traffic_areas import *

class Worldmodel():
    def __init__(self):
       pass

    def update_sit(self, sit):
        self.situation = sit

    def config_sit(self, road_width, road_yaw):
        self.road_width = road_width
        self.road_yaw = road_yaw

    def set_behaviour_map(self, robot):
        l = 20
        self.behaviour_map = OneLaneRoad(np.array([0,l+robot.pos[1]]), 0.5*math.pi, 2*l+2*robot.length, self.road_width)