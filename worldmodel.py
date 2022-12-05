import numpy as np
from traffic_areas import *

class Worldmodel():
    def __init__(self):
        self.areas = {}

    def add_area(self, area, uri=0):
        self.areas.update({uri: area})  # dit kan later iets van geopackage of sqlite/spatialite zijn

    def print_areas(self):
        print(self.areas)

    def plot_areas(self):
        ## ik zou hier later de kleuren nog kunnen veranderen
        for x in self.areas.values():
            plt.fill(*x['polygon'].exterior.xy)
        
    def update_sit(self, sit):
        self.situation = sit

    def current_area(self, robot):     
        for uri, area in self.areas.items():
            if robot.rel_box.intersection(area['polygon']).area > 0.7*robot.rel_box.area:
                return uri
            
