import numpy as np
from traffic_areas import *
from rdflib import URIRef

class Worldmodel():
    def __init__(self):
        self.areas = {}
        self.robot_pos = []
        self.situation = URIRef("http://example.com/intersection")
        self.add_robot_pos(self.situation)
        
    def set_goal(self, direction):
        # human grounding, miss ook met bepaalde intersection geven, of met de whole queryen
        self.goal_finished = False
        self.start = URIRef("http://example.com/intersection/road1")

        if direction == 'right':
            self.goal = URIRef("http://example.com/intersection/road2")
        
        if direction == 'straight':
            self.goal = URIRef("http://example.com/intersection/road3")
        
        if direction == 'left':
            self.goal = URIRef("http://example.com/intersection/road4")

        self.add_robot_pos(self.start)

    def add_robot_pos(self, pos):
        self.robot_pos.append(pos)

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
            
