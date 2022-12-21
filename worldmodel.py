import numpy as np
from traffic_areas import *
from rdflib import URIRef
from basic_functions import *

class Worldmodel():
    def __init__(self):
        self.absolute_areas = {}
        self.relative_areas = {}
        self.robot_pos = []
        self.situation = URIRef("http://example.com/intersection")
        self.add_robot_pos(self.situation)
        self.number_of_relative_areas = 0
        self.number_of_absolute_areas = 0

    def clear_relative_areas(self):
        self.relative_areas = {}
        self.number_of_relative_areas = 0
        
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

    def set_subgoal(self, line):
        self.subgoal = line
        plt.plot(*self.subgoal.xy, linewidth=5, color='green')

    def add_robot_pos(self, pos):
        self.robot_pos.append(pos)

    def add_area(self, robot, area, uri):
        self.add_relative_area(area, uri)
        abs_area = coordinate_transform_rel_to_abs(robot, area)
        self.add_absolute_area(abs_area, uri)

    def add_relative_area(self, area, uri=0):
        self.relative_areas.update({uri: area})  # dit kan later iets van geopackage of sqlite/spatialite zijn
        self.number_of_relative_areas += 1

    def add_absolute_area(self, area, uri=0):
        self.absolute_areas.update({uri: area})  # dit kan later iets van geopackage of sqlite/spatialite zijn
        self.number_of_absolute_areas += 1
    
    def get_area(self, uri):
        return self.relative_areas[uri]

    def plot_relative_areas(self):
        ## ik zou hier later de kleuren nog kunnen veranderen
        for x in self.relative_areas.values():
            if x.geom_type == 'Polygon':
                plt.fill(*x.exterior.xy, color='lightblue')
            elif x.geom_type == 'LineString':
                plt.plot(*x.xy, color='red', linewidth=2)
        
    def plot_absolute_areas(self):
        ## ik zou hier later de kleuren nog kunnen veranderen
        for x in self.absolute_areas.values():
            if x.geom_type == 'Polygon':
                plt.fill(*x.exterior.xy, color='lightblue')
            elif x.geom_type == 'LineString':
                plt.plot(*x.xy, color='red', linewidth=2)

    def update_sit(self, sit):
        self.situation = sit

    def check_current_area(self, robot):     
        for uri, area in self.relative_areas.items():
            if robot.rel_box.intersection(area).area > 0.7*robot.rel_box.area:
                self.current_area = uri

                return uri

    def update_pos(self, robot):
        self.clear_relative_areas()
        for uri, area in self.absolute_areas.items():
            new_area = coordinate_transform_abs_to_rel(robot, area)
            self.add_relative_area(new_area, uri)

            
