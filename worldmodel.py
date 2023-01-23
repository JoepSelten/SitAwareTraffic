import numpy as np
from traffic_areas import *
from rdflib import URIRef
from basic_functions import *

class Worldmodel(): # moet uiteindelijk n keus maken tussen relatief of absoluut
    def __init__(self, robot):
        self.absolute_areas = {}
        self.relative_areas = {}
        # self.robot_pos = []
        self.situation = 0
        self.area = 0
        # self.add_robot_pos(self.situation)
        self.number_of_relative_areas = 0
        self.number_of_absolute_areas = 0

        self.robot = robot

        ## booleans for monitor
        self.different_area = False

        self.map_configured = False

        self.changed_geometry = False

        self.relative_subgoal = 0

    def clear_relative_areas(self):
        self.relative_areas = {}
        self.number_of_relative_areas = 0
        
    def set_goal(self, direction):
        # human grounding, miss ook met bepaalde intersection geven, of met de whole queryen
        ## moet deze relatie niet in de graph staan
        self.goal_finished = False
        self.start = URIRef("http://example.com/intersection/road_current")

        if direction == 'right':
            self.goal = URIRef("http://example.com/intersection/road2")
        
        if direction == 'straight':
            self.goal = URIRef("http://example.com/intersection/road3")
        
        if direction == 'left':
            self.goal = URIRef("http://example.com/intersection/road_left")


    def set_situation(self, sit):
        if sit == 'road':
            self.situation = URIRef("http://example.com/intersection/road1")
        elif sit == 'intersection':
            self.situation = URIRef("http://example.com/intersection")
        else:
            print("Unknown situation!")

    def get_situation(self):
        return self.situation

    def set_relative_subgoal(self, line):
        self.relative_subgoal = line

    def get_relative_subgoal(self):
        return self.relative_subgoal

    def set_absolute_subgoal(self, line):
        self.absolute_subgoal = coordinate_transform_rel_to_abs(self.robot, line)
        
    def plot_relative_subgoal(self):
        plt.plot(*self.relative_subgoal.xy, linewidth=5, color='green')

    def plot_absolute_subgoal(self):
        plt.plot(*self.absolute_subgoal.xy, linewidth=5, color='green')

    # def add_robot_pos(self, pos):
    #     self.robot_pos.append(pos)

    def add_area(self, area, uri):
        self.add_relative_area(area, uri)
        abs_area = coordinate_transform_rel_to_abs(self.robot, area)
        self.add_absolute_area(abs_area, uri)

    def add_relative_area(self, area, uri=0):
        self.relative_areas.update({uri: area})  # dit kan later iets van geopackage of sqlite/spatialite zijn
        self.number_of_relative_areas += 1

    def add_absolute_area(self, area, uri=0):
        self.absolute_areas.update({uri: area})  # dit kan later iets van geopackage of sqlite/spatialite zijn
        self.number_of_absolute_areas += 1
    
    def get_relative_area(self, uri):
        if uri in self.relative_areas:
            return self.relative_areas[uri]
        else:
            return 0

    def get_absolute_area(self, uri):
        if uri in self.absolute_areas:
            return self.absolute_areas[uri]
        else:
            return 0

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


    def check_area(self):     
        for uri, area in self.relative_areas.items():
            if self.robot.rel_box.intersection(area).area > 0.7*self.robot.rel_box.area:
                self.area = uri

                return uri

    def update_pos(self):
        self.clear_relative_areas()
        for uri, area in self.absolute_areas.items():
            new_area = coordinate_transform_abs_to_rel(self.robot, area)
            self.add_relative_area(new_area, uri)

        
        

            
