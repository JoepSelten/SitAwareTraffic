from queries import *
from basic_functions import *
import matplotlib.pyplot as plt
import numpy as np
import geopandas
import math
from shapely.geometry import Polygon, LineString
from rdflib import URIRef
from global_variables import EX


class Perception():
    def __init__(self):
        self.spotted_sign = False
        self.pos = 0

    def perceive(self, world, inputs):
        self.check_worldmodel(world, inputs)
        #print(world.map_configured)
        if world.changed_geometry:
            self.update_subgoal(world)
        self.update_geometries(world)
        if not world.map_configured:
            self.config_map(world, inputs)
        
        
    def perceive2(self, world, inputs):
        self.check_worldmodel(world, inputs)
        #print(world.map_configured)
        if world.changed_geometry:
            self.update_subgoal(world)
        self.update_geometries(world)
        if not world.map_configured:
            self.config_map(world, inputs)
        

    def check_worldmodel(self, world, inputs):
        for input in inputs:
            ## vergelijk ik met absolute of relative areas?
            abs_input = coordinate_transform_rel_to_abs(world.robot, input)
            for uri, area in world.absolute_areas.items():
                intersect = area.intersection(abs_input)
                
                if intersect.geom_type == abs_input.geom_type and area.geom_type == abs_input.geom_type and not intersect.is_empty:
                    #print(abs_input.difference(area))
                    #print(abs_input)
                    #print(LineString(abs_input.union(area).boundary))   # deze zijn nu nog hetzelfde, maar mocht je n deel niet meer zien dan houd deze t oude ook nog
                    world.absolute_areas.update({uri: abs_input})   # should be union?
                    world.relative_areas.update({uri: input})       # is de relatieve nodig?
                    #print(query_part_of(uri))
                    world.changed_geometry = True
                    ## updating lane that depends on this?

    def update_subgoal(self, world):
        pass
    
    def update_geometries(self, world):
        pass

    def config_map(self, world, inputs):
        #self.situation_from_inputs(world, inputs)
        #self.association(world, inputs)
        pass

    def odometry(self, world):
        # dit doe ik nu in control.actuate
        pass

    
    def situation_from_inputs(self, world, inputs):
        self.road = False
        self.intersection = False
        # this should be a more advanced way to distinguish road and intersection
        # maybe also add a obstacle recognition skills

        if len(inputs)==2:
            world.set_situation('road')
            self.road = True
        elif len(inputs) > 2:
            world.set_situation('intersection')
            self.intersection = True
        else:
            print("Unknown situation!!")

    def association(self, world, inputs):
        sit = world.get_situation()
        #self.separate_perceivable_parts(sit)
        
        if len(inputs)==2:
            self.road_association(world, inputs)
        elif len(inputs) > 2:
            self.intersection_association(world, inputs)

    def road_association(self, world, inputs):
        self.query_parts(world)
        xaxis = LineString(([-30, 0],[30, 0]))
        association = {}    

        for input in inputs:
            intersect = input.intersection(xaxis)
            #print(intersection)
            if intersect:
                if intersect.coords[0][0] <= 0:
                    left_part = query_left_part(self.perceivable_road_parts)
                    world.add_area(input, left_part)
                elif intersect.coords[0][0] > 0:
                    right_part = query_right_part(self.perceivable_road_parts)
                    world.add_area(input, right_part)

        ## moet nog zorgen dat het geen error geeft als het een side niet meer ziet, miss data van de vorige run meenemen
        self.config_lane(world)

    def config_lane(self, world):
        sub_part_areas = []
        for part in self.not_perceivable_road_parts:
            sub_parts = has_a(part)

            #print(part)
            for sub_part in sub_parts:
                for perceivable_part in self.perceivable_road_parts:
                    is_equal = query_if_equal(sub_part, perceivable_part)
                    if is_equal:
                        #print('hello')
                        #association.update(perceivable_part)
                        sub_part_areas.append(world.get_relative_area(perceivable_part))
                    
        #lane_polygon = self.create_polygon_from_lines(line1, line2)
        lane_polygon = Polygon([sub_part_areas[0].coords[0], sub_part_areas[0].coords[1], sub_part_areas[1].coords[1], sub_part_areas[1].coords[0]])
        world.add_area(lane_polygon, self.not_perceivable_road_parts[0])

    def intersection_association(self, world, inputs):
        self.pos = query_part_of(world.start)
        parts = has_first_layer_geometries(self.pos)
        self.perceivable_road_parts = []
        self.not_perceivable_road_parts = []
        n = 0

        intersection = query_part_of(self.pos)
        intersection_parts = has_a(intersection)
        #for part in intersection_parts:
            #print(part)
        for i in range(len(inputs)):
            for j in range(len(inputs)):
                line_intersect = inputs[i].intersection(inputs[j])

                if line_intersect.geom_type=='Point':
                    print(f'corner found at {line_intersect}')
                    print(inputs[i])


    def query_parts(self, world):
        self.pos = world.start
        parts = has_first_layer_geometries(self.pos)
        self.perceivable_road_parts = []
        self.not_perceivable_road_parts = []
        n = 0

        ## seperating what can be perceived, for intersection moet ik eigenlijk nog n laag lager gaan 
        for part in parts:
            perceivable = query_if_perceivable(part)
            
            if perceivable:
                self.perceivable_road_parts.append(part)

            if not perceivable:
                self.not_perceivable_road_parts.append(part)
