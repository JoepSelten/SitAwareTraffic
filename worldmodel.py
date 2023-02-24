import numpy as np
from traffic_areas import *
from rdflib import URIRef
from basic_functions import *
from simplegraph3 import EX
from queries import *

class WorldModel(): # moet uiteindelijk n keus maken tussen relatief of absoluut
    def __init__(self, robot):
        self.robot = robot
        self.AV_uri = URIRef("http://example.com/" + robot.name)
        #self.task = robot.task
        self.skill_selected = False
        self.skill_configured = False
        self.skill_finished = True
        self.same_situation = False

    def init_geometric_map(self, map):
        if map.traffic_situation == "two-lane_intersection":
            self.map_dict = {
                URIRef("http://example.com/intersection/middle"): {'poly': map.polygon_list[0]},
                URIRef("http://example.com/intersection/road_down/lane_right"): {'poly': map.polygon_list[1], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_down/lane_left"): {'poly': map.polygon_list[2], 'orientation': -0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/lane_right"): {'poly': map.polygon_list[3], 'orientation': math.pi},
                URIRef("http://example.com/intersection/road_right/lane_left"): {'poly': map.polygon_list[4], 'orientation': 0},
                URIRef("http://example.com/intersection/road_up/lane_right"): {'poly': map.polygon_list[5], 'orientation': -0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/lane_left"): {'poly': map.polygon_list[6], 'orientation': 0.5*math.pi}, 
                URIRef("http://example.com/intersection/road_left/lane_right"): {'poly': map.polygon_list[7], 'orientation': 0},
                URIRef("http://example.com/intersection/road_left/lane_left"): {'poly': map.polygon_list[8], 'orientation': math.pi},
                URIRef("http://example.com/intersection/road_down/side_right"): {'poly': map.polygon_list[9], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_down/side_left"): {'poly': map.polygon_list[10], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/side_right"): {'poly': map.polygon_list[11], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/side_left"): {'poly': map.polygon_list[12], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/side_right"): {'poly': map.polygon_list[13], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/side_left"): {'poly': map.polygon_list[14], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_left/side_right"): {'poly': map.polygon_list[15], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_left/side_left"): {'poly': map.polygon_list[16], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_down/centerline"): {'poly': map.polygon_list[17], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/centerline"): {'poly': map.polygon_list[18], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/centerline"): {'poly': map.polygon_list[19], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_left/centerline"): {'poly': map.polygon_list[20], 'orientation': 0.5*math.pi}
            }

        else:
            self.map_dict = {}
            print("Unknown map!")
        

    def init_kg(self, kg):
        self.kg = kg

    def update(self, sim):
        self.update_map(sim)
        self.update_kg(sim)

    def update_map(self, sim):
        pass

    def update_kg(self, sim):
        self.update_current_pos()
        self.update_approaching()
        self.update_is_on(sim)
        ## only check is on for the road when not approaching intersection yet
        

    def update_current_pos(self):
        current_pos = self.check_current_area()
        if current_pos == None:
            self.robot.random_reset()
            current_pos = self.check_current_area()
        prev_pos = query_is_on(self.kg, self.AV_uri)
        if current_pos == prev_pos:
            self.same_situation = True
        else:
            self.same_situation = False
            self.kg.remove((self.AV_uri, EX.is_on, None))
            self.kg.add((self.AV_uri, EX.is_on, current_pos))
            self.scope = query_part_of(self.kg, current_pos)
            print(self.scope)

    def update_is_on(self, sim):
        print('hoi')
        # ik moet hier niet queryen maar juist toevoegen. Ook kijk ik nu nog steeds alleen naar dezelfde robot. 
        for robot in sim.robots:
            print(query_is_on_within_scope(self.kg, self.AV_uri, self.scope))
            
        
    def update_approaching(self):
        self.approaching = False
        if self.approaching:
            self.scope = URIRef("http://example.com/intersection")

    def set_goal(self, direction):
        # human grounding, miss ook met bepaalde intersection geven, of met de whole queryen
        ## moet deze relatie niet in de graph staan
        self.goal_finished = False
        self.start = URIRef("http://example.com/intersection/road_current")

        if direction == 'right':
            self.goal = URIRef("http://example.com/intersection/road_right")
        
        if direction == 'straight':
            self.goal = URIRef("http://example.com/intersection/road_straight")
        
        if direction == 'left':
            self.goal = URIRef("http://example.com/intersection/road_left")

    def robot_pos(self):
        pass

    def check_current_area(self):
        self.current_areas = {}
        total = 0
        for key, value in self.map_dict.items():
            if value['poly'].intersects(self.robot.poly):
                intersect_area = self.robot.poly.intersection(value['poly']).area
                self.current_areas[key] = intersect_area
                total += intersect_area
        if total > 0.98*self.robot.poly.area:
            return max(self.current_areas, key=self.current_areas.get)
               




        
        

            
