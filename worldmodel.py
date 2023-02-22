import numpy as np
from traffic_areas import *
from rdflib import URIRef
from basic_functions import *
from simplegraph3 import EX

class WorldModel(): # moet uiteindelijk n keus maken tussen relatief of absoluut
    def __init__(self, robot):
        self.robot = robot
        self.AV_uri = URIRef("http://example.com/" + robot.name)

    def init_geometric_map(self, map):
        if map.traffic_situation == "two-lane_intersection":
            self.map_dict = {'0': {'uri': URIRef("http://example.com/intersection/middle"), 'poly': map.polygon_list[0]},
            '1': {'uri': URIRef("http://example.com/intersection/road_current/lane_right"), 'poly': map.polygon_list[1]},
            '2': {'uri': URIRef("http://example.com/intersection/road_current/lane_left"), 'poly': map.polygon_list[2]},
            '3': {'uri': URIRef("http://example.com/intersection/road_right/lane_right"), 'poly': map.polygon_list[3]},
            '4': {'uri': URIRef("http://example.com/intersection/road_right/lane_left"), 'poly': map.polygon_list[4]},
            '5': {'uri': URIRef("http://example.com/intersection/road_up/lane_right"), 'poly': map.polygon_list[5]},
            '6': {'uri': URIRef("http://example.com/intersection/road_up/lane_left"), 'poly': map.polygon_list[6]}, 
            '7': {'uri': URIRef("http://example.com/intersection/road_left/lane_right"), 'poly': map.polygon_list[7]},
            '8': {'uri': URIRef("http://example.com/intersection/road_left/lane_left"), 'poly': map.polygon_list[8]},
            '9': {'uri': URIRef("http://example.com/intersection/road_current/side_right"), 'poly': map.polygon_list[9]},
            '10': {'uri': URIRef("http://example.com/intersection/road_current/side_left"), 'poly': map.polygon_list[10]},
            '11': {'uri': URIRef("http://example.com/intersection/road_right/side_right"), 'poly': map.polygon_list[11]},
            '12': {'uri': URIRef("http://example.com/intersection/road_right/side_left"), 'poly': map.polygon_list[12]},
            '13': {'uri': URIRef("http://example.com/intersection/road_up/side_right"), 'poly': map.polygon_list[13]},
            '14': {'uri': URIRef("http://example.com/intersection/road_up/side_left"), 'poly': map.polygon_list[14]},
            '15': {'uri': URIRef("http://example.com/intersection/road_left/side_right"), 'poly': map.polygon_list[15]},
            '16': {'uri': URIRef("http://example.com/intersection/road_left/side_left"), 'poly': map.polygon_list[16]},
            '17': {'uri': URIRef("http://example.com/intersection/road_down/centerline"), 'poly': map.polygon_list[17]},
            '18': {'uri': URIRef("http://example.com/intersection/road_right/centerline"), 'poly': map.polygon_list[18]},
            '19': {'uri': URIRef("http://example.com/intersection/road_up/centerline"), 'poly': map.polygon_list[19]},
            '20': {'uri': URIRef("http://example.com/intersection/road_left/centerline"), 'poly': map.polygon_list[20]}
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
        current_pos = self.check_current_area()
        ## miss ergens nog n check hebben of de positie wel verandert. 
        self.kg.remove((self.AV_uri, EX.is_on, None))
        self.kg.add((self.AV_uri, EX.is_on, current_pos))
        
        
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
        for key, item in self.map_dict.items():
            if self.robot.box.intersection(item['poly']).area > 0.5*self.robot.rel_box.area:
               return item['uri']




        
        

            
