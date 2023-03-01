import numpy as np
from traffic_areas import *
from rdflib import URIRef
from basic_functions import *
from simplegraph3 import EX
from queries import *
from global_variables import *

class WorldModel():
    def __init__(self, robot):
        self.reset(robot)
        ## of doe ik hier scope ipv pos? is miss wel beter

    def reset(self, robot):
        self.robot = robot
        g.remove((self.robot.uri, None, None))
        g.add((self.robot.uri, RDF.type, EX.vehicle))
        self.current_pos = None
        self.skill_selected = False
        self.skill_configured = False
        self.skill_finished = True
        self.same_situation = False
        self.before_intersection = True
        self.approaching = False
        self.condition_failed = False
        self.omega = 0
        self.velocity = 0
        self.skill = 'move_in_lane'
        if self.robot.start == 'down':
            self.current_pos = URIRef("http://example.com/intersection/road_down/lane_right")
        elif self.robot.start == 'right':
            self.current_pos = URIRef("http://example.com/intersection/road_right/lane_right")
        elif self.robot.start == 'up':
            self.current_pos = URIRef("http://example.com/intersection/road_up/lane_right")
        elif self.robot.start == 'left':
            self.current_pos = URIRef("http://example.com/intersection/road_left/lane_right")

        if self.robot.task == 'down':
            self.goal = URIRef("http://example.com/intersection/road_down/lane_left")
            
        if self.robot.task == 'right':
            self.goal = URIRef("http://example.com/intersection/road_right/lane_left")

        if self.robot.task == 'up':
            self.goal = URIRef("http://example.com/intersection/road_up/lane_left")

        if self.robot.task == 'left':
            self.goal = URIRef("http://example.com/intersection/road_left/lane_left")

        self.plan = [self.current_pos, URIRef("http://example.com/intersection/middle"), self.goal]

        self.update_av_is_on()

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
        

    def update(self, sim):
        self.update_map(sim)
        self.update_kg(sim)

    def update_map(self, sim):
        pass

    def update_kg(self, sim):
        self.update_current_pos()
        self.update_approaching()
        #print(f'scope, {self.robot.name}: {self.scope}')
        self.update_vehicles()
        
        #self.update_is_on(sim)
        ## only check is on for the road when not approaching intersection yet
        

    def update_current_pos(self):
        prev_pos = self.current_pos
        self.current_pos = self.current_area()
        #print(f'Current pos, {self.robot.name}: {self.current_pos}')
        if self.current_pos == None:
            self.robot.random_reset()
            self.reset(self.robot)
            #self.current_pos = self.current_area()
        #prev_pos = query_is_on(self.kg, self.robot.uri)
        #print(f'prev_pos, {self.robot.name}: {prev_pos}')
        #print(f'Current pos, {self.robot.name}: {self.current_pos}')
        if self.current_pos == prev_pos:
            self.same_situation = True
        else:
            self.same_situation = False
            self.update_av_is_on()
            
            #print(self.scope)
        
    def update_av_is_on(self):
        g.remove((self.robot.uri, EX.is_on, None))
        g.add((self.robot.uri, EX.is_on, self.current_pos))
        self.scope = query_part_of(g, self.current_pos)
        self.update_scope()

    def update_scope(self):
        self.scope_list = query_parts(g, self.scope)

    def update_is_on(self, sim):
        for name, robot in sim.robots.items():
            if name == self.robot.name:
                continue
            robot_pos = self.robot_in_scope(robot)
            #print(f'scope: {self.scope}')
            #print(f'robot_pos: {robot_pos}')
            if robot_pos:
                g.remove((robot.uri, EX.is_on, None))
                g.add((robot.uri, EX.is_on, robot_pos))

           
    def update_approaching(self):
        if self.before_intersection and not self.approaching: # miss n check hier of je achter de intersection bent. Miss moet ik hier de hgh level plan queryen
            if query_check_on_road(g, self.robot.uri):
                if self.robot.poly.distance(self.map_dict[URIRef("http://example.com/intersection/middle")]['poly']) < APPROACH_DISTANCE:
                    g.add((self.robot.uri, EX.approaches, URIRef("http://example.com/intersection/middle")))
                    # moet ik dit ook nog ergens uit de graph halen?
                    self.approaching = True
                    self.scope = URIRef("http://example.com/intersection")
        if self.approaching and self.current_pos == URIRef("http://example.com/intersection/middle"):
            self.before_intersection = False
            self.approaching = False
            g.remove((self.robot.uri, EX.approaches, URIRef("http://example.com/intersection/middle")))

    def update_vehicles(self):
        ## in front of, behind, right of etc
        self.vehicles_in_scope = query_vehicles(g, self.scope)
        self.vehicles_in_scope.remove(self.robot.uri)
        #print(self.vehicles_in_scope)
        
        if self.vehicles_in_scope:
            for vehicle in self.vehicles_in_scope:
                self.associate_vehicle(vehicle)

            
    def associate_vehicle(self, vehicle):
        #pos = query_is_on_within_scope(g, vehicle, self.scope)
        
        ## check if and which road
        #print(self.scope)
        if query_check_on_road(g, vehicle) and query_check_on_road(g, self.robot.uri):
            
            road_vehicle = query_road(g, vehicle, self.scope)
            road_current = query_road(g, self.robot.uri, self.scope)
            #print(f'road_current: {road_current}')
            #print(f'road_vehicle: {road_vehicle}')
            if road_current == URIRef("http://example.com/intersection/road_down"):
                if road_vehicle == URIRef("http://example.com/intersection/road_right"):
                    g.add((vehicle, EX.right_of, self.robot.uri))
            if road_current == URIRef("http://example.com/intersection/road_right"):
                if road_vehicle == URIRef("http://example.com/intersection/road_up"):
                    g.add((vehicle, EX.right_of, self.robot.uri))

            if road_current == URIRef("http://example.com/intersection/road_up"):
                if road_vehicle == URIRef("http://example.com/intersection/road_left"):
                    g.add((vehicle, EX.right_of, self.robot.uri))

            if road_current == URIRef("http://example.com/intersection/road_left"):
                if road_vehicle == URIRef("http://example.com/intersection/road_down"):
                    g.add((vehicle, EX.right_of, self.robot.uri))

    def current_area(self):
        self.current_areas = {}
        total = 0
        for key, value in self.map_dict.items():
            if self.robot.poly.intersects(value['poly']):
                intersect_area = self.robot.poly.intersection(value['poly']).area
                self.current_areas[key] = intersect_area
                total += intersect_area
        if total > 0.98*self.robot.poly.area:
            #print(f'current_areas, {self.robot.name}: {self.current_areas}')
            return max(self.current_areas, key=self.current_areas.get)

    def robot_in_scope(self, robot):
        robot_areas = {}
        for uri in self.scope_list:
            #print(uri)
            if robot.poly.intersects(self.map_dict[uri]['poly']):
                intersect_area = self.robot.poly.intersection(self.map_dict[uri]['poly']).area
                robot_areas[uri] = intersect_area
        if robot_areas:
            if max(robot_areas) >= 0.5*self.robot.poly.area:
                return max(robot_areas, key=robot_areas.get)




        
        

            
