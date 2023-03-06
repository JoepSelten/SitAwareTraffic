import numpy as np
from traffic_areas import *
from rdflib import URIRef
from basic_functions import *
from simplegraph3 import EX
from queries import *
from global_variables import *

class WorldModel():
    def __init__(self, robot, map):
        self.init_geometric_map(map)
        self.reset(robot)

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
        self.horizon = None
        self.omega = 0
        self.velocity = 0
        self.skill = 'move_in_lane'
        if self.robot.start == 'down':
            self.start_pos = URIRef("http://example.com/intersection/road_down/lane_right")
        elif self.robot.start == 'right':
            self.start_pos = URIRef("http://example.com/intersection/road_right/lane_right")
        elif self.robot.start == 'up':
            self.start_pos = URIRef("http://example.com/intersection/road_up/lane_right")
        elif self.robot.start == 'left':
            self.start_pos = URIRef("http://example.com/intersection/road_left/lane_right")

        if self.robot.task == 'down':
            self.goal = URIRef("http://example.com/intersection/road_down/lane_left")
            self.side_uri = URIRef("http://example.com/intersection/road_down/side_left")
            
        if self.robot.task == 'right':
            self.goal = URIRef("http://example.com/intersection/road_right/lane_left")
            self.side_uri = URIRef("http://example.com/intersection/road_right/side_left")

        if self.robot.task == 'up':
            self.goal = URIRef("http://example.com/intersection/road_up/lane_left")
            self.side_uri = URIRef("http://example.com/intersection/road_up/side_left")

        if self.robot.task == 'left':
            self.goal = URIRef("http://example.com/intersection/road_left/lane_left")
            self.side_uri = URIRef("http://example.com/intersection/road_left/side_left")


        self.phi_before = self.map_dict[self.start_pos].get('orientation')
        self.phi_after = self.map_dict[self.goal].get('orientation')
        self.current_pos = self.start_pos
        side = self.map_dict[self.side_uri].get('poly')
        centerline = shift_line(side, -0.25*w)
        self.extended_centerline = extend_line(centerline, self.phi_after, w)

        self.init_plan()
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
        

    def init_plan(self):
        self.plan = {'0': {'area': self.start_pos, 'skill': 'move_in_lane', 'parameters': [self.phi_before, self.robot.velocity_max]},
                    '1': {'area': URIRef("http://example.com/intersection/middle"), 'skill': 'turn', 'parameters': [self.phi_after, self.robot.velocity_max, self.extended_centerline]},
                    '2': {'area': self.goal, 'skill': 'move_in_lane', 'parameters': [self.phi_after, self.robot.velocity_max]}
        }
        #self.plan = [self.current_pos, URIRef("http://example.com/intersection/middle"), self.goal]
        self.plan_step = 0

    def update(self):
        self.update_kg()
        self.prediction_horizon()

    def update_kg(self):
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

    def prediction_horizon(self):
        ## de eerste grid waar je kunt wachten
        x = self.robot.pos[0]
        y = self.robot.pos[1]
        lane_width = 10
        l = self.robot.length
        H = 20
        self.horizon = None

        for i in range(self.plan_step, len(self.plan)):
            step = self.plan[str(i)]
            type_step = query_type(g, step['area'])
            #print(step['area'].split('/')[-1])
            if str(type_step) == "http://example.com/lane":              
                phi = step['parameters'][0]
                self.horizon = Polygon([(x+0.5*l*math.cos(phi)-lane_width*math.sin(phi), y+0.5*l*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(0.5*l+H)*math.cos(phi)-lane_width*math.sin(phi), y+(0.5*l+H)*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(0.5*l+H)*math.cos(phi)+lane_width*math.sin(phi), y+(0.5*l+H)*math.sin(phi)+lane_width*math.cos(phi)),
                    (x+0.5*l*math.cos(phi)+lane_width*math.sin(phi), y+0.5*l*math.sin(phi)+lane_width*math.cos(phi))]).intersection(self.map_dict[self.current_pos]['poly'])
                #can_wait = query_if_affordance(g, step['area'], "http://example.com/waiting")
                #if step['area'].split('/')[-1]=='lane_right':
                #horizon_length = 
                #print(self.horizon.bounds)
                if str(type_step) == "http://example.com/lane":
                    break
                if self.approaching:
                    self.horizon = self.horizon.union(self.map_dict[self.plan[1]]['poly'])

            if str(type_step) == "http://example.com/middle":  
                self.horizon = self.map_dict[self.plan['1']['area']]['poly']
    
        self.robot.horizon = self.horizon

        




        
        

            
