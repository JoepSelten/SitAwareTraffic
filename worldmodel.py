import numpy as np
from traffic_areas import *
from rdflib import URIRef
from basic_functions import *
from simplegraph3 import EX
from queries import *
from global_variables import *
from shapely.validation import make_valid
from shapely.ops import unary_union

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
        self.same_situation = True
        self.before_intersection = True
        self.approaching = False
        self.condition_failed = False
        self.wait = False
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

        if map.traffic_situation == "two-lane_intersection2":
            self.map_dict = {
                #URIRef("http://example.com/intersection/middle"): {'poly': map.polygon_list[0].union(map.polygon_list[1]).union(map.polygon_list[2]).union(map.polygon_list[3])},
                URIRef("http://example.com/intersection/middle_dr"): {'poly': map.polygon_list[0]},
                URIRef("http://example.com/intersection/middle_ur"): {'poly': map.polygon_list[1]},
                URIRef("http://example.com/intersection/middle_ul"): {'poly': map.polygon_list[2]},
                URIRef("http://example.com/intersection/middle_dl"): {'poly': map.polygon_list[3]},
                URIRef("http://example.com/intersection/road_down/lane_right"): {'poly': map.polygon_list[4], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_down/lane_left"): {'poly': map.polygon_list[5], 'orientation': -0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/lane_right"): {'poly': map.polygon_list[6], 'orientation': math.pi},
                URIRef("http://example.com/intersection/road_right/lane_left"): {'poly': map.polygon_list[7], 'orientation': 0},
                URIRef("http://example.com/intersection/road_up/lane_right"): {'poly': map.polygon_list[8], 'orientation': -0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/lane_left"): {'poly': map.polygon_list[9], 'orientation': 0.5*math.pi}, 
                URIRef("http://example.com/intersection/road_left/lane_right"): {'poly': map.polygon_list[10], 'orientation': 0},
                URIRef("http://example.com/intersection/road_left/lane_left"): {'poly': map.polygon_list[11], 'orientation': math.pi},
                URIRef("http://example.com/intersection/road_down/side_right"): {'poly': map.polygon_list[12], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_down/side_left"): {'poly': map.polygon_list[13], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/side_right"): {'poly': map.polygon_list[14], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/side_left"): {'poly': map.polygon_list[15], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/side_right"): {'poly': map.polygon_list[16], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/side_left"): {'poly': map.polygon_list[17], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_left/side_right"): {'poly': map.polygon_list[18], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_left/side_left"): {'poly': map.polygon_list[19], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_down/centerline"): {'poly': map.polygon_list[20], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/centerline"): {'poly': map.polygon_list[21], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/centerline"): {'poly': map.polygon_list[22], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_left/centerline"): {'poly': map.polygon_list[23], 'orientation': 0.5*math.pi}
            }

        else:
            self.map_dict = {}
            print("Unknown map!")
        

    def init_plan(self):
        if self.robot.start == 'down':
            if self.robot.task == 'right':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_down/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '2': {'area': URIRef("http://example.com/intersection/road_right/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if self.robot.task == 'up':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_down/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'area': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '3': {'area': URIRef("http://example.com/intersection/road_up/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if self.robot.task == 'left':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_down/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'area': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '3': {'area': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'area': URIRef("http://example.com/intersection/road_left/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }

        if self.robot.start == 'right':
            if self.robot.task == 'up':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_right/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '2': {'area': URIRef("http://example.com/intersection/road_up/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if self.robot.task == 'left':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_right/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'area': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '3': {'area': URIRef("http://example.com/intersection/road_left/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if self.robot.task == 'down':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_right/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'area': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '3': {'area': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'area': URIRef("http://example.com/intersection/road_down/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }

        if self.robot.start == 'up':
            if self.robot.task == 'left':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_up/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '2': {'area': URIRef("http://example.com/intersection/road_left/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if self.robot.task == 'down':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_up/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'area': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '3': {'area': URIRef("http://example.com/intersection/road_down/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if self.robot.task == 'right':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_up/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'area': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '3': {'area': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'area': URIRef("http://example.com/intersection/road_right/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }

        if self.robot.start == 'left':
            if self.robot.task == 'down':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_left/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '2': {'area': URIRef("http://example.com/intersection/road_down/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if self.robot.task == 'right':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_left/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'area': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '3': {'area': URIRef("http://example.com/intersection/road_right/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if self.robot.task == 'up':
                self.plan = {'0': {'area': URIRef("http://example.com/intersection/road_left/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'area': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'area': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'turn_line': self.extended_centerline}},
                    '3': {'area': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'area': URIRef("http://example.com/intersection/road_up/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
        #self.plan = [self.current_pos, URIRef("http://example.com/intersection/middle"), self.goal]
        
        self.plan_step = 0

        self.right_lane = None
        right_lane_list = []

        for value in self.plan.values():
            new_area = self.map_dict[value['area']]['poly']
            right_lane_list.append(new_area)
        self.right_lane = unary_union(right_lane_list)

        


    def update(self, sim):
        self.update_current_pos(sim)
        #print(f'{self.robot.name}: {self.current_pos}')
        self.prediction_horizon(sim)

        #self.robot.horizon = None
        self.robot.approaching_horizon = None
        self.robot.obstructed_area = None
        self.update_is_on(sim)
        
        if not self.robot_is_on:
            self.update_approaching(sim)
        self.update_vehicles()       

    def update_current_pos(self, sim):
        prev_pos = self.current_pos
        self.current_pos = self.current_area()
        #print(f'Current pos, {self.robot.name}: {self.current_pos}')
        if self.current_pos == None:
            self.robot.random_reset()
            self.reset(self.robot)
            return
            #self.current_pos = self.current_area()
        #prev_pos = query_is_on(self.kg, self.robot.uri)
        #print(f'prev_pos, {self.robot.name}: {prev_pos}')
        #print(f'Current pos, {self.robot.name}: {self.current_pos}')
        if self.current_pos == prev_pos:
            self.same_situation = True
        else:
            self.same_situation = False
            self.plan_step += 1
            
            #self.check_progress()
            self.update_av_is_on()
            
            #print(self.scope)

    def update_av_is_on(self):
        g.remove((self.robot.uri, EX.is_on, None))
        g.add((self.robot.uri, EX.is_on, self.current_pos))
        #print(self.current_pos)
        #self.scope = query_part_of(g, self.current_pos)
        #self.update_scope()

    def update_scope(self):
        self.scope_list = query_parts(g, self.scope)

    def update_is_on2(self, sim):
        for name, robot in sim.robots.items():
            if name == self.robot.name:
                continue
            robot_pos = self.robot_in_scope(robot)
            #print(f'scope: {self.scope}')
            #print(f'robot_pos: {robot_pos}')
            if robot_pos:
                g.remove((robot.uri, EX.is_on, None))
                g.add((robot.uri, EX.is_on, robot_pos))

    def update_is_on(self, sim):
        g.remove((None, EX.obstructs, self.robot.uri))
        self.robot_is_on = False
        self.approaching = False
        self.robot_dict = sim.robots.copy()
        self.robots_is_on = []
        self.scope = URIRef("http://example.com/intersection")
        del self.robot_dict[self.robot.name]
        if self.robot.horizon:
            for robot in self.robot_dict.values():
                if robot.poly:
                    if self.robot.horizon.intersects(robot.poly):
                        self.robots_is_on.append(robot)
                        self.robot.is_on_horizon = self.robot.horizon.intersection(robot.poly)
                        ##  dit kunnen er later ook meer zijn
                        label = self.get_label(self.robot.is_on_horizon)
                        self.robot.obstructed_area = self.map_dict[label]['poly']
                        g.add((robot.uri, EX.obstructs, self.robot.uri))
                        g.add((self.robot.uri, EX.obstructs, label))
                        g.remove((robot.uri, EX.approaches, self.robot.uri))
                        self.robot_is_on = True

           
    def update_approaching2(self):
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

    def update_approaching(self, sim):
        self.approaching = False
        self.robot_dict = sim.robots.copy()
        self.robots_approaching = []
        self.scope = URIRef("http://example.com/intersection")
        del self.robot_dict[self.robot.name]
        if self.robot.horizon:
            for robot in self.robot_dict.values():
                if robot.horizon:
                    if self.robot.horizon.intersects(robot.horizon):
                        self.robots_approaching.append(robot)
                        self.robot.approaching_horizon = self.robot.horizon.intersection(robot.horizon)
                        if self.robot.approaching_horizon.area < 5:
                            break
                        ##  dit kunnen er later ook meer zijn
                        #print(f'{self.robot.name}: {self.robot.approaching_horizon.area}')
                        label = self.get_label(self.robot.approaching_horizon)
                        g.add((robot.uri, EX.approaches, self.robot.uri))
                        g.add((self.robot.uri, EX.approaches, label))
                        self.approaching = True
                        


    def get_label(self, geom):
        prev_intersection = 0
        for key, value in self.map_dict.items():
            if geom.intersects(value['poly']):
                intersect_area = geom.intersection(value['poly']).area
                if intersect_area > prev_intersection:
                    new_key = key
        return new_key
                



    def update_vehicles2(self):
        ## in front of, behind, right of etc
        self.vehicles_in_scope = query_vehicles(g, self.scope)
        self.vehicles_in_scope.remove(self.robot.uri)
        #print(self.vehicles_in_scope)
        
        if self.vehicles_in_scope:
            for vehicle in self.vehicles_in_scope:
                self.associate_vehicle(vehicle)

    def update_vehicles(self):
        #print(f'{self.robot.name}: {self.approaching}')
        if self.approaching:
            for robot in self.robots_approaching:
                self.associate_vehicle(robot)


            
    def associate_vehicle(self, vehicle):
        ## nu nog alleen voor de intersection, later generiek maken

        #if query_check_on_road(g, vehicle) and query_check_on_road(g, self.robot.uri):
        #print(f'{self.robot.name}: {vehicle}')

        road_vehicle = query_road(g, vehicle.uri, self.scope)
        road_current = query_road(g, self.robot.uri, self.scope)
        #print(f'road_current: {road_current}')
        #print(f'road_vehicle: {road_vehicle}')
        if road_current == URIRef("http://example.com/intersection/road_down"):
            if road_vehicle == URIRef("http://example.com/intersection/road_right"):
                g.add((vehicle.uri, EX.right_of, self.robot.uri))
        if road_current == URIRef("http://example.com/intersection/road_right"):
            if road_vehicle == URIRef("http://example.com/intersection/road_up"):
                g.add((vehicle.uri, EX.right_of, self.robot.uri))

        if road_current == URIRef("http://example.com/intersection/road_up"):
            if road_vehicle == URIRef("http://example.com/intersection/road_left"):
                g.add((vehicle.uri, EX.right_of, self.robot.uri))

        if road_current == URIRef("http://example.com/intersection/road_left"):
            if road_vehicle == URIRef("http://example.com/intersection/road_down"):
                g.add((vehicle.uri, EX.right_of, self.robot.uri))

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

    def get_intersections(self, area):
        intersection_dict = {}
        for key, value in self.map_dict.items():
            if area.intersects(value['poly']):
                intersect_area = area.intersection(value['poly']).area
                intersection_dict[key] = intersect_area
                #print(f'current_areas, {self.robot.name}: {self.current_areas}')
        return intersection_dict


    def get_max_intersection(self, area):
        intersections = self.get_intersections(area) 
        return max(intersections, key=intersections.get)

    def prediction_horizon(self, sim):
        ## de eerste grid waar je kunt wachten
        x = self.robot.pos[0]
        y = self.robot.pos[1]
        lane_width = 10
        l = self.robot.length
        H = 10
        self.horizon = None
        n = 0
        horizon_list = []

        middle_dr = sim.map.polygon_list[0]
        middle_ur = sim.map.polygon_list[1]
        middle_ul = sim.map.polygon_list[2]
        middle_dl = sim.map.polygon_list[3]
        #print(f'current pos: {self.current_pos}')

        while True:
            #print(f'n: {n}')
            #print(f'plan step: {self.plan_step}')
            if str(n+self.plan_step) not in self.plan:
                break    
            step = self.plan[str(n+self.plan_step)]
            robot_intersections = self.get_intersections(self.robot.poly)
            middle_step = False
            if step['area'] in [URIRef("http://example.com/intersection/middle_dr"), URIRef("http://example.com/intersection/middle_ur"), URIRef("http://example.com/intersection/middle_ul"), URIRef("http://example.com/intersection/middle_dl")]:
                middle_step = True

            if n==0 and step['skill'] == 'move_in_lane' and not middle_step:
                phi = step['parameters']['phi']
                new_horizon = Polygon([(x+0.5*l*math.cos(phi)-lane_width*math.sin(phi), y+0.5*l*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(0.5*l+H)*math.cos(phi)-lane_width*math.sin(phi), y+(0.5*l+H)*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(0.5*l+H)*math.cos(phi)+lane_width*math.sin(phi), y+(0.5*l+H)*math.sin(phi)+lane_width*math.cos(phi)),
                    (x+0.5*l*math.cos(phi)+lane_width*math.sin(phi), y+0.5*l*math.sin(phi)+lane_width*math.cos(phi))]).intersection(self.right_lane)
                
            elif n>0 and step['area'] and middle_step:
                new_horizon = self.map_dict[step['area']]['poly']

            elif n>0 and step['area'] and not middle_step and step['area'] in robot_intersections:
                phi = step['parameters']['phi']
                new_horizon = Polygon([(x+0.5*l*math.cos(phi)-lane_width*math.sin(phi), y+0.5*l*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(0.5*l+H)*math.cos(phi)-lane_width*math.sin(phi), y+(0.5*l+H)*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(0.5*l+H)*math.cos(phi)+lane_width*math.sin(phi), y+(0.5*l+H)*math.sin(phi)+lane_width*math.cos(phi)),
                    (x+0.5*l*math.cos(phi)+lane_width*math.sin(phi), y+0.5*l*math.sin(phi)+lane_width*math.cos(phi))]).intersection(self.right_lane)


            elif n>0 and step['area'] and not middle_step:
                if self.robot.task == 'left':
                    new_horizon = Polygon([(40-H,50),(40,50),(40,60),(40-H,60)])
                elif self.robot.task == 'up':
                    new_horizon = Polygon([(50,60),(60,60),(60,60+H),(50,60+H)])
                elif self.robot.task == 'right':
                    new_horizon = Polygon([(60,40),(60+H,40),(60+H,50),(60,50)])
                elif self.robot.task == 'down':
                    new_horizon = Polygon([(40,40-H),(50,40-H),(50,40),(40,40)])
                            
            else:
                n+=1
                continue

            if new_horizon.geom_type=='Polygon':
                horizon_list.append(new_horizon)

            if new_horizon.geom_type=='GeometryCollection':
                for polygon in new_horizon:
                    if polygon.geom_type=='Polygon':
                        #print(new_horizon.geom_type)
                        horizon_list.append(polygon)      

 
            horizon_intersections = self.get_intersections(new_horizon)
            #print(horizon_intersections)
            if step['area'] in horizon_intersections:
                if horizon_intersections[step['area']] > 95 and not middle_step:
                    break

            n+=1    

        self.robot.horizon = unary_union(horizon_list)
        




        
        

            
