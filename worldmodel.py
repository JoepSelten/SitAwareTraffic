import numpy as np
from traffic_areas import *
from rdflib import URIRef
from closure_graph import Semantics
from owlrl import DeductiveClosure
from basic_functions import *
from simplegraph3 import EX
from queries import *
from global_variables import *
from shapely.validation import make_valid
from shapely.ops import unary_union
from sys import exit
import copy

class WorldModel():
    def __init__(self, g, robot, map):
        self.init_geometric_map(map)
        self.reset(g, robot)

    def reset(self, g, robot):
        self.g = g
        self.robot = robot
        self.horizon_dict = {}
        #g.remove((self.robot.uri, None, None))
        self.g.add((self.robot.uri, RDF.type, EX.vehicle))
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
        self.skill = 'drive'
        self.wait_pos = None
        self.turn_area = None
        self.switch_phi = False
        self.turn_pos = None
        self.prev_pos = None
        self.horizon_uris = []
        self.found_waiting_area = False
        self.waiting_area = []

        self.extend_horizon = []
        self.obstacles = []
        if self.robot.start == 'down':
            self.start = URIRef("http://example.com/intersection/road_down/lane_right")
        elif self.robot.start == 'right':
            self.start = URIRef("http://example.com/intersection/road_right/lane_right")
        elif self.robot.start == 'up':
            self.start = URIRef("http://example.com/intersection/road_up/lane_right")
        elif self.robot.start == 'left':
            self.start = URIRef("http://example.com/intersection/road_left/lane_right")

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

        self.phi_before = self.map_dict[self.start].get('orientation')
        self.phi_after = self.map_dict[self.goal].get('orientation')
        self.current_pos = self.start
        #side = self.map_dict[self.side_uri].get('poly')
        #centerline = shift_line(side, -0.25*w)
        #self.extended_centerline = extend_line(centerline, self.phi_after, w)
        self.horizon_uri = []
        self.horizon_length = 1

        self.g.add((self.start, RDF.type, EX.lane_right))
        self.g.add((self.goal, RDF.type, EX.lane_right))
        self.g.add((URIRef("http://example.com/intersection/middle_dr"), RDF.type, EX.middle))
        self.g.add((URIRef("http://example.com/intersection/middle_ur"), RDF.type, EX.middle))
        self.g.add((URIRef("http://example.com/intersection/middle_ul"), RDF.type, EX.middle))
        self.g.add((URIRef("http://example.com/intersection/middle_dl"), RDF.type, EX.middle))
        DeductiveClosure(Semantics).expand(self.g)


        self.init_plan()
        self.update_av_is_on()

    def init_geometric_map(self, map):
        if map.traffic_situation == "two-lane_intersection":
            self.map_dict = {
                #URIRef("http://example.com/intersection/middle"): {'poly': map.polygon_list[0].union(map.polygon_list[1]).union(map.polygon_list[2]).union(map.polygon_list[3])},
                URIRef("http://example.com/intersection/middle_dr"): {'poly': map.polygon_list[0], 'position': [55, 45]},
                URIRef("http://example.com/intersection/middle_ur"): {'poly': map.polygon_list[1], 'position': [55, 55]},
                URIRef("http://example.com/intersection/middle_ul"): {'poly': map.polygon_list[2], 'position': [45, 55]},
                URIRef("http://example.com/intersection/middle_dl"): {'poly': map.polygon_list[3], 'position': [45, 45]},
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

        #self.plan, self.plan_left_lane = self.get_plan(self.robot.start, self.robot.task)
        self.plan = self.get_plan(self.robot.start, self.robot.task)
        
        #self.plan_step = 0

        self.right_lane = None
        self.right_lane_list = []

        for key in self.plan.keys():
            new_area = self.map_dict[key]['poly']
            self.right_lane_list.append(new_area)
        self.right_lane = unary_union(self.right_lane_list)
        

        self.left_lane = None
        self.left_lane_list = []

        # for value in self.plan_left_lane.values():
        #     new_area = self.map_dict[value['uri']]['poly']
        #     self.left_lane_list.append(new_area)
        # self.left_lane = unary_union(self.left_lane_list)


    def get_plan(self, start, task):
        plan = {}
        if start == 'down' and task == 'left':
            plan = {URIRef("http://example.com/intersection/road_down/lane_right"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_pos': URIRef("http://example.com/intersection/middle_dr")},
            URIRef("http://example.com/intersection/road_down/lane_left"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_pos': URIRef("http://example.com/intersection/middle_dl")},
            URIRef("http://example.com/intersection/middle_dr"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_pos': URIRef("http://example.com/intersection/middle_ur")},
            URIRef("http://example.com/intersection/middle_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_pos': URIRef("http://example.com/intersection/middle_ul")},
            URIRef("http://example.com/intersection/middle_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_pos': URIRef("http://example.com/intersection/road_left/lane_left")},
            URIRef("http://example.com/intersection/middle_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_pos': URIRef("http://example.com/intersection/road_left/lane_right")},
            URIRef("http://example.com/intersection/road_left/lane_right"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_pos': None},
            URIRef("http://example.com/intersection/road_left/lane_left"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_pos': None}
            }
        return plan

    def get_plan2(self, start, task):
           
        if start == 'down':
            if task == 'right':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_down/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/road_right/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_down/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'uri': URIRef("http://example.com/intersection/road_right/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}             
                }
            if task == 'up':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_down/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/road_up/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_down/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/road_up/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}                    
                }
            if task == 'left':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_down/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'uri': URIRef("http://example.com/intersection/road_left/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_down/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/road_left/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}             
                }

        if start == 'right':
            if task == 'up':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_right/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/road_up/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_right/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'uri': URIRef("http://example.com/intersection/road_up/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if task == 'left':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_right/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/road_left/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_right/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/road_left/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}               
                }
            if task == 'down':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_right/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'uri': URIRef("http://example.com/intersection/road_down/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_right/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/road_down/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}            
                }

        if start == 'up':
            if task == 'left':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_up/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/road_left/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_up/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'uri': URIRef("http://example.com/intersection/road_left/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if task == 'down':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_up/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/road_down/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_up/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/road_down/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                }
            if task == 'right':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_up/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'uri': URIRef("http://example.com/intersection/road_right/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_up/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/road_right/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }

        if start == 'left':
            if task == 'down':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_left/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/road_down/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_left/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'uri': URIRef("http://example.com/intersection/road_down/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.after, 'velocity': self.robot.velocity_max}}
                }
            if task == 'right':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_left/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/road_right/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_left/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/road_right/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
            if task == 'up':
                right_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_left/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_dl"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/middle_dr"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '3': {'uri': URIRef("http://example.com/intersection/middle_ur"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '4': {'uri': URIRef("http://example.com/intersection/road_up/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
                left_lane = {'0': {'uri': URIRef("http://example.com/intersection/road_left/lane_left"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_before, 'velocity': self.robot.velocity_max}},
                    '1': {'uri': URIRef("http://example.com/intersection/middle_ul"), 'skill': 'turn', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}},
                    '2': {'uri': URIRef("http://example.com/intersection/road_up/lane_right"), 'skill': 'move_in_lane', 'parameters': {'phi': self.phi_after, 'velocity': self.robot.velocity_max}}
                }
        return right_lane, left_lane 
        
        
    def update(self, sim):
        self.update_current_pos(sim)

        self.update_horizon(sim)

        #Associate the things that are in the robots horizon
        self.associate(sim)
        

    def update_current_pos(self, sim):
        prev_pos = self.current_pos
        self.current_pos = self.current_area()
        self.pos_list = self.get_intersections(self.robot.poly)
        for pos in self.pos_list:
            if pos in self.extend_horizon:
                self.extend_horizon.remove(pos)
        

        if self.current_pos == None:
            if self.robot.name == 'AV1':
                DeductiveClosure(Semantics).expand(self.g)
                self.g.serialize(format="json-ld", destination=self.robot.name + ".json")
                exit(0)
            else:
                self.robot.random_reset()
                self.reset(self.g, self.robot)
            return

        if self.current_pos == prev_pos:
            self.same_situation = True
        else:
            self.same_situation = False
            #self.plan_step += 1                 
            self.update_av_is_on()
            
            #self.horizon_length -= 1


    def update_horizon(self, sim):
        if self.found_waiting_area:
            self.waiting_area = self.horizon_list[-1]
        if self.horizon_dict:
            self.prev_pos = self.horizon_dict[str(len(self.horizon_dict)-1)]['poly']
        self.horizon_dict = {}
        self.update_current_horizon(sim)
        self.update_extended_horizon(sim)
        #print(self.horizon_dict)
        #if self.found_waiting_area:
        #    self.horizon_list.append(self.waiting_area)
        
            
        self.robot.horizon = unary_union(self.horizon_list)
        self.robot.horizon_dict = self.horizon_dict
    
    def update_current_horizon(self, sim):
        ## de eerste grid waar je kunt wachten
        
        x = self.robot.pos[0]
        y = self.robot.pos[1]
        lane_width = 20
        offset = 0.5*self.robot.length
        H = 10
        self.horizon_list = []
        #print(f'horizon uris: {self.horizon_uris}')

        #for horizon_uri in self.horizon_uris:
        #phi = self.plan[str(i+self.plan_step)]['parameters']['phi']
        phi = self.plan[self.current_pos]['phi']
        if self.plan[self.current_pos]['next_pos']:
            cn_pos = unary_union([self.map_dict[self.current_pos]['poly'], self.map_dict[self.plan[self.current_pos]['next_pos']]['poly']])
        else:
            cn_pos = self.map_dict[self.current_pos]['poly']
        
        horizon_poly = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
            (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
            (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
            (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)



        if horizon_poly.geom_type=='Polygon':
            self.horizon_list.append(horizon_poly)
            current_horizon_dict = {}
            current_horizon_dict['uri'] = self.current_pos
            current_horizon_dict['poly'] = horizon_poly
            current_horizon_dict['type'] = 'current'
            current_horizon_dict['color'] = 'blue'
            self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
        
        if horizon_poly.geom_type=='GeometryCollection':
            for polygon in horizon_poly:
                if polygon.geom_type=='Polygon':
                    self.horizon_list.append(horizon_poly)
                    current_horizon_dict['uri'] = self.current_pos
                    current_horizon_dict['poly'] = horizon_poly
                    current_horizon_dict['type'] = 'current'
                    current_horizon_dict['color'] = 'blue'
                    self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
        

        #self.prev_pos = copy.copy(self.robot.pos)

    def update_extended_horizon(self, sim):
        #print(f'extended horizon: {self.extend_horizon}')
        for horizon in self.extend_horizon:
            horizon_type = query_type(self.g, horizon)
            #print(horizon_type)
            if horizon_type == EX.obstacle:
                #print(self.current_pos)
                ## dit zou ook relative distance kunnen zijn
                obstacle = sim.obstacles[horizon]
                semantic_pos = self.get_max_intersection(obstacle.poly)
                x = obstacle.pos[0]
                y = obstacle.pos[1]
                lane_width = 20
                phi = self.plan[semantic_pos]['phi']
                print(semantic_pos)
                offset = -0.5*obstacle.length
                H = obstacle.length
                horizon_poly = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                    (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(self.map_dict[semantic_pos]['poly'])
                #self.horizon_list.append(horizon_poly)
                #print(horizon_poly)

                if horizon_poly.geom_type=='Polygon':
                    self.horizon_list.append(horizon_poly)
                    current_horizon_dict = {}
                    current_horizon_dict['uri'] = semantic_pos
                    current_horizon_dict['poly'] = horizon_poly
                    current_horizon_dict['position'] = [x, y]
                    current_horizon_dict['length'] = obstacle.length       
                    current_horizon_dict['color'] = 'green'          
                    self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
                
                if horizon_poly.geom_type=='GeometryCollection':
                    for polygon in horizon_poly:
                        if polygon.geom_type=='Polygon':
                            self.horizon_list.append(horizon_poly)
                            current_horizon_dict = {}
                            current_horizon_dict['uri'] = semantic_pos
                            current_horizon_dict['poly'] = horizon_poly
                            current_horizon_dict['position'] = [x, y]
                            current_horizon_dict['length'] = obstacle.length
                            current_horizon_dict['color'] = 'green'
                            self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
            
            #elif horizon.type == EX.lane:
                previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]
                #x = self.obstacles[0].pos[0]
                #y = self.obstacles[0].pos[1]
                x = previous_horizon_dict['position'][0]
                y = previous_horizon_dict['position'][1]
                lane_width = 20
                phi = self.plan[semantic_pos]['phi']
                offset = 0.5*current_horizon_dict['length']
                H = 10

                if self.plan[semantic_pos]['next_pos']:
                    cn_pos = unary_union([self.map_dict[semantic_pos]['poly'], self.map_dict[self.plan[semantic_pos]['next_pos']]['poly']])
                else:
                    cn_pos = self.map_dict[semantic_pos]['poly']

                horizon_poly = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                    (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)


                if horizon_poly.geom_type=='Polygon':
                    self.horizon_list.append(horizon_poly)
                    current_horizon_dict = {}
                    current_horizon_dict['uri'] = semantic_pos
                    current_horizon_dict['poly'] = horizon_poly
                    current_horizon_dict['color'] = 'green'
                    self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
                
                if horizon_poly.geom_type=='GeometryCollection':
                    for polygon in horizon_poly:
                        if polygon.geom_type=='Polygon':
                            self.horizon_list.append(horizon_poly)
                            current_horizon_dict = {}
                            current_horizon_dict['uri'] = semantic_pos
                            current_horizon_dict['poly'] = horizon_poly
                            current_horizon_dict['color'] = 'green'
                            self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict

            elif horizon_type == EX.middle:
                horizon_poly = self.map_dict[horizon]['poly']
                if horizon_poly.geom_type=='Polygon':
                    self.horizon_list.append(horizon_poly)
                    current_horizon_dict = {}
                    current_horizon_dict['uri'] = horizon
                    current_horizon_dict['poly'] = horizon_poly
                    current_horizon_dict['color'] = 'green'
                    self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
                
                if horizon_poly.geom_type=='GeometryCollection':
                    for polygon in horizon_poly:
                        if polygon.geom_type=='Polygon':
                            self.horizon_list.append(horizon_poly)
                            current_horizon_dict = {}
                            current_horizon_dict['uri'] = horizon
                            current_horizon_dict['poly'] = horizon_poly
                            current_horizon_dict['color'] = 'green'
                            self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict

                next_horizon_type = query_type(self.g, self.plan[horizon]['next_pos'])
                if next_horizon_type == EX.middle:
                    horizon_poly = self.map_dict[self.plan[horizon]['next_pos']]['poly']
                elif next_horizon_type == EX.lane:
                    x, y = self.map_dict[horizon]['position']
                    lane_width = 20
                    phi = self.plan[horizon]['phi']
                    offset = 5
                    H = 10

                    if self.plan[horizon]['next_pos']:
                        cn_pos = unary_union([self.map_dict[horizon]['poly'], self.map_dict[self.plan[horizon]['next_pos']]['poly']])
                    else:
                        cn_pos = self.map_dict[horizon]['poly']

                    horizon_poly = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                        (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                        (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                        (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)

                if horizon_poly.geom_type=='Polygon':
                    self.horizon_list.append(horizon_poly)
                    current_horizon_dict = {}
                    current_horizon_dict['uri'] = self.current_pos
                    current_horizon_dict['poly'] = horizon_poly
                    current_horizon_dict['color'] = 'green'
                    self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
                
                if horizon_poly.geom_type=='GeometryCollection':
                    for polygon in horizon_poly:
                        if polygon.geom_type=='Polygon':
                            self.horizon_list.append(horizon_poly)
                            current_horizon_dict = {}
                            current_horizon_dict['uri'] = self.current_pos
                            current_horizon_dict['poly'] = horizon_poly
                            current_horizon_dict['color'] = 'green'
                            self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict

            #print(horizon_poly.area)


    def associate(self, sim):
        self.associate_obstacles(sim)
        self.associate_vehicles(sim)
        self.associate_approaching_vehicles(sim)
        self.associate_approaching()

    def associate_obstacles(self, sim):
        self.g.remove((None, EX.obstructs, self.robot.uri))
        self.robot.obstructed_area = None
        for obstacle in sim.obstacles.values():
            if self.robot.horizon.intersects(obstacle.poly) and self.robot.horizon.intersection(obstacle.poly).area > 2:
                #self.obstacles.append(obstacle)
                intersection = self.robot.horizon.intersection(obstacle.poly)
                self.robot.obstructed_area = intersection
                #obstacle.semantic_pos = self.get_max_intersection(intersection)
                #print(obstacle_pos)
                self.g.add((obstacle.uri, EX.obstructs, self.robot.uri))
                self.g.add((obstacle.uri, RDF.type, EX.obstacle))
                self.g.add((obstacle.uri, RDF.type, EX.polygon))
                self.g.add((obstacle.uri, RDF.type, EX.geometry))
                #self.g.remove((obstacle_pos, EX.affordance, None))
                self.g.add((self.robot.uri, EX.approaches, obstacle.uri))
                if not self.wait_pos:
                    self.wait_pos = {}
                    self.wait_pos['poly'] = self.prev_pos
                    self.wait_pos['color'] = 'black'
                    self.robot.horizon_dict[str(len(self.robot.horizon_dict))] = self.wait_pos


    
    def associate_vehicles(self, sim):
        self.g.remove((None, EX.passes, self.robot.uri))
        self.robot.vehicle_area = None
        for robot in sim.robots.values():
            if self.robot.horizon.intersects(robot.poly) and self.robot.uri != robot.uri:
                intersection = self.robot.horizon.intersection(robot.poly)
                self.robot.vehicle_area = intersection
                self.g.add((robot.uri, EX.passes, self.robot.uri))
                #self.g.add((self.robot.uri, EX.approaches, robot.uri))


    def associate_approaching_vehicles(self, sim):
        self.g.remove((None, EX.conflict, self.robot.uri))
        self.robot.approaching_vehicle_area = None
        for robot in sim.robots.values():
            if self.robot.uri != robot.uri and self.robot.horizon and robot.horizon:
                #print(self.robot.horizon.intersection(robot.horizon).area)
                if  self.robot.horizon.intersects(robot.horizon) and self.robot.horizon.intersection(robot.horizon).area>20:
                    intersection = self.robot.horizon.intersection(robot.horizon)
                    self.robot.approaching_vehicle_area = intersection
                    self.g.add((robot.uri, EX.conflict, self.robot.uri))
                    self.associate_direction(robot)
                    #self.g.add((self.robot.uri, EX.approaches, robot.uri))

    def associate_approaching(self):
        self.robot.approaching_horizon = None
        approaching_areas = self.get_intersections(self.robot.horizon)
        for uri, area in approaching_areas.items():
            if area>5:
                print(f'approaching area: {uri}')
                self.g.add((self.robot.uri, EX.approaches, uri))
                if not self.wait_pos and self.prev_pos:
                    self.wait_pos = {}
                    self.wait_pos['poly'] = self.prev_pos
                    self.wait_pos['color'] = 'black'
                    self.robot.horizon_dict[str(len(self.robot.horizon_dict))] = self.wait_pos

    def associate_direction(self, vehicle):
        ## later generieker doen met regels die je declaratief aan een road of intersection kunt plaatsen

        road_vehicle = query_road(self.g, vehicle.uri)
        road_current = query_road(self.g, self.robot.uri)
        #print(f'road_current: {road_current}')
        #print(f'road_vehicle: {road_vehicle}')
        if road_current == URIRef("http://example.com/intersection/road_down"):
            if road_vehicle == URIRef("http://example.com/intersection/road_right"):
                self.g.add((vehicle.uri, EX.right_of, self.robot.uri))
        if road_current == URIRef("http://example.com/intersection/road_right"):
            if road_vehicle == URIRef("http://example.com/intersection/road_up"):
                self.g.add((vehicle.uri, EX.right_of, self.robot.uri))

        if road_current == URIRef("http://example.com/intersection/road_up"):
            if road_vehicle == URIRef("http://example.com/intersection/road_left"):
                self.g.add((vehicle.uri, EX.right_of, self.robot.uri))

        if road_current == URIRef("http://example.com/intersection/road_left"):
            if road_vehicle == URIRef("http://example.com/intersection/road_down"):
                self.g.add((vehicle.uri, EX.right_of, self.robot.uri))


    def update_av_is_on(self):
        self.g.remove((self.robot.uri, EX.is_on, None))
        self.g.add((self.robot.uri, EX.is_on, self.current_pos))
        #if self.current_pos in self.extend_horizon:
        #    self.extend_horizon.remove(self.current_pos)
        # if self.horizon_uris:
        #     self.horizon_uris[0] = self.current_pos
        # else:
        #     self.horizon_uris.append(self.current_pos)

    def update_is_on(self, sim):
        self.g.remove((None, EX.obstructs, self.robot.uri))
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
                        self.g.add((robot.uri, EX.obstructs, self.robot.uri))
                        self.g.add((self.robot.uri, EX.obstructs, label))
                        self.g.remove((robot.uri, EX.approaches, self.robot.uri))
                        self.robot_is_on = True

    def get_label(self, geom):
        prev_intersection = 0
        for key, value in self.map_dict.items():
            if geom.intersects(value['poly']):
                intersect_area = geom.intersection(value['poly']).area
                if intersect_area > prev_intersection:
                    new_key = key
        return new_key

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



 