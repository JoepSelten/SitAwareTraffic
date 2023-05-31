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
    def __init__(self, robot, map):
        self.init_geometric_map(map)
        self.reset(robot)
        self.sit7 = False
        self.sit3 = True

    def reset(self, robot):
        self.g = copy.deepcopy(g)
        self.robot = robot
        self.horizon_dict = {}
        self.position_dict = {}
        #g.remove((self.robot.uri, None, None))
        self.g.add((self.robot.uri, RDF.type, EX.vehicle))
        self.current_pos = None
        self.skill_selected = False
        self.skill_configured = False
        self.skill_finished = True
        #self.same_situation = True
        self.before_intersection = True
        self.approaching = False
        self.condition_failed = False
        self.wait = False
        self.horizon = None
        self.omega = 0
        self.velocity = 0
        self.skill = 'drive'
        self.plan_configured = True
        self.replan = False
        self.switch_lane_configured = False
        self.switch_phi_current = False
        self.switch_phi_next = False
        self.positions_configured = False
        self.after_obstacle_configured = False

        self.turn_area = None
        self.switch_phi = False
        self.turn_pos = None
        self.prev_pos = None
        self.prev_area = None
        self.horizon_uris = []
        self.found_waiting_area = False
        self.waiting_area = []
        self.on_left_lane = False

        self.current_wait_pos = {}
        self.next_wait_pos = {}
        self.set_current_wait_pos = False
        self.set_next_wait_pos = False

        self.before_obstacle_rl = {}
        self.before_obstacle_ll = {}
        self.after_obstacle_ll = {}
        self.after_obstacle_rl = {}
        self.set_current_turn_pos = False
        self.set_next_turn_pos = False
        self.add_next_horizon = False

        self.obstacle_on_crossing = False

        self.extend_horizon = []
        self.obstacles = []

        if self.robot.start == 'down':
            self.start = URIRef("http://example.com/intersection/road_down/lane1")
        elif self.robot.start == 'right':
            self.start = URIRef("http://example.com/intersection/road_right/lane1")
        elif self.robot.start == 'up':
            self.start = URIRef("http://example.com/intersection/road_up/lane1")
        elif self.robot.start == 'left':
            self.start = URIRef("http://example.com/intersection/road_left/lane1")

        if self.robot.task == 'down':
            self.goal = URIRef("http://example.com/intersection/road_down/lane2")
        if self.robot.task == 'right':
            self.goal = URIRef("http://example.com/intersection/road_right/lane2")
        if self.robot.task == 'up':
            self.goal = URIRef("http://example.com/intersection/road_up/lane2")
        if self.robot.task == 'left':
            self.goal = URIRef("http://example.com/intersection/road_left/lane2")


        self.phi_before = self.map_dict[self.start].get('orientation')
        self.phi_after = self.map_dict[self.goal].get('orientation')
        self.current_pos = self.start

        self.horizon_uri = []
        self.horizon_length = 1
        self.g.add((self.start, RDF.type, EX.lane_right))
        self.g.add((self.goal, RDF.type, EX.lane_right))

        lane_right_affordances = query_affordances(self.g, EX.lane_right)
        #input(lane_right_affordances)
        for affordance in lane_right_affordances:
            self.g.add((self.start, EX.affordance, affordance))
            self.g.add((self.goal, EX.affordance, affordance))
        ## deze moeten miss ook type lane_right en lane_left
        self.g.add((URIRef("http://example.com/intersection/crossing_dr"), RDF.type, EX.crossing))
        self.g.add((URIRef("http://example.com/intersection/crossing_ur"), RDF.type, EX.crossing))
        self.g.add((URIRef("http://example.com/intersection/crossing_ul"), RDF.type, EX.crossing))
        self.g.add((URIRef("http://example.com/intersection/crossing_dl"), RDF.type, EX.crossing))

        self.g.add((URIRef("http://example.com/intersection/crossing_dr"), EX.affordance, EX.drivable))
        self.g.add((URIRef("http://example.com/intersection/crossing_ur"), EX.affordance, EX.drivable))
        self.g.add((URIRef("http://example.com/intersection/crossing_ul"), EX.affordance, EX.drivable))
        self.g.add((URIRef("http://example.com/intersection/crossing_dl"), EX.affordance, EX.drivable))
        
        self.init_plan()
        #DeductiveClosure(Semantics).expand(self.g)
        
        self.update_av_is_on()

    def init_geometric_map(self, map):
        if map.traffic_situation == "two-lane_intersection":
            self.map_dict = {
                #URIRef("http://example.com/intersection/crossing"): {'polygon': map.polygon_list[0].union(map.polygon_list[1]).union(map.polygon_list[2]).union(map.polygon_list[3])},
                URIRef("http://example.com/intersection/crossing_dr"): {'polygon': map.polygon_list[0], 'position': [55, 45], 'weight': 0},
                URIRef("http://example.com/intersection/crossing_ur"): {'polygon': map.polygon_list[1], 'position': [55, 55], 'weight': 0},
                URIRef("http://example.com/intersection/crossing_ul"): {'polygon': map.polygon_list[2], 'position': [45, 55], 'weight': 0},
                URIRef("http://example.com/intersection/crossing_dl"): {'polygon': map.polygon_list[3], 'position': [45, 45], 'weight': 0},
                URIRef("http://example.com/intersection/road_down/lane1"): {'polygon': map.polygon_list[4], 'orientation': 0.5*math.pi, 'weight': 0},
                URIRef("http://example.com/intersection/road_down/lane2"): {'polygon': map.polygon_list[5], 'orientation': -0.5*math.pi, 'weight': 0},
                URIRef("http://example.com/intersection/road_right/lane1"): {'polygon': map.polygon_list[6], 'orientation': math.pi, 'weight': 0},
                URIRef("http://example.com/intersection/road_right/lane2"): {'polygon': map.polygon_list[7], 'orientation': 0, 'weight': 0},
                URIRef("http://example.com/intersection/road_up/lane1"): {'polygon': map.polygon_list[8], 'orientation': -0.5*math.pi, 'weight': 0},
                URIRef("http://example.com/intersection/road_up/lane2"): {'polygon': map.polygon_list[9], 'orientation': 0.5*math.pi, 'weight': 0}, 
                URIRef("http://example.com/intersection/road_left/lane1"): {'polygon': map.polygon_list[10], 'orientation': 0, 'weight': 0},
                URIRef("http://example.com/intersection/road_left/lane2"): {'polygon': map.polygon_list[11], 'orientation': math.pi, 'weight': 0}
            }

        else:
            self.map_dict = {}
            print("Unknown map!")

    def init_plan(self):
        self.plan = self.get_plan(self.robot.start, self.robot.task)
        
        self.right_lane = None
        self.right_lane_list = []

        for key in self.plan.keys():
            new_area = self.map_dict[key]['polygon']
            self.right_lane_list.append(new_area)
        self.right_lane = unary_union(self.right_lane_list)
        

        self.left_lane = None
        self.left_lane_list = []


    def get_plan(self, start, task):
        ## dit configureert gelijk wat de right en left lane is
        plan = {}
        lane_left_affordances = query_affordances(self.g, EX.lane_left)
        if start == 'down' and task == 'left':
            plan = {URIRef("http://example.com/intersection/road_down/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/road_down/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane2")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane1")},
            URIRef("http://example.com/intersection/road_left/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_left/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_down/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_left/lane1"), RDF.type, EX.lane_left))
            
            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_down/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_left/lane1"), EX.affordance, affordance))

        if start == 'down' and task == 'right':
            
            plan = {URIRef("http://example.com/intersection/road_down/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/road_down/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane2")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane1")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/road_right/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_right/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_down/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_right/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_down/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_right/lane1"), EX.affordance, affordance))

        if start == 'down' and task == 'up':
            
            plan = {URIRef("http://example.com/intersection/road_down/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/road_down/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_up/lane2")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_up/lane1")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/road_up/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_up/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_down/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_up/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_down/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_up/lane1"), EX.affordance, affordance))


        if start == 'right' and task == 'left':
            plan = {URIRef("http://example.com/intersection/road_right/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/road_right/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane2")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane1")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/road_left/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_left/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_right/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_left/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_right/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_left/lane1"), EX.affordance, affordance))

        if start == 'right' and task == 'up':
            plan = {URIRef("http://example.com/intersection/road_right/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/road_right/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_up/lane1")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_up/lane2")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/road_up/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_up/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_right/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_up/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_right/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_up/lane1"), EX.affordance, affordance))

        if start == 'right' and task == 'down':
            plan = {URIRef("http://example.com/intersection/road_right/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/road_right/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_down/lane2")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_down/lane1")},
            URIRef("http://example.com/intersection/road_down/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_down/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_right/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_down/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_right/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_up/lane1"), EX.affordance, affordance))


        if start == 'left' and task == 'down':
            plan = {URIRef("http://example.com/intersection/road_left/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/road_left/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_down/lane2")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_down/lane1")},
            URIRef("http://example.com/intersection/road_down/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_down/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_left/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_down/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_left/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_down/lane1"), EX.affordance, affordance))

        if start == 'left' and task == 'up':
            plan = {URIRef("http://example.com/intersection/road_left/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/road_left/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_up/lane1")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_up/lane2")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/road_up/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_up/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_left/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_up/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_left/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_down/lane1"), EX.affordance, affordance))

        if start == 'left' and task == 'right':
            plan = {URIRef("http://example.com/intersection/road_left/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/road_left/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane1")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane2")},
            URIRef("http://example.com/intersection/road_right/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_right/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_left/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_right/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_left/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_right/lane1"), EX.affordance, affordance))

        if start == 'up' and task == 'right':
            plan = {URIRef("http://example.com/intersection/road_up/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/road_up/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane1")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane2")},
            URIRef("http://example.com/intersection/road_right/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_right/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_up/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_right/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_up/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_right/lane1"), EX.affordance, affordance))

        if start == 'up' and task == 'left':
            plan = {URIRef("http://example.com/intersection/road_up/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/road_up/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane2")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane1")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/road_left/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_left/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_up/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_left/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_up/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_left/lane1"), EX.affordance, affordance))

        if start == 'up' and task == 'down':
            plan = {URIRef("http://example.com/intersection/road_up/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/road_up/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_down/lane2")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_down/lane1")},
            URIRef("http://example.com/intersection/road_down/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_down/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_up/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_down/lane1"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_up/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_down/lane1"), EX.affordance, affordance))

        return plan        
        
    def update(self, sim):
        ## associate is-on       
        self.update_current_pos()

        ## save positions from previous horizon that might be important for configuring behaviours
        self.save_positions()

        ## update horizon
        self.update_horizon(sim)

        ## associate the things that are in the robots horizon
        self.associate(sim)
        

    def update_current_pos(self):
        self.intersect_dict = self.get_intersections(self.map_dict, self.robot.polygon)
        self.weight_dict = {}
        for uri, intersection in self.intersect_dict.items():
            if intersection > 0.4*self.robot.polygon.area:
                self.weight_dict[uri] = self.map_dict[uri]['weight']

        if self.weight_dict:
            self.current_pos = max(self.weight_dict, key=self.weight_dict.get)
            self.update_av_is_on()
        else:
            self.current_pos = None
            self.robot.random_reset()
            self.reset(self.robot)            


    def update_av_is_on(self):
        self.g.remove((self.robot.uri, EX.is_on, None))
        self.g.add((self.robot.uri, EX.is_on, self.current_pos))
        self.robot.pos_uri = self.current_pos


    def update_horizon(self, sim):
        ## if the path is reconfigured, the horizon resets
        if self.replan:
            self.horizon_dict = {}
            self.horizon_dict = copy.deepcopy(self.horizon_before_turn)
            self.horizon_dict[str(len(self.horizon_dict))] = self.before_obstacle_ll
            self.replan = False
            self.on_left_lane = True

        ## update the horizon that is directly in front of the AV
        self.update_current_horizon(sim)

        ## extend the horizon if necessary
        if self.extend_horizon:
            self.update_extended_horizon(sim)

        ## deleting the part of the extended horizon that is intersect by the newly added horizon in front of the AV
        self.delete_extended_horizon(sim)


        self.robot.horizon_dict = self.horizon_dict

        self.horizon_list = []
        for key, value in self.horizon_dict.items():
            self.horizon_list.append(value['polygon'])
        self.robot.horizon = unary_union(self.horizon_list)

    
    def save_positions(self):
        if self.after_obstacle_rl and self.current_pos==self.after_obstacle_rl['uri']:
            self.plan[URIRef("http://example.com/intersection/road_down/lane1")]['next_position'] = URIRef("http://example.com/intersection/crossing_dr")
            #self.plan[URIRef("http://example.com/intersection/road_right/lane2")]['next_position'] = URIRef("http://example.com/intersection/crossing_dr")
            self.map_dict[self.after_obstacle_rl['uri']]['weight'] = -1
            self.set_current_wait_pos = False
            self.current_wait_pos = {}
            self.next_wait_pos = {}
            self.set_current_turn_pos = False
            self.set_next_wait_pos = False 
            self.set_next_turn_pos = False          

        if self.set_next_wait_pos and self.plan_configured and self.after_obstacle_rl and self.after_obstacle_rl['uri'] not in self.intersect_dict:
            self.map_dict[self.after_obstacle_rl['uri']]['weight'] = 1

        if self.horizon_dict:
            self.current_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]

        if self.set_current_wait_pos and not self.current_wait_pos:
            self.current_wait_pos['polygon'] = self.prev_dict['polygon']
            self.current_wait_pos['position'] = self.prev_dict['position']
            self.current_wait_pos['semantic_position'] = self.get_intersections(self.map_dict, self.current_wait_pos['polygon'])
            self.current_wait_pos['uri'] = self.prev_dict['uri']
            self.current_wait_pos['color'] = 'orange'
            self.current_wait_pos['type'] = 'wait_pos'
            self.position_dict[str(len(self.position_dict))] = self.current_wait_pos

                    
        if self.set_next_wait_pos and not self.next_wait_pos:
            self.next_wait_pos['polygon'] = self.current_dict['polygon']
            self.next_wait_pos['position'] = self.current_dict['position']
            self.next_wait_pos['semantic_position'] = self.get_intersections(self.map_dict, self.next_wait_pos['polygon'])
            self.next_wait_pos['uri'] = self.current_dict['uri']
            self.next_wait_pos['color'] = 'orange'
            self.next_wait_pos['type'] = 'next_wait_pos'
            self.position_dict[str(len(self.position_dict))] = self.next_wait_pos


        if self.set_current_turn_pos and not self.before_obstacle_rl:
            self.before_obstacle_rl['polygon'] = self.prev_dict['polygon']
            self.before_obstacle_rl['position'] = self.prev_dict['position']
            self.before_obstacle_rl['semantic_position'] = self.get_intersections(self.map_dict, self.before_obstacle_rl['polygon'])
            self.before_obstacle_rl['uri'] =  EX.before_obstacle_rl
            #print(self.before_obstacle_rl['semantic_position'])
            for key, value in self.before_obstacle_rl['semantic_position'].items():
                if value > 60:
                    self.g.add((key, EX.has_a, self.before_obstacle_rl['uri']))
            self.before_obstacle_rl['color'] = 'red'
            self.before_obstacle_rl['type'] = 'before_right'
            self.position_dict[str(len(self.position_dict))] = self.before_obstacle_rl
            # print(self.prev_dict['type'])
            # print(self.current_dict['type'])
            self.horizon_before_turn = copy.deepcopy(self.horizon_dict)
            #print(self.current_dict['type'])
            
            if self.current_dict['type'] == 'crossing':
                self.horizon_before_turn.pop(str(len(self.horizon_dict)-1))
            #print(self.horizon_before_turn)
           
            lane_type = query_type(self.g, self.prev_dict['uri'])
            lane_affordances = query_affordances(self.g, self.prev_dict['uri'])
            self.g.add((self.before_obstacle_rl['uri'], RDF.type, lane_type))
            for affordance in lane_affordances:
                self.g.add((self.before_obstacle_rl['uri'], EX.affordance, affordance))

            self.before_obstacle_ll['uri'] = EX.before_obstacle_ll

            ## de lane uri is nu nog max intersection
            self.before_obstacle_ll['polygon'], self.before_obstacle_ll['position'], lane_uri = self.get_left_lane(self.before_obstacle_rl)
            self.before_obstacle_ll['semantic_position'] = self.get_intersections(self.map_dict, self.before_obstacle_ll['polygon'])
            self.before_obstacle_ll['color'] = 'grey'
            self.before_obstacle_ll['type'] = 'before_left'
            self.position_dict[str(len(self.position_dict))] = self.before_obstacle_ll
            
            lane_type = query_type(self.g, lane_uri)
            lane_affordances = query_affordances(self.g, lane_uri)
            self.g.add((self.before_obstacle_ll['uri'], RDF.type, lane_type))
            for affordance in lane_affordances:
                self.g.add((self.before_obstacle_ll['uri'], EX.affordance, affordance))
                



            self.g.add((self.before_obstacle_rl['uri'], EX.connects, self.before_obstacle_ll['uri']))
            #DeductiveClosure(Semantics).expand(self.g)
            
            

        if self.set_next_turn_pos and not self.after_obstacle_rl:
            #input(self.before_obstacle_rl)
            self.plan[URIRef("http://example.com/intersection/road_down/lane1")]['next_position'] = self.before_obstacle_rl['uri']
          
            
            plan_step = copy.deepcopy(self.plan[URIRef("http://example.com/intersection/road_down/lane1")])
            plan_step['next_position'] = self.before_obstacle_ll['uri']
            self.plan[self.before_obstacle_rl['uri']] = plan_step
            new_map = {}
            new_map['polygon'] = self.before_obstacle_rl['polygon']
            new_map['position'] = self.before_obstacle_rl['position']
            new_map['weight'] = 1
            self.map_dict[self.before_obstacle_rl['uri']] = new_map

            if self.sit7:
                plan_step = copy.deepcopy(self.plan[URIRef("http://example.com/intersection/road_right/lane1")])
                plan_step['next_position'] = URIRef("http://example.com/intersection/road_right/lane1")
            else:
                plan_step = copy.deepcopy(self.plan[URIRef("http://example.com/intersection/road_down/lane2")])
                plan_step['next_position'] = URIRef("http://example.com/intersection/road_down/lane2")
            self.plan[self.before_obstacle_ll['uri']] = plan_step
            new_map = {}
            new_map['polygon'] = self.before_obstacle_ll['polygon']
            new_map['position'] = self.before_obstacle_ll['position']
            new_map['weight'] = 1
            self.map_dict[self.before_obstacle_ll['uri']] = new_map


            self.after_obstacle_rl['polygon'] = self.current_dict['polygon']
            self.after_obstacle_rl['position'] = self.current_dict['position']
            self.after_obstacle_rl['semantic_position'] = self.current_dict['semantic_position']
            self.after_obstacle_rl['max_semantic_position'] = self.current_dict['max_semantic_position']
            # max(self.current_dict['semantic_position'], key=self.current_dict['semantic_position'].get)
            # #print(f"last horizon pos: {self.horizon_dict[str(len(self.horizon_dict)-1)]['position']}")
            # input('hoi')
            self.after_obstacle_rl['uri'] = EX.after_obstacle_rl
            self.after_obstacle_rl['color'] = 'yellow'
            self.after_obstacle_rl['type'] = 'after_right'
            self.position_dict[str(len(self.position_dict))] = self.after_obstacle_rl

            lane_type = query_type(self.g, self.after_obstacle_rl['max_semantic_position'])
            lane_affordances = query_affordances(self.g, self.after_obstacle_rl['max_semantic_position'])
            self.g.add((self.after_obstacle_rl['uri'], RDF.type, lane_type))
            for affordance in lane_affordances:
                self.g.add((self.after_obstacle_rl['uri'], EX.affordance, affordance))
            
            plan_step = copy.deepcopy(self.plan[self.current_dict['uri']])
            #print(f"next pos: {self.current_dict['max_semantic_position']}")
            plan_step['next_position'] = self.current_dict['max_semantic_position']
            #plan_step['next_position'] = self.plan[self.current_dict['uri']]['next_position']
            self.plan[self.after_obstacle_rl['uri']] = plan_step
            new_map = {}
            new_map['polygon'] = self.after_obstacle_rl['polygon']
            new_map['position'] = self.after_obstacle_rl['position']
            new_map['weight'] = 1
            self.map_dict[self.after_obstacle_rl['uri']] = new_map

            self.after_obstacle_ll['polygon'], self.after_obstacle_ll['position'], lane_uri = self.get_left_lane(self.after_obstacle_rl)
            self.after_obstacle_ll['semantic_position'] = self.get_intersections(self.map_dict, self.after_obstacle_ll['polygon'])
            self.after_obstacle_ll['color'] = 'blue'
            self.after_obstacle_ll['type'] = 'after_left'
            self.after_obstacle_ll['uri'] = EX.after_obstacle_ll
            self.position_dict[str(len(self.position_dict))] = self.after_obstacle_ll
            
            self.replan = True
            self.plan[lane_uri]['next_position'] = self.after_obstacle_ll['uri']

            lane_type = query_type(self.g, lane_uri)
            lane_affordances = query_affordances(self.g, lane_uri)
            self.g.add((self.after_obstacle_ll['uri'], RDF.type, lane_type))
            for affordance in lane_affordances:
                self.g.add((self.after_obstacle_ll['uri'], EX.affordance, affordance))
      

            #print(lane_uri)
            #input('hoi')
            plan_step = copy.deepcopy(self.plan[lane_uri])
            plan_step['next_position'] = EX.after_obstacle_rl
            self.plan[EX.after_obstacle_ll] = plan_step
            new_map = {}
            new_map['polygon'] = self.after_obstacle_ll['polygon']
            new_map['position'] = self.after_obstacle_ll['position']
            new_map['weight'] = 1
            self.map_dict[EX.after_obstacle_ll] = new_map
            self.g.add((EX.after_obstacle_ll, EX.connects, EX.after_obstacle_rl))

            self.positions_configured = True
            #DeductiveClosure(Semantics).expand(self.g)
            #print(self.plan)

        self.robot.position_dict = self.position_dict


        if self.horizon_dict:
            self.prev_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]

    def get_left_lane(self, next_drivable_pos):
        #print(f'next_drivable_pos: {next_drivable_pos}')
        max_intersect = self.get_max_intersection(self.map_dict, next_drivable_pos['polygon'])
        phi = self.plan[max_intersect]['phi']
        x = next_drivable_pos['position'][0]
        y = next_drivable_pos['position'][1]
        relative_phi = phi+0.5*math.pi

        offset = 5
        H = 10
        lane_width = 5

        horizon_polygon = Polygon([(x+offset*math.cos(relative_phi)-lane_width*math.sin(relative_phi), y+offset*math.sin(relative_phi)-lane_width*math.cos(relative_phi)),
            (x+(offset+H)*math.cos(relative_phi)-lane_width*math.sin(relative_phi), y+(offset+H)*math.sin(relative_phi)-lane_width*math.cos(relative_phi)),
            (x+(offset+H)*math.cos(relative_phi)+lane_width*math.sin(relative_phi), y+(offset+H)*math.sin(relative_phi)+lane_width*math.cos(relative_phi)),
            (x+offset*math.cos(relative_phi)+lane_width*math.sin(relative_phi), y+offset*math.sin(relative_phi)+lane_width*math.cos(relative_phi))])

        pos_x = x + (offset+0.5*H)*math.cos(relative_phi)
        pos_y = y + (offset+0.5*H)*math.sin(relative_phi)
        horizon_pos = [pos_x, pos_y]

        horizon_uri = self.get_max_intersection(self.map_dict, horizon_polygon)

        return horizon_polygon, horizon_pos, horizon_uri
    
    def update_current_horizon(self, sim):
        x = self.robot.pos[0]
        y = self.robot.pos[1]
        lane_width = 20
        offset = 0.5*self.robot.length
        H = 10

        phi = self.plan[self.current_pos]['phi']
        current_plan = []
        current_plan.append(self.map_dict[self.current_pos]['polygon'])
        pos = self.current_pos
        while self.plan[pos]['next_position']:
            current_plan.append(self.map_dict[self.plan[pos]['next_position']]['polygon'])
            if len(current_plan)>1:
                break
            pos = self.plan[pos]['next_position']
        cn_pos = unary_union(current_plan)
            
        horizon_polygon = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
            (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
            (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
            (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)

        pos_x = x + (0.5*self.robot.length+0.5*H)*math.cos(phi)
        pos_y = y + (0.5*self.robot.length+0.5*H)*math.sin(phi)
        horizon_pos = [pos_x, pos_y]

        self.add_horizon(self.current_pos, horizon_polygon, horizon_pos, H, 'current', 'blue')

    def update_extended_horizon(self, sim):
        max_weight = -5
        for ext_horizon in self.extend_horizon:
            weight = self.map_dict[ext_horizon]['weight']
            if weight >= max_weight:
                max_weight = weight
                horizon = ext_horizon

        horizon_type = query_type(self.g, horizon)
        previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]
        if horizon_type == EX.obstacle:
            obstacle = sim.obstacles[horizon]
            semantic_pos = self.get_max_intersection(self.map_dict, obstacle.polygon)
            x = obstacle.pos[0]
            y = obstacle.pos[1]
            lane_width = 20
            phi = self.plan[semantic_pos]['phi']
            offset = -0.5*obstacle.length
            H = obstacle.length
            horizon_polygon = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(self.map_dict[semantic_pos]['polygon'])

            horizon_pos = [x, y]
            self.add_horizon(semantic_pos, horizon_polygon, horizon_pos, obstacle.length, 'obstacle')

            type_semantic_pos = query_type(self.g, semantic_pos)

            if type_semantic_pos == EX.lane_right or type_semantic_pos == EX.lane_left:
                previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]
                x = previous_horizon_dict['position'][0]
                y = previous_horizon_dict['position'][1]
                lane_width = 20
                phi = self.plan[semantic_pos]['phi']
                offset = 0.5*previous_horizon_dict['length']
                H = 10

                current_plan = []
                current_plan.append(self.map_dict[semantic_pos]['polygon'])
                pos = semantic_pos
                while self.plan[pos]['next_position']:
                    current_plan.append(self.map_dict[self.plan[pos]['next_position']]['polygon'])
                    if len(current_plan)>1:
                        break
                    pos = self.plan[pos]['next_position']
                cn_pos = unary_union(current_plan)

                horizon_polygon = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                    (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                    (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)

                pos_x = x + (0.5*previous_horizon_dict['length']+0.5*H)*math.cos(phi)
                pos_y = y + (0.5*previous_horizon_dict['length']+0.5*H)*math.sin(phi)
                horizon_pos = [pos_x, pos_y]

                
            elif type_semantic_pos == EX.crossing:
                self.obstacle_on_crossing = True
                next_horizon = self.plan[semantic_pos]['next_position']
                next_horizon_type = query_type(self.g, next_horizon)
                
                if next_horizon_type == EX.crossing:
                    horizon_polygon = self.map_dict[next_horizon]['polygon']
                    
                    horizon_pos = self.map_dict[next_horizon]['position']
                    horizon_length = 10
                elif next_horizon_type == EX.lane_left or next_horizon_type == EX.lane_right:
                    x, y = self.map_dict[semantic_pos]['position']
                    lane_width = 20
                    phi = self.plan[semantic_pos]['phi']
                    offset = 5
                    H = 10
                    if self.plan_configured:
                        self.add_next_horizon = False
                    

                    if self.plan[semantic_pos]['next_position']:
                        cn_pos = unary_union([self.map_dict[semantic_pos]['polygon'], self.map_dict[self.plan[semantic_pos]['next_position']]['polygon']])
                    else:
                        cn_pos = self.map_dict[semantic_pos]['polygon']

                    horizon_polygon = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                        (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                        (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                        (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)

                    pos_x = x + (0.5*previous_horizon_dict['length']+0.5*H)*math.cos(phi)
                    pos_y = y + (0.5*previous_horizon_dict['length']+0.5*H)*math.sin(phi)
                    horizon_pos = [pos_x, pos_y]
                    horizon_length = H
            self.add_horizon(semantic_pos, horizon_polygon, horizon_pos, H, 'next_horizon')


        elif horizon_type == EX.crossing and (self.plan_configured or self.obstacle_on_crossing or horizon==EX.before_obstacle_ll):
            crossing_intersect = previous_horizon_dict['semantic_position'].get(URIRef("http://example.com/intersection/crossing_dr"), 0) + \
                previous_horizon_dict['semantic_position'].get(URIRef("http://example.com/intersection/crossing_ur"), 0) + \
                    previous_horizon_dict['semantic_position'].get(URIRef("http://example.com/intersection/crossing_dl"), 0) + \
                    previous_horizon_dict['semantic_position'].get(URIRef("http://example.com/intersection/crossing_ul"), 0)
            if crossing_intersect < 95:
                if horizon == EX.after_obstacle_rl:
                    horizon = URIRef("http://example.com/intersection/crossing_dr")

                horizon_polygon = self.map_dict[horizon]['polygon']
                horizon_pos = self.map_dict[horizon]['position']
                horizon_length = 10
                self.add_next_horizon = True
            
            else:
                previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]
                next_horizon_type = query_type(self.g, self.plan[horizon]['next_position'])
                if next_horizon_type == EX.crossing:
                    horizon_polygon = self.map_dict[self.plan[horizon]['next_position']]['polygon']
                    horizon_pos = self.map_dict[self.plan[horizon]['next_position']]['position']
                    horizon_length = 10
                elif next_horizon_type == EX.lane_left or next_horizon_type == EX.lane_right:
                    x, y = self.map_dict[horizon]['position']
                    lane_width = 20
                    phi = self.plan[horizon]['phi']
                    offset = 5
                    H = 10
                    if self.plan_configured:
                        self.add_next_horizon = False
                    

                    if self.plan[horizon]['next_position']:
                        cn_pos = unary_union([self.map_dict[horizon]['polygon'], self.map_dict[self.plan[horizon]['next_position']]['polygon']])
                    else:
                        cn_pos = self.map_dict[horizon]['polygon']

                    horizon_polygon = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                        (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                        (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                        (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)

                    pos_x = x + (0.5*H+0.5*H)*math.cos(phi) # first H was previous_horizon_dict['position']
                    pos_y = y + (0.5*H+0.5*H)*math.sin(phi) # first H was previous_horizon_dict['position']
                    horizon_pos = [pos_x, pos_y]
                    horizon_length = H

            self.add_horizon(horizon, horizon_polygon, horizon_pos, horizon_length, 'crossing')           

        elif horizon_type==EX.lane_left and not horizon == EX.after_obstacle_ll:
            previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]
            x, y = previous_horizon_dict['position']
            lane_width = 20
            phi = self.plan[horizon]['phi']
            offset = 5
            H = 10

            current_plan = []
            current_plan.append(self.map_dict[horizon]['polygon'])
            pos = horizon
            while self.plan[pos]['next_position']:
                current_plan.append(self.map_dict[self.plan[pos]['next_position']]['polygon'])
                if len(current_plan)>2:
                    break
                pos = self.plan[pos]['next_position']
            cn_pos = unary_union(current_plan)

            horizon_polygon = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)
                
            if horizon == EX.before_obstacle_ll:
                self.map_dict[horizon]['weight'] = -2

            pos_x = x + (0.5*H+0.5*H)*math.cos(phi)
            pos_y = y + (0.5*H+0.5*H)*math.sin(phi)
            horizon_pos = [pos_x, pos_y]
            horizon_length = H
            horizon_type = 'left_lane'

            self.add_horizon(horizon, horizon_polygon, horizon_pos, horizon_length, horizon_type)

        elif horizon == EX.after_obstacle_ll:
            horizon_polygon = self.map_dict[horizon]['polygon']
            horizon_pos = self.map_dict[horizon]['position']
            self.add_horizon(horizon, horizon_polygon, horizon_pos, 10, 'after_obstacle')


            next_horizon = self.plan[horizon]['next_position']
            horizon_polygon = self.map_dict[next_horizon]['polygon']
            horizon_pos = self.map_dict[next_horizon]['position']

            self.add_horizon(next_horizon, horizon_polygon, horizon_pos, 10, 'after_obstacle')
            self.map_dict[next_horizon]['weight'] = -1
            self.plan_configured = True
            self.after_obstacle_configured =  True
            if not self.sit7:
                self.plan[self.before_obstacle_rl['uri']]['phi'] += 0.5*math.pi
            if self.sit3:
                self.plan[self.after_obstacle_ll['uri']]['phi'] -= math.pi
            else:
                self.plan[self.after_obstacle_ll['uri']]['phi'] -= 0.5*math.pi

        self.extend_horizon = []


    def add_horizon(self, uri, polygon, position, length, type='normal', color='green'):
        current_horizon_dict = {}
        polygon_list = []
        if polygon.geom_type=='GeometryCollection':
            for polygon in polygon:
                if polygon.geom_type=='Polygon':
                    polygon_list.append(polygon)
        elif polygon.geom_type=='Polygon':
            polygon_list.append(polygon)

        for poly in polygon_list:
            current_horizon_dict['uri'] = uri
            current_horizon_dict['polygon'] = poly
            current_horizon_dict['semantic_position'] = self.get_intersections(self.map_dict, poly)
            if current_horizon_dict['semantic_position']:
                current_horizon_dict['max_semantic_position'] = max(current_horizon_dict['semantic_position'], key=current_horizon_dict['semantic_position'].get)
            current_horizon_dict['position'] = position
            current_horizon_dict['length'] = length
            current_horizon_dict['type'] = type
            current_horizon_dict['color'] = color
            if type=='current':
                self.horizon_dict['0'] = current_horizon_dict
            else:
                self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
        

    def delete_extended_horizon(self, sim):
        new_horizon_dict = copy.deepcopy(self.horizon_dict)
        new_horizon_dict = {}
        for key, horizon in self.horizon_dict.items():
            if self.robot.polygon.intersects(horizon['polygon']) and self.robot.polygon.intersection(horizon['polygon']).area > 2:
                continue
            else:
                new_horizon_dict[str(len(new_horizon_dict))] = horizon

        self.horizon_dict = new_horizon_dict

    def associate(self, sim):
        self.associate_obstacles(sim)
        self.associate_vehicles(sim)
        self.associate_approaching_vehicles(sim)
        self.associate_approaching()

    def associate_obstacles(self, sim):
        self.g.remove((self.robot.uri, EX.approaches, None))
        approaching_obstacles = self.get_intersections(sim.obstacles_dict, self.robot.horizon_dict[str(len(self.robot.horizon_dict)-1)]['polygon'])
        for uri, area in approaching_obstacles.items():
            if area>2:  # extra margin
                new_map = {}
                new_map['polygon'] = sim.obstacles_dict[uri]['polygon']
                new_map['weight'] = 1
                self.map_dict[uri] = new_map
                
                self.g.add((self.robot.uri, EX.approaches, uri))
                self.g.add((uri, RDF.type, EX.obstacle))
                self.plan_configured = False
    
    def associate_vehicles(self, sim):
        #self.g.remove((None, EX.passes, self.robot.uri))
        self.robot.vehicle_area = None
        for robot in sim.robots.values():
            self.g.remove((robot.uri, EX.is_on, None))
            self.g.add((robot.uri, EX.is_on, robot.pos_uri))
            if self.robot.horizon.intersects(robot.polygon) and self.robot.uri != robot.uri:
                intersection = self.robot.horizon.intersection(robot.polygon)
                self.robot.vehicle_area = intersection
                self.g.add((robot.uri, EX.passes, self.robot.uri))
                self.g.add((self.robot.uri, EX.approaches, robot.uri))


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

                    self.associate_right_of(robot)
                    #self.g.add((self.robot.uri, EX.approaches, robot.uri))

    def associate_approaching(self):
        approaching_areas = self.get_intersections(self.map_dict, self.robot.horizon_dict[str(len(self.robot.horizon_dict)-1)]['polygon'])
        for uri, area in approaching_areas.items():
            if area>5:
                self.g.add((self.robot.uri, EX.approaches, uri))


    def associate_right_of(self, vehicle):
        ## later generieker doen met regels die je declaratief aan een road of intersection kunt plaatsen

        road_vehicle = query_road(self.g, vehicle.uri)
        road_current = query_road(self.g, self.robot.uri)

        #print(road_current)
        #print(road_vehicle)
        #input('asdf')

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
                if robot.polygon:
                    if self.robot.horizon.intersects(robot.polygon):
                        self.robots_is_on.append(robot)
                        self.robot.is_on_horizon = self.robot.horizon.intersection(robot.polygon)
                        ##  dit kunnen er later ook meer zijn
                        label = self.get_label(self.robot.is_on_horizon)
                        self.robot.obstructed_area = self.map_dict[label]['polygon']
                        self.g.add((robot.uri, EX.obstructs, self.robot.uri))
                        self.g.add((self.robot.uri, EX.obstructs, label))
                        self.g.remove((robot.uri, EX.approaches, self.robot.uri))
                        self.robot_is_on = True

    def get_label(self, geom):
        prev_intersection = 0
        for key, value in self.map_dict.items():
            if geom.intersects(value['polygon']):
                intersect_area = geom.intersection(value['polygon']).area
                if intersect_area > prev_intersection:
                    new_key = key
        return new_key

    def current_area(self):
        self.current_areas = {}
        total = 0
        for key, value in self.map_dict.items():
            if self.robot.polygon.intersects(value['polygon']):
                intersect_area = self.robot.polygon.intersection(value['polygon']).area
                self.current_areas[key] = intersect_area
                total += intersect_area
        if total > 0.98*self.robot.polygon.area:
            #print(f'current_areas: {self.current_areas}')
            return max(self.current_areas, key=self.current_areas.get)


    def get_intersections(self, dict, area):
        intersection_dict = {}
        for key, value in dict.items():
            if area.intersects(value['polygon']):
                intersect_area = area.intersection(value['polygon']).area
                intersection_dict[key] = intersect_area
                #print(f'current_areas: {self.current_areas}')
        return intersection_dict


    def get_max_intersection(self, dict, area):
        intersections = self.get_intersections(dict, area) 
        return max(intersections, key=intersections.get)



 