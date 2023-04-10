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
        self.same_situation = True
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

            #tmp = self.map_dict[URIRef("http://example.com/intersection/road_down/lane2")]
            #self.map_dict[URIRef("http://example.com/intersection/road_down/lane2")] = self.map_dict[URIRef("http://example.com/intersection/road_down/lane1")]
            #self.map_dict[URIRef("http://example.com/intersection/road_down/lane1")] = tmp
            #input(self.map_dict)
            #self.map_dict[URIRef("http://example.com/intersection/road_down/lane_left")] = self.map_dict.pop(URIRef("http://example.com/intersection/road_down/lane2"))

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
            self.goal = URIRef("http://example.com/intersection/road_left/lane2")
            #self.map_dict[URIRef("http://example.com/intersection/road_left/lane_right")] = self.map_dict.pop(URIRef("http://example.com/intersection/road_left/lane2"))
            #self.map_dict[URIRef("http://example.com/intersection/road_left/lane_left")] = self.map_dict.pop(URIRef("http://example.com/intersection/road_left/lane1"))
            

        self.phi_before = self.map_dict[self.start].get('orientation')
        self.phi_after = self.map_dict[self.goal].get('orientation')
        self.current_pos = self.start
        #side = self.map_dict[self.side_uri].get('polygon')
        #centerline = shift_line(side, -0.25*w)
        #self.extended_centerline = extend_line(centerline, self.phi_after, w)
        self.horizon_uri = []
        self.horizon_length = 1

        #self.g.remove((self.start, RDF.type, None))
        #self.g.remove((self.goal, RDF.type, None))
        self.g.add((self.start, RDF.type, EX.lane_right))
        self.g.add((self.goal, RDF.type, EX.lane_right))
        start_type = query_type(self.g, self.start)
        goal_type = query_type(self.g, self.goal)
        # print(f"start: {self.start}, start type: {start_type}")
        # print(f"goal: {self.goal}, goal type: {goal_type}")
        # input('wait')
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
        #input(lane_left_affordances)
        # if self.robot.uri == EX.AV1:
        #     input(lane_left_affordances)
        if start == 'down' and task == 'left':
            #self.map_dict[URIRef("http://example.com/intersection/road_down/lane_right")] = self.map_dict.pop(URIRef("http://example.com/intersection/road_down/lane1"))
            #self.map_dict[URIRef("http://example.com/intersection/road_down/lane_left")] = self.map_dict.pop(URIRef("http://example.com/intersection/road_down/lane2"))
            #self.map_dict[URIRef("http://example.com/intersection/road_left/lane_right")] = self.map_dict.pop(URIRef("http://example.com/intersection/road_down/lane2"))
            #self.map_dict[URIRef("http://example.com/intersection/road_left/lane_left")] = self.map_dict.pop(URIRef("http://example.com/intersection/road_down/lane1"))


            plan = {URIRef("http://example.com/intersection/road_down/lane1"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/road_down/lane2"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane2")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane1")},
            URIRef("http://example.com/intersection/road_left/lane2"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_left/lane1"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            ## miss later queryen?
            #self.g.remove((URIRef("http://example.com/intersection/road_down/lane_left"), RDF.type, None))
            #self.g.remove((URIRef("http://example.com/intersection/road_left/lane_left"), RDF.type, None))
            self.g.add((URIRef("http://example.com/intersection/road_down/lane2"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_left/lane1"), RDF.type, EX.lane_left))
            
            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_down/lane2"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_left/lane1"), EX.affordance, affordance))

        if start == 'down' and task == 'right':
            
            plan = {URIRef("http://example.com/intersection/road_down/lane_right"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/road_down/lane_left"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane_left")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane_right")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/road_right/lane_right"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_right/lane_left"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_down/lane_left"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_right/lane_right"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_down/lane_left"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_right/lane_right"), EX.affordance, affordance))

        if start == 'right' and task == 'left':
            plan = {URIRef("http://example.com/intersection/road_right/lane_right"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ur")},
            URIRef("http://example.com/intersection/road_right/lane_left"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dr")},
            URIRef("http://example.com/intersection/crossing_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane_left")},
            URIRef("http://example.com/intersection/crossing_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane_right")},
            URIRef("http://example.com/intersection/crossing_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_ul")},
            URIRef("http://example.com/intersection/crossing_dr"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/crossing_dl")},
            URIRef("http://example.com/intersection/road_left/lane_right"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_left/lane_left"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_right/lane_left"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_left/lane_right"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_right/lane_left"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_left/lane_right"), EX.affordance, affordance))

        return plan        
        
    def update(self, sim):
        #input(query_type(self.g, URIRef("http://example.com/intersection/road_right/lane_right")))
        if self.horizon_dict:
            #print(f"last uri in dict: {self.horizon_dict[str(len(self.horizon_dict)-1)]['uri']}")
            #print(f"last horizon pos: {self.horizon_dict[str(len(self.horizon_dict)-1)]['position']}")
            pass
        
        self.update_current_pos(sim)

        ## deze miss later andersom doen
        
            
        self.associate_positions()
        self.update_horizon(sim)

        #Associate the things that are in the robots horizon
        self.associate(sim)
        

    def update_current_pos(self, sim):
        prev_pos = self.current_pos
        #self.current_pos = self.current_area()
        self.intersect_dict = self.get_intersections(self.map_dict, self.robot.polygon)
        self.weight_dict = {}
        for uri, intersection in self.intersect_dict.items():
            if intersection > 0.4*self.robot.polygon.area:
                self.weight_dict[uri] = self.map_dict[uri]['weight']

        if self.weight_dict:
            self.current_pos = max(self.weight_dict, key=self.weight_dict.get)
        else:
            self.current_pos = None

        #print(f'Current position: {self.current_pos}')
            

        if self.current_pos == None:
            if self.robot.uri == EX.AV1:
                DeductiveClosure(Semantics).expand(self.g)
                self.g.serialize(format="json-ld", destination='AV1' + ".json")
                exit(0)
            elif self.robot.uri == EX.AV2:
                self.robot.reset('down', 'right')
                self.reset(self.robot)
            else:
                self.robot.random_reset()
                self.reset(self.robot)
            return

        if self.current_pos == prev_pos:
            self.same_situation = True
        else:
            self.same_situation = False          
            self.update_av_is_on()


    def update_horizon(self, sim):
        if self.replan:
            self.horizon_dict = {}
            self.horizon_dict = copy.deepcopy(self.horizon_before_turn)
            self.horizon_dict[str(len(self.horizon_dict))] = self.before_obstacle_ll
            #print(self.horizon_dict)
            #input('aipuhfuiwheohf')
            self.replan = False
            self.on_left_lane = True
        
        self.update_current_horizon(sim)

        #print(f'extend horizon: {self.extend_horizon}')
        if self.extend_horizon:
            self.update_extended_horizon(sim)

        self.delete_extended_horizon(sim)
        

        self.robot.horizon_dict = self.horizon_dict

        self.horizon_list = []
        for key, value in self.horizon_dict.items():
            self.horizon_list.append(value['polygon'])
        self.robot.horizon = unary_union(self.horizon_list)

    
    def associate_positions(self):
        ## ga weer n abstractie hoger?
        if self.after_obstacle_rl and self.current_pos==self.after_obstacle_rl['uri']:
            self.plan[URIRef("http://example.com/intersection/road_down/lane1")]['next_position'] = URIRef("http://example.com/intersection/crossing_dr")
            self.map_dict[self.after_obstacle_rl['uri']]['weight'] = -1
            self.set_current_wait_pos = False
            self.current_wait_pos = {}
            self.next_wait_pos = {}
            self.set_current_turn_pos = False
            self.set_next_wait_pos = False 
            self.set_next_turn_pos = False          
            #query_part_of()

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
            self.plan[URIRef("http://example.com/intersection/road_down/lane1")]['next_position'] = self.before_obstacle_rl['uri']
            
            plan_step = copy.deepcopy(self.plan[URIRef("http://example.com/intersection/road_down/lane1")])
            plan_step['next_position'] = self.before_obstacle_ll['uri']
            self.plan[self.before_obstacle_rl['uri']] = plan_step
            new_map = {}
            new_map['polygon'] = self.before_obstacle_rl['polygon']
            new_map['position'] = self.before_obstacle_rl['position']
            new_map['weight'] = 1
            self.map_dict[self.before_obstacle_rl['uri']] = new_map


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
        #print(f"current pos: {self.current_pos}, next pos: {self.plan[self.current_pos]['next_position']}")
        current_plan = []
        current_plan.append(self.map_dict[self.current_pos]['polygon'])
        pos = self.current_pos
        while self.plan[pos]['next_position']:
                #cn_pos = unary_union([self.map_dict[pos]['polygon'], self.map_dict[self.plan[pos]['next_position']]['polygon']])
            current_plan.append(self.map_dict[self.plan[pos]['next_position']]['polygon'])
            if len(current_plan)>1:
                break
            pos = self.plan[pos]['next_position']
                #cn_pos = self.map_dict[self.current_pos]['polygon']
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
        #horizon = self.extend_horizon[-1]
        #print(self.current_dict)
        #semantic_pos_dict = self.current_dict['semantic_position']
        max_weight = -5
        for ext_horizon in self.extend_horizon:
            weight = self.map_dict[ext_horizon]['weight']
            #intersection = semantic_pos_dict.get(ext_horizon, 0)
            if weight >= max_weight:
                max_weight = weight
                horizon = ext_horizon

        horizon_type = query_type(self.g, horizon)
        #print(f'horizon: {horizon}, horizon type: {horizon_type}')
        #print(f'intersections horizon: {self.get_intersections(self.map_dict, horizon)}')
        #print(f'plan configured: {self.plan_configured}')
        #print(f'Add next horizon {self.add_next_horizon}')
        previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]
        if horizon_type == EX.obstacle:
            obstacle = sim.obstacles[horizon]
            semantic_pos = self.get_max_intersection(self.map_dict, obstacle.polygon)
            x = obstacle.pos[0]
            y = obstacle.pos[1]
            lane_width = 20
            phi = self.plan[semantic_pos]['phi']
            #print(semantic_pos)
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
                #x = self.obstacles[0].pos[0]
                #y = self.obstacles[0].pos[1]
                x = previous_horizon_dict['position'][0]
                y = previous_horizon_dict['position'][1]
                lane_width = 20
                phi = self.plan[semantic_pos]['phi']
                offset = 0.5*previous_horizon_dict['length']
                H = 10

                # if self.plan[semantic_pos]['next_position']:
                #     cn_pos = unary_union([self.map_dict[semantic_pos]['polygon'], self.map_dict[self.plan[semantic_pos]['next_position']]['polygon']])
                # else:
                #     cn_pos = self.map_dict[semantic_pos]['polygon']


                current_plan = []
                current_plan.append(self.map_dict[semantic_pos]['polygon'])
                pos = semantic_pos
                #print(self.plan[pos]['next_position'])
                #input('hoi')
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
                    #self.add_next_horizon = False
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
                #horizon_polygon = self.map_dict[self.plan[semantic_pos]['next_position']]['polygon']
            self.add_horizon(semantic_pos, horizon_polygon, horizon_pos, H, 'next_horizon')

            
       #elif horizon_type == EX.crossing and self.plan_configured and not self.add_next_horizon:
        elif horizon_type == EX.crossing and (self.plan_configured or self.obstacle_on_crossing):
            #max_intersection = max(previous_horizon_dict['semantic_position'], key=previous_horizon_dict['semantic_position'].get)
            #print(previous_horizon_dict)
            crossing_intersect = previous_horizon_dict['semantic_position'].get(URIRef("http://example.com/intersection/crossing_dr"), 0) + \
                previous_horizon_dict['semantic_position'].get(URIRef("http://example.com/intersection/crossing_ur"), 0) + \
                    previous_horizon_dict['semantic_position'].get(URIRef("http://example.com/intersection/crossing_dl"), 0) + \
                    previous_horizon_dict['semantic_position'].get(URIRef("http://example.com/intersection/crossing_ul"), 0)
            #print(crossing_intersect)
            #input('qwerr')
            if crossing_intersect < 95:

            # print(previous_horizon_dict['semantic_position'])
            # print(horizon)
            # print(crossing_intersect)
            # input('asd')
                if horizon == EX.after_obstacle_rl:
                    #uri = self.plan[horizon]['next_position']
                    horizon = URIRef("http://example.com/intersection/crossing_dr")
                    #print(horizon)

                horizon_polygon = self.map_dict[horizon]['polygon']
                horizon_pos = self.map_dict[horizon]['position']
                horizon_length = 10
                #self.add_horizon(horizon, horizon_polygon, horizon_pos, 10, 'crossing')
                self.add_next_horizon = True
            
        
            #elif horizon_type == EX.crossing and self.add_next_horizon:
            else:
                previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]
                next_horizon_type = query_type(self.g, self.plan[horizon]['next_position'])
                #print(self.plan[horizon]['next_position'])
                #input('asdf')
                #print(f"next horizon: {self.plan[horizon]['next_position']}, next horizon type: {next_horizon_type}")
                if next_horizon_type == EX.crossing:
                    horizon_polygon = self.map_dict[self.plan[horizon]['next_position']]['polygon']
                    horizon_pos = self.map_dict[self.plan[horizon]['next_position']]['position']
                    horizon_length = 10
                elif next_horizon_type == EX.lane_left or next_horizon_type == EX.lane_right:
                    #self.add_next_horizon = False
                    x, y = self.map_dict[horizon]['position']
                    lane_width = 20
                    phi = self.plan[horizon]['phi']
                    #input(phi)
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

                    pos_x = x + (0.5*previous_horizon_dict['length']+0.5*H)*math.cos(phi)
                    pos_y = y + (0.5*previous_horizon_dict['length']+0.5*H)*math.sin(phi)
                    horizon_pos = [pos_x, pos_y]
                    horizon_length = H

            # if previous_horizon_dict['type'] == 'before_obstacle_ll':
            #     new_horizon_polygon = horizon_polygon.difference(self.before_obstacle_ll['polygon'])
            #     horizon_polygon = new_horizon_polygon
            #     input('apgjn')
            self.add_horizon(horizon, horizon_polygon, horizon_pos, horizon_length, 'crossing')
                    #self.add_next_horizon = True
                    #self.crossing_approaching = True

                

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
                    #cn_pos = unary_union([self.map_dict[pos]['polygon'], self.map_dict[self.plan[pos]['next_position']]['polygon']])
                current_plan.append(self.map_dict[self.plan[pos]['next_position']]['polygon'])
                if len(current_plan)>2:
                    break
                pos = self.plan[pos]['next_position']
                    #cn_pos = self.map_dict[self.current_pos]['polygon']
            cn_pos = unary_union(current_plan)
            # print(self.plan[horizon]['next_position'])
            # input('hallo')
            # if self.plan[horizon]['next_position']:
            #     cn_pos = unary_union([self.map_dict[horizon]['polygon'], self.map_dict[self.plan[horizon]['next_position']]['polygon']])
            # else:
            #     cn_pos = self.map_dict[horizon]['polygon']

            horizon_polygon = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)
                
            if horizon == EX.before_obstacle_ll:
                self.map_dict[horizon]['weight'] = -2
            # else:
            #     horizon_type = 'left_lane'
            #     new_horizon_polygon = horizon_polygon.difference(self.before_obstacle_ll['polygon'])
            #     horizon_polygon = new_horizon_polygon

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
            
            

        #elif horizon == EX.after_obstacle_rl:
            # horizon_polygon = self.map_dict[horizon]['polygon']
            # horizon_pos = self.map_dict[horizon]['position']
            self.add_horizon(next_horizon, horizon_polygon, horizon_pos, 10, 'after_obstacle')
            self.map_dict[next_horizon]['weight'] = -1
            self.plan_configured = True
            self.after_obstacle_configured =  True
            self.plan[self.before_obstacle_rl['uri']]['phi'] += 0.5*math.pi
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
        self.g.remove((None, EX.obstructs, self.robot.uri))
        self.g.remove((self.robot.uri, EX.approaches, None))
        #self.robot.obstructed_area = None
        approaching_obstacles = self.get_intersections(sim.obstacles_dict, self.robot.horizon_dict[str(len(self.robot.horizon_dict)-1)]['polygon'])
        for uri, area in approaching_obstacles.items():
            if area>2:
                #print(f'approaching area: {uri}')

                new_map = {}
                new_map['polygon'] = sim.obstacles_dict[uri]['polygon']
                new_map['weight'] = 1
                self.map_dict[uri] = new_map
                
                self.g.add((self.robot.uri, EX.approaches, uri))
        #for obstacle in sim.obstacles.values():
        #    if self.robot.horizon.intersects(obstacle.polygon) and self.robot.horizon.intersection(obstacle.polygon).area > 2:
                #self.obstacles.append(obstacle)
                #intersection = self.robot.horizon.intersection(obstacle.polygon)
                #self.robot.obstructed_area = intersection
                #obstacle.semantic_pos = self.get_max_intersection(intersection)
                #print(obstacle_pos)
                self.g.add((uri, EX.obstructs, self.robot.uri))
                self.g.add((uri, RDF.type, EX.obstacle))
                self.plan_configured = False
                #print(f"uri: {uri}, type: {query_type(self.g, uri)}")
                #input('wait')
                #self.g.add((uri, RDF.type, EX.polygon))
                #self.g.add((uri, RDF.type, EX.geometry))
                #self.g.remove((obstacle_pos, EX.affordance, None))
                #self.g.add((self.robot.uri, EX.approaches, uri))
                # if not self.wait_pos:
                #     self.wait_pos = {}
                #     self.wait_pos['polygon'] = self.prev_pos
                #     self.wait_pos['color'] = 'black'
                    #self.robot.horizon_dict[str(len(self.robot.horizon_dict))] = self.wait_pos


    
    def associate_vehicles(self, sim):
        self.g.remove((None, EX.passes, self.robot.uri))
        self.robot.vehicle_area = None
        for robot in sim.robots.values():
            if self.robot.horizon.intersects(robot.polygon) and self.robot.uri != robot.uri:
                intersection = self.robot.horizon.intersection(robot.polygon)
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
        #self.robot.approaching_horizon = None
        #self.g.remove((self.robot.uri, EX.approaches, None))
        approaching_areas = self.get_intersections(self.map_dict, self.robot.horizon_dict[str(len(self.robot.horizon_dict)-1)]['polygon'])
        for uri, area in approaching_areas.items():
            if area>5:
                #print(f'approaching area: {uri}')
                #print(f'uri: {uri}')
                self.g.add((self.robot.uri, EX.approaches, uri))
                #self.approaching_dict[uri] = area
                # if not self.wait_pos and self.prev_pos:
                #     self.wait_pos = {}
                #     self.wait_pos['polygon'] = self.prev_pos
                #     self.wait_pos['color'] = 'black'
                    #self.robot.horizon_dict[str(len(self.robot.horizon_dict))] = self.wait_pos


    def associate_direction(self, vehicle):
        ## later generieker doen met regels die je declaratief aan een road of intersection kunt plaatsen

        road_vehicle = query_road(self.g, vehicle.uri)
        road_current = query_road(self.g, self.robot.uri)
        #print(f'road current: {road_current}, road vehicle: {road_vehicle}')
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



 