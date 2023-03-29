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
        self.plan_configured = False
        self.replan = False
        self.switch_lane_configured = False
        self.switch_phi_current = False
        self.switch_phi_next = False
        self.positions_configured = False

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
        #side = self.map_dict[self.side_uri].get('polygon')
        #centerline = shift_line(side, -0.25*w)
        #self.extended_centerline = extend_line(centerline, self.phi_after, w)
        self.horizon_uri = []
        self.horizon_length = 1

        self.g.add((self.start, RDF.type, EX.lane_right))
        self.g.add((self.goal, RDF.type, EX.lane_right))
        lane_right_affordances = query_affordances(self.g, EX.lane_right)
        for affordance in lane_right_affordances:
            self.g.add((self.start, EX.affordance, affordance))
            self.g.add((self.goal, EX.affordance, affordance))
        ## deze moeten miss ook type lane_right en lane_left
        self.g.add((URIRef("http://example.com/intersection/middle_dr"), RDF.type, EX.middle))
        self.g.add((URIRef("http://example.com/intersection/middle_ur"), RDF.type, EX.middle))
        self.g.add((URIRef("http://example.com/intersection/middle_ul"), RDF.type, EX.middle))
        self.g.add((URIRef("http://example.com/intersection/middle_dl"), RDF.type, EX.middle))
        
        self.init_plan()
        #DeductiveClosure(Semantics).expand(self.g)
        
        self.update_av_is_on()

    def init_geometric_map(self, map):
        if map.traffic_situation == "two-lane_intersection":
            self.map_dict = {
                #URIRef("http://example.com/intersection/middle"): {'polygon': map.polygon_list[0].union(map.polygon_list[1]).union(map.polygon_list[2]).union(map.polygon_list[3])},
                URIRef("http://example.com/intersection/middle_dr"): {'polygon': map.polygon_list[0], 'position': [55, 45]},
                URIRef("http://example.com/intersection/middle_ur"): {'polygon': map.polygon_list[1], 'position': [55, 55]},
                URIRef("http://example.com/intersection/middle_ul"): {'polygon': map.polygon_list[2], 'position': [45, 55]},
                URIRef("http://example.com/intersection/middle_dl"): {'polygon': map.polygon_list[3], 'position': [45, 45]},
                URIRef("http://example.com/intersection/road_down/lane_right"): {'polygon': map.polygon_list[4], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_down/lane_left"): {'polygon': map.polygon_list[5], 'orientation': -0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/lane_right"): {'polygon': map.polygon_list[6], 'orientation': math.pi},
                URIRef("http://example.com/intersection/road_right/lane_left"): {'polygon': map.polygon_list[7], 'orientation': 0},
                URIRef("http://example.com/intersection/road_up/lane_right"): {'polygon': map.polygon_list[8], 'orientation': -0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/lane_left"): {'polygon': map.polygon_list[9], 'orientation': 0.5*math.pi}, 
                URIRef("http://example.com/intersection/road_left/lane_right"): {'polygon': map.polygon_list[10], 'orientation': 0},
                URIRef("http://example.com/intersection/road_left/lane_left"): {'polygon': map.polygon_list[11], 'orientation': math.pi},
                URIRef("http://example.com/intersection/road_down/side_right"): {'polygon': map.polygon_list[12], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_down/side_left"): {'polygon': map.polygon_list[13], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/side_right"): {'polygon': map.polygon_list[14], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/side_left"): {'polygon': map.polygon_list[15], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/side_right"): {'polygon': map.polygon_list[16], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/side_left"): {'polygon': map.polygon_list[17], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_left/side_right"): {'polygon': map.polygon_list[18], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_left/side_left"): {'polygon': map.polygon_list[19], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_down/centerline"): {'polygon': map.polygon_list[20], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_right/centerline"): {'polygon': map.polygon_list[21], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_up/centerline"): {'polygon': map.polygon_list[22], 'orientation': 0.5*math.pi},
                URIRef("http://example.com/intersection/road_left/centerline"): {'polygon': map.polygon_list[23], 'orientation': 0.5*math.pi}
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
        plan = {}
        lane_left_affordances = query_affordances(self.g, EX.lane_left)
        if start == 'down' and task == 'left':
            plan = {URIRef("http://example.com/intersection/road_down/lane_right"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/middle_dr")},
            URIRef("http://example.com/intersection/road_down/lane_left"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/middle_dl")},
            URIRef("http://example.com/intersection/middle_dr"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/middle_ur")},
            URIRef("http://example.com/intersection/middle_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/middle_ul")},
            URIRef("http://example.com/intersection/middle_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane_left")},
            URIRef("http://example.com/intersection/middle_dl"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_left/lane_right")},
            URIRef("http://example.com/intersection/road_left/lane_right"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_left/lane_left"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            ## miss later queryen?
            self.g.add((URIRef("http://example.com/intersection/road_down/lane_left"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_left/lane_right"), RDF.type, EX.lane_left))
            
            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_down/lane_left"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_left/lane_right"), EX.affordance, affordance))

        if start == 'down' and task == 'right':
            plan = {URIRef("http://example.com/intersection/road_down/lane_right"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/middle_dr")},
            URIRef("http://example.com/intersection/road_down/lane_left"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/middle_dl")},
            URIRef("http://example.com/intersection/middle_dr"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane_left")},
            URIRef("http://example.com/intersection/middle_ur"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/road_right/lane_right")},
            URIRef("http://example.com/intersection/middle_ul"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/middle_ur")},
            URIRef("http://example.com/intersection/middle_dl"): {'phi': self.phi_before, 'velocity': self.robot.velocity_max, 'next_position': URIRef("http://example.com/intersection/middle_ul")},
            URIRef("http://example.com/intersection/road_right/lane_right"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None},
            URIRef("http://example.com/intersection/road_right/lane_left"): {'phi': self.phi_after, 'velocity': self.robot.velocity_max, 'next_position': None}
            }
            self.g.add((URIRef("http://example.com/intersection/road_down/lane_left"), RDF.type, EX.lane_left))
            self.g.add((URIRef("http://example.com/intersection/road_right/lane_right"), RDF.type, EX.lane_left))

            for affordance in lane_left_affordances:
                self.g.add((URIRef("http://example.com/intersection/road_down/lane_left"), EX.affordance, affordance))
                self.g.add((URIRef("http://example.com/intersection/road_right/lane_right"), EX.affordance, affordance))

        return plan        
        
    def update(self, sim):
        self.update_current_pos(sim)

        ## deze miss later andersom doen
        if self.horizon_dict:
            print(f"last uri in dict: {self.horizon_dict[str(len(self.horizon_dict)-1)]['uri']}")
        self.associate_positions()
        self.update_horizon(sim)

        #Associate the things that are in the robots horizon
        self.associate(sim)
        

    def update_current_pos(self, sim):
        prev_pos = self.current_pos
        self.current_pos = self.current_area()
        self.pos_list = self.get_intersections(self.map_dict, self.robot.polygon)        

        if self.current_pos == None:
            if self.robot.uri == EX.AV1:
                DeductiveClosure(Semantics).expand(self.g)
                self.g.serialize(format="json-ld", destination='AV1' + ".json")
                exit(0)
            else:
                self.robot.random_reset()
                self.reset(self.g, self.robot)
            return

        if self.current_pos == prev_pos:
            self.same_situation = True
        else:
            self.same_situation = False          
            self.update_av_is_on()


    def update_horizon(self, sim):
        # if not self.replan and self.on_left_lane:
        #     if self.horizon_dict[str(len(self.horizon_dict)-1)]['polygon'].intersects(self.after_obstacle_ll['polygon']):
        #         self.horizon_dict[str(len(self.horizon_dict))] = self.after_obstacle_ll
        #         self.horizon_dict[str(len(self.horizon_dict))] = self.next_wait_pos
        #         self.on_left_lane = False
        #         self.extend_horizon.pop()
        #         self.plan_configured = True


        if self.replan:
            self.horizon_dict = {}
            self.horizon_dict = copy.deepcopy(self.horizon_before_turn)
            self.horizon_dict[str(len(self.horizon_dict))] = self.before_obstacle_ll
            self.replan = False
            self.on_left_lane = True

            #self.plan[self.before_obstacle_rl['uri']]

            ## if statement voor extended horizon op true te zetten
            ## de current horizon moet alleen samenvoegen met de switch lane pos.
            ## deze pos is dus wel echt de next pos, en niet de hele left lane. 
            ## of ik maak een polygon entiteit in t plan, of n nieuwe label

        #self.update_positions(sim)
        
        self.update_current_horizon(sim)

        if self.extend_horizon:
            self.update_extended_horizon(sim)

        self.delete_extended_horizon(sim)
        
            
        self.robot.horizon = unary_union(self.horizon_list)
        self.robot.horizon_dict = self.horizon_dict

    
    def associate_positions(self):
        if self.horizon_dict:
            self.current_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]

        if self.set_current_wait_pos and not self.current_wait_pos:
            self.current_wait_pos['polygon'] = self.prev_dict['polygon']
            self.current_wait_pos['position'] = self.prev_dict['position']
            self.current_wait_pos['uri'] = self.prev_dict['uri']
            self.current_wait_pos['color'] = 'orange'
            self.current_wait_pos['type'] = 'wait_pos'
            self.position_dict[str(len(self.position_dict))] = self.current_wait_pos

                    
        if self.set_next_wait_pos and not self.next_wait_pos:
            self.next_wait_pos['polygon'] = self.horizon_dict[str(len(self.horizon_dict)-1)]['polygon']
            self.next_wait_pos['position'] = self.horizon_dict[str(len(self.horizon_dict)-1)]['position']
            self.next_wait_pos['uri'] = self.horizon_dict[str(len(self.horizon_dict)-1)]['uri']
            self.next_wait_pos['color'] = 'orange'
            self.next_wait_pos['type'] = 'next_wait_pos'
            self.position_dict[str(len(self.position_dict))] = self.next_wait_pos


        if self.set_current_turn_pos and not self.before_obstacle_rl:
            self.before_obstacle_rl['polygon'] = self.prev_dict['polygon']
            self.before_obstacle_rl['position'] = self.prev_dict['position']
            self.before_obstacle_rl['uri'] =  EX.before_obstacle_rl
            self.before_obstacle_rl['color'] = 'red'
            self.before_obstacle_rl['type'] = 'before_right'
            self.position_dict[str(len(self.position_dict))] = self.before_obstacle_rl
            self.horizon_before_turn = copy.deepcopy(self.horizon_dict)
            
           
            lane_type = query_type(self.g, self.prev_dict['uri'])
            lane_affordances = query_affordances(self.g, self.prev_dict['uri'])
            self.g.add((self.before_obstacle_rl['uri'], RDF.type, lane_type))
            for affordance in lane_affordances:
                self.g.add((self.before_obstacle_rl['uri'], EX.affordance, affordance))


            self.plan[self.prev_dict['uri']]['next_position'] = self.before_obstacle_rl['uri']
            
            plan_step = copy.deepcopy(self.plan[self.prev_dict['uri']])
            self.before_obstacle_ll['uri'] = EX.before_obstacle_ll
            plan_step['next_position'] = self.before_obstacle_ll['uri']
            self.plan[self.before_obstacle_rl['uri']] = plan_step
            new_map = {}
            new_map['polygon'] = self.before_obstacle_rl['polygon']
            new_map['position'] = self.before_obstacle_rl['position']
            self.map_dict[self.before_obstacle_rl['uri']] = new_map
            
            ## de lane uri is nu nog max intersection
            self.before_obstacle_ll['polygon'], self.before_obstacle_ll['position'], lane_uri = self.get_left_lane(self.before_obstacle_rl)
            self.before_obstacle_ll['color'] = 'grey'
            self.before_obstacle_ll['type'] = 'before_left'
            self.position_dict[str(len(self.position_dict))] = self.before_obstacle_ll
            
            lane_type = query_type(self.g, lane_uri)
            lane_affordances = query_affordances(self.g, lane_uri)
            self.g.add((self.before_obstacle_ll['uri'], RDF.type, lane_type))
            for affordance in lane_affordances:
                self.g.add((self.before_obstacle_ll['uri'], EX.affordance, affordance))

            plan_step = copy.deepcopy(self.plan[lane_uri])
            plan_step['next_position'] = lane_uri
            self.plan[self.before_obstacle_ll['uri']] = plan_step
            new_map = {}
            new_map['polygon'] = self.before_obstacle_ll['polygon']
            new_map['position'] = self.before_obstacle_ll['position']
            self.map_dict[self.before_obstacle_ll['uri']] = new_map

            self.g.add((self.before_obstacle_rl['uri'], EX.connects, self.before_obstacle_ll['uri']))
            #DeductiveClosure(Semantics).expand(self.g)
            
            

        if self.set_next_turn_pos and not self.after_obstacle_rl:
            self.after_obstacle_rl['polygon'] = self.current_dict['polygon']
            self.after_obstacle_rl['position'] = self.current_dict['position']
            self.after_obstacle_rl['uri'] = EX.after_obstacle_rl
            self.after_obstacle_rl['color'] = 'yellow'
            self.after_obstacle_rl['type'] = 'after_right'
            self.position_dict[str(len(self.position_dict))] = self.after_obstacle_rl

            lane_type = query_type(self.g, self.current_dict['uri'])
            lane_affordances = query_affordances(self.g, self.current_dict['uri'])
            self.g.add((self.after_obstacle_rl['uri'], RDF.type, lane_type))
            for affordance in lane_affordances:
                self.g.add((self.after_obstacle_rl['uri'], EX.affordance, affordance))
            
            plan_step = copy.deepcopy(self.plan[self.current_dict['uri']])
            print(f"next pos: {self.current_dict['uri']}")
            plan_step['next_position'] = self.current_dict['uri']
            self.plan[self.after_obstacle_rl['uri']] = plan_step
            new_map = {}
            new_map['polygon'] = self.after_obstacle_rl['polygon']
            new_map['position'] = self.after_obstacle_rl['position']
            self.map_dict[self.after_obstacle_rl['uri']] = new_map

            self.after_obstacle_ll['polygon'], self.after_obstacle_ll['position'], lane_uri = self.get_left_lane(self.after_obstacle_rl)
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

            plan_step = copy.deepcopy(self.plan[lane_uri])
            plan_step['next_position'] = EX.after_obstacle_rl
            self.plan[EX.after_obstacle_ll] = plan_step
            new_map = {}
            new_map['polygon'] = self.after_obstacle_ll['polygon']
            new_map['position'] = self.after_obstacle_ll['position']
            self.map_dict[EX.after_obstacle_ll] = new_map
            self.g.add((EX.after_obstacle_ll, EX.connects, EX.after_obstacle_rl))

            self.positions_configured = True
            #DeductiveClosure(Semantics).expand(self.g)
            print(self.plan)

        self.robot.position_dict = self.position_dict


        if self.horizon_dict:
            self.prev_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]

    def get_left_lane(self, next_driveable_pos):
        #print(f'next_driveable_pos: {next_driveable_pos}')
        max_intersect = self.get_max_intersection(self.map_dict, next_driveable_pos['polygon'])
        phi = self.plan[max_intersect]['phi']
        x = next_driveable_pos['position'][0]
        y = next_driveable_pos['position'][1]
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
        self.horizon_list = []

        phi = self.plan[self.current_pos]['phi']
        if self.plan[self.current_pos]['next_position']:
            cn_pos = unary_union([self.map_dict[self.current_pos]['polygon'], self.map_dict[self.plan[self.current_pos]['next_position']]['polygon']])
        else:
            cn_pos = self.map_dict[self.current_pos]['polygon']
        
        horizon_polygon = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
            (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
            (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
            (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)

        pos_x = x + (0.5*self.robot.length+0.5*H)*math.cos(phi)
        pos_y = y + (0.5*self.robot.length+0.5*H)*math.sin(phi)
        horizon_pos = [pos_x, pos_y]

        if horizon_polygon.geom_type=='Polygon':
            self.horizon_list.append(horizon_polygon)
            current_horizon_dict = {}
            current_horizon_dict['uri'] = self.current_pos
            current_horizon_dict['polygon'] = horizon_polygon
            current_horizon_dict['length'] = H
            current_horizon_dict['position'] = horizon_pos
            current_horizon_dict['type'] = 'current'
            current_horizon_dict['color'] = 'blue'
            self.horizon_dict['0'] = current_horizon_dict
        
        if horizon_polygon.geom_type=='GeometryCollection':
            for polygon in horizon_polygon:
                if polygon.geom_type=='Polygon':
                    self.horizon_list.append(horizon_polygon)
                    current_horizon_dict['uri'] = self.current_pos
                    current_horizon_dict['polygon'] = horizon_polygon
                    current_horizon_dict['length'] = H
                    current_horizon_dict['position'] = horizon_pos
                    current_horizon_dict['type'] = 'current'
                    current_horizon_dict['color'] = 'blue'
                    self.horizon_dict['0'] = current_horizon_dict


    def update_extended_horizon(self, sim):
        horizon = self.extend_horizon[-1]
        horizon_type = query_type(self.g, horizon)
        print(f'horizon: {horizon}, horizon type: {horizon_type}')
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
            #self.horizon_list.append(horizon_polygon)
            #print(horizon_polygon)

            if horizon_polygon.geom_type=='Polygon':
                self.horizon_list.append(horizon_polygon)
                current_horizon_dict = {}
                current_horizon_dict['uri'] = semantic_pos
                current_horizon_dict['polygon'] = horizon_polygon
                current_horizon_dict['position'] = [x, y]
                current_horizon_dict['length'] = obstacle.length       
                current_horizon_dict['color'] = 'green'
                current_horizon_dict['type'] = 'obstacle'
                self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
            
            if horizon_polygon.geom_type=='GeometryCollection':
                for polygon in horizon_polygon:
                    if polygon.geom_type=='Polygon':
                        self.horizon_list.append(horizon_polygon)
                        current_horizon_dict = {}
                        current_horizon_dict['uri'] = semantic_pos
                        current_horizon_dict['polygon'] = horizon_polygon
                        current_horizon_dict['position'] = [x, y]
                        current_horizon_dict['length'] = obstacle.length
                        current_horizon_dict['color'] = 'green'
                        current_horizon_dict['type'] = 'obstacle'
                        self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict

            # if current_horizon_dict['polygon'].intersects(self.horizon_dict[str(len(self.horizon_dict)-2)]['polygon']):
            #     horizon_difference = self.horizon_dict[str(len(self.horizon_dict)-2)]['polygon'].difference(current_horizon_dict['polygon'])
            #     self.horizon_dict[str(len(self.horizon_dict)-2)]['polygon'] = horizon_difference
        
        #elif horizon.type == EX.lane:
            previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]
            #x = self.obstacles[0].pos[0]
            #y = self.obstacles[0].pos[1]
            x = previous_horizon_dict['position'][0]
            y = previous_horizon_dict['position'][1]
            lane_width = 20
            phi = self.plan[semantic_pos]['phi']
            offset = 0.5*previous_horizon_dict['length']
            H = 10

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


            if horizon_polygon.geom_type=='Polygon':
                self.horizon_list.append(horizon_polygon)
                current_horizon_dict = {}
                current_horizon_dict['uri'] = semantic_pos
                current_horizon_dict['polygon'] = horizon_polygon
                current_horizon_dict['position'] = horizon_pos
                current_horizon_dict['length'] = H
                current_horizon_dict['color'] = 'green'
                current_horizon_dict['type'] = 'next_horizon'
                self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
            
            if horizon_polygon.geom_type=='GeometryCollection':
                for polygon in horizon_polygon:
                    if polygon.geom_type=='Polygon':
                        self.horizon_list.append(horizon_polygon)
                        current_horizon_dict = {}
                        current_horizon_dict['uri'] = semantic_pos
                        current_horizon_dict['polygon'] = horizon_polygon
                        current_horizon_dict['position'] = horizon_pos
                        current_horizon_dict['length'] = H
                        current_horizon_dict['color'] = 'green'
                        current_horizon_dict['type'] = 'next_horizon'
                        self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict

        elif horizon_type == EX.middle:
            horizon_polygon = self.map_dict[horizon]['polygon']
            if horizon_polygon.geom_type=='Polygon':
                self.horizon_list.append(horizon_polygon)
                current_horizon_dict = {}
                current_horizon_dict['uri'] = horizon
                current_horizon_dict['polygon'] = horizon_polygon
                current_horizon_dict['color'] = 'green'
                current_horizon_dict['position'] = self.map_dict[horizon]['position']
                current_horizon_dict['length'] = 10
                current_horizon_dict['type'] = 'middle'
                self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
            
            if horizon_polygon.geom_type=='GeometryCollection':
                for polygon in horizon_polygon:
                    if polygon.geom_type=='Polygon':
                        self.horizon_list.append(horizon_polygon)
                        current_horizon_dict = {}
                        current_horizon_dict['uri'] = horizon
                        current_horizon_dict['polygon'] = horizon_polygon
                        current_horizon_dict['position'] = self.map_dict[horizon]['position']
                        current_horizon_dict['length'] = 10
                        current_horizon_dict['type'] = 'middle'
                        current_horizon_dict['color'] = 'green'
                        self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict

            # if current_horizon_dict['polygon'].intersects(self.horizon_dict[str(len(self.horizon_dict)-2)]['polygon']):
            #     horizon_difference = self.horizon_dict[str(len(self.horizon_dict)-2)]['polygon'].difference(current_horizon_dict['polygon'])
            #     self.horizon_dict[str(len(self.horizon_dict)-2)]['polygon'] = horizon_difference
            
            previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]

            next_horizon_type = query_type(self.g, self.plan[horizon]['next_position'])
            if next_horizon_type == EX.middle:
                horizon_polygon = self.map_dict[self.plan[horizon]['next_position']]['polygon']
                horizon_pos = self.map_dict[self.plan[horizon]['next_position']]['position']
                horizon_length = 10
            elif next_horizon_type == EX.lane:
                x, y = self.map_dict[horizon]['position']
                lane_width = 20
                phi = self.plan[horizon]['phi']
                offset = 5
                H = 10
                

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

            if horizon_polygon.geom_type=='Polygon':
                self.horizon_list.append(horizon_polygon)
                current_horizon_dict = {}
                current_horizon_dict['uri'] = horizon
                current_horizon_dict['polygon'] = horizon_polygon
                current_horizon_dict['position'] = horizon_pos
                current_horizon_dict['length'] = horizon_length
                current_horizon_dict['color'] = 'green'
                current_horizon_dict['type'] = 'middle'
                self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
            
            if horizon_polygon.geom_type=='GeometryCollection':
                for polygon in horizon_polygon:
                    if polygon.geom_type=='Polygon':
                        self.horizon_list.append(horizon_polygon)
                        current_horizon_dict = {}
                        current_horizon_dict['uri'] = horizon
                        current_horizon_dict['polygon'] = horizon_polygon
                        current_horizon_dict['position'] = horizon_pos
                        current_horizon_dict['length'] = horizon_length
                        current_horizon_dict['color'] = 'green'
                        current_horizon_dict['type'] = 'middle'
                        self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict


        elif horizon_type==EX.lane_left and not horizon == EX.after_obstacle_ll:
            previous_horizon_dict = self.horizon_dict[str(len(self.horizon_dict)-1)]
            x, y = previous_horizon_dict['position']
            lane_width = 20
            phi = self.plan[horizon]['phi']
            offset = 5
            H = 10
            
            if self.plan[horizon]['next_position']:
                cn_pos = unary_union([self.map_dict[horizon]['polygon'], self.map_dict[self.plan[horizon]['next_position']]['polygon']])
            else:
                cn_pos = self.map_dict[horizon]['polygon']

            horizon_polygon = Polygon([(x+offset*math.cos(phi)-lane_width*math.sin(phi), y+offset*math.sin(phi)-lane_width*math.cos(phi)),
                (x+(offset+H)*math.cos(phi)-lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)-lane_width*math.cos(phi)),
                (x+(offset+H)*math.cos(phi)+lane_width*math.sin(phi), y+(offset+H)*math.sin(phi)+lane_width*math.cos(phi)),
                (x+offset*math.cos(phi)+lane_width*math.sin(phi), y+offset*math.sin(phi)+lane_width*math.cos(phi))]).intersection(cn_pos)

            pos_x = x + (0.5*H+0.5*H)*math.cos(phi)
            pos_y = y + (0.5*H+0.5*H)*math.sin(phi)
            horizon_pos = [pos_x, pos_y]
            horizon_length = H

            if horizon_polygon.geom_type=='Polygon':
                self.horizon_list.append(horizon_polygon)
                current_horizon_dict = {}
                current_horizon_dict['uri'] = horizon
                current_horizon_dict['polygon'] = horizon_polygon
                current_horizon_dict['position'] = horizon_pos
                current_horizon_dict['length'] = horizon_length
                current_horizon_dict['color'] = 'green'
                current_horizon_dict['type'] = 'left_lane'
                self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
            
            if horizon_polygon.geom_type=='GeometryCollection':
                for polygon in horizon_polygon:
                    if polygon.geom_type=='Polygon':
                        self.horizon_list.append(horizon_polygon)
                        current_horizon_dict = {}
                        current_horizon_dict['uri'] = horizon
                        current_horizon_dict['polygon'] = horizon_polygon
                        current_horizon_dict['position'] = horizon_pos
                        current_horizon_dict['length'] = horizon_length
                        current_horizon_dict['color'] = 'green'
                        current_horizon_dict['type'] = 'left_lane'
                        self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict

        elif horizon == EX.after_obstacle_ll:
            horizon_polygon = self.map_dict[horizon]['polygon']
            if horizon_polygon.geom_type=='Polygon':
                self.horizon_list.append(horizon_polygon)
                current_horizon_dict = {}
                current_horizon_dict['uri'] = horizon
                current_horizon_dict['polygon'] = horizon_polygon
                current_horizon_dict['color'] = 'green'
                current_horizon_dict['position'] = self.map_dict[horizon]['position']
                current_horizon_dict['length'] = 10
                current_horizon_dict['type'] = 'after_obstacle'
                self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
            
            if horizon_polygon.geom_type=='GeometryCollection':
                for polygon in horizon_polygon:
                    if polygon.geom_type=='Polygon':
                        self.horizon_list.append(horizon_polygon)
                        current_horizon_dict = {}
                        current_horizon_dict['uri'] = horizon
                        current_horizon_dict['polygon'] = horizon_polygon
                        current_horizon_dict['position'] = self.map_dict[horizon]['position']
                        current_horizon_dict['length'] = 10
                        current_horizon_dict['type'] = 'after_obstacle'
                        current_horizon_dict['color'] = 'green'
                        self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict

            horizon_polygon = self.map_dict[self.plan[horizon]['next_position']]['polygon']

            if horizon_polygon.geom_type=='Polygon':
                self.horizon_list.append(horizon_polygon)
                current_horizon_dict = {}
                current_horizon_dict['uri'] = horizon
                current_horizon_dict['polygon'] = horizon_polygon
                current_horizon_dict['color'] = 'green'
                current_horizon_dict['position'] = self.map_dict[horizon]['position']
                current_horizon_dict['length'] = 10
                current_horizon_dict['type'] = 'after_obstacle'
                self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
            
            if horizon_polygon.geom_type=='GeometryCollection':
                for polygon in horizon_polygon:
                    if polygon.geom_type=='Polygon':
                        self.horizon_list.append(horizon_polygon)
                        current_horizon_dict = {}
                        current_horizon_dict['uri'] = horizon
                        current_horizon_dict['polygon'] = horizon_polygon
                        current_horizon_dict['position'] = self.map_dict[horizon]['position']
                        current_horizon_dict['length'] = 10
                        current_horizon_dict['type'] = 'after_obstacle'
                        current_horizon_dict['color'] = 'green'
                        self.horizon_dict[str(len(self.horizon_dict))] = current_horizon_dict
            

        # if current_horizon_dict['polygon'].intersects(self.horizon_dict[str(len(self.horizon_dict)-2)]['polygon']):
        #     horizon_difference = self.horizon_dict[str(len(self.horizon_dict)-2)]['polygon'].difference(current_horizon_dict['polygon'])
        #     self.horizon_dict[str(len(self.horizon_dict)-2)]['polygon'] = horizon_difference

        self.extend_horizon.pop()


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
                print(f'uri: {uri}')
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
            #print(f'current_areas, {self.robot.name}: {self.current_areas}')
            return max(self.current_areas, key=self.current_areas.get)


    def get_intersections(self, dict, area):
        intersection_dict = {}
        for key, value in dict.items():
            if area.intersects(value['polygon']):
                intersect_area = area.intersection(value['polygon']).area
                intersection_dict[key] = intersect_area
                #print(f'current_areas, {self.robot.name}: {self.current_areas}')
        return intersection_dict


    def get_max_intersection(self, dict, area):
        intersections = self.get_intersections(dict, area) 
        return max(intersections, key=intersections.get)



 