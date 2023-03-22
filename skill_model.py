from queries import *
from simplegraph3 import EX
import shapely
import math
import matplotlib.pyplot as plt
from global_variables import w
from basic_functions import shift_line, extend_line
from skills import MoveInLane, Turn, SlowDownAndStop
from traffic_rules import ApproachingMiddle, Priority
import copy

class SkillModel():
    def __init__(self):
        self.skill_dict = {'move_in_lane': MoveInLane(),
                            'turn': Turn(),
                            'slow_down_and_stop:': SlowDownAndStop()
        }
        self.rule_dict = {'approaching_middle': ApproachingMiddle(),
                            'priority': Priority()
        }
        self.condition_failed = False
        self.traffic_rules = None

    def check_conditions(self, world):
        ## Get current skill (here I assume skill is known and position needs to be queried from KG)
        #skill = world.plan[str(world.plan_step)]['skill']

        ## Check conditions:
        # todo: check the conditions of the skill, for I know I just preprogram it 
        
 
        ## First condition is being on an area that is driveable
        area_uri = query_is_on(world.g, world.robot.uri)
        affordances_current = query_affordances(world.g, area_uri)
        
        ## Second condition is that also the next/approaching area is driveable
        approaches_list = query_approaches(world.g, world.robot.uri)
        #print(f'approaches_list: {approaches_list}')
        if approaches_list:
        #if world.horizon_dict:
            #last_area = world.horizon_dict[str(len(world.horizon_dict)-1)]['uri']
            last_area = approaches_list[-1]
            #print(last_area)
            next_affordances = query_affordances(world.g, last_area)
            #print(next_affordances)
            
            #if not next_affordances:
                #self.condition_failed = True
                #print('NO SKILL POSSIBLE')
                #world.skill = 'switch'

            ## Due to the rule that there always should be a skill available, when you cannot wait on the next position,
            ## requires also the next position to be driveable, which in turn requires looking further by increasing the horizon
            #print(f'approaches: {approaches_list}')
            #print(f'affordances: {next_affordances}')
            if not EX.waiting in next_affordances:
                #world.extend = True
                if not world.extend_horizon or world.extend_horizon[-1] != last_area:
                    #world.horizon_uris.append(approaches_list[-1])
                    world.extend_horizon.append(last_area)
                    
                    #print(world.extend_horizon)
                #world.horizon_length += 1
                #print(f'uris: {world.horizon_uris}')
                return
            else:
                world.found_waiting_area = True
                

        ## later integrate the above with vehicles and obstacles that inhibit driveability.
        ## however for convenience first do this separetely to get things working

        passing_robot = query_passes(world.g, world.robot.uri)
        conflict_robot = query_conflict(world.g, world.robot.uri)
        obstruct_robot = query_obstructs(world.g, world.robot.uri)
        #print(obstruct_robot)

        if passing_robot:
            ## stop in first area of horizon list.
            #if world.robot.name == 'AV1':
            #print(f'{world.robot.name}: passing')
            world.skill = 'wait'

        elif conflict_robot:
            #print(f'{world.robot.name}: conflict')
            world.skill = 'check_priority'

        elif obstruct_robot:
            world.skill = 'switch_lane'

        else:
            world.skill = 'drive'
            world.wait_pos = None

    def check_condition(self, world, condition):
        if not condition.for_all:
            check = query_check(world.g, condition.subject, condition.relation, condition.object)
        elif condition.for_all:
            check = query_check_for_all(world.g, condition.subject, condition.relation, condition.object)
       
        if condition.effect is not condition.negation:
            return not check
        return check

    def monitor_skills(self, world, control):
        self.check_conditions(world)
        self.config_skill(world)
        self.execute_skill(world, control)

    def check_traffic_rules(self, world, traffic_rules):
        #input("Press Enter to continue...")
        ## check what to do when robot approaching middle
        #print(traffic_rules)
        rule_obj = self.rule_dict[traffic_rules]
        rule_obj.config_rules(world)
        for condition in rule_obj.condition_list:
            check = self.check_condition(world, condition)
            
            if check == False:
                world.failed_condition = condition
                #print(f'{condition.subject}, {condition.relation}, {condition.object}: {check}')
                #input("Press Enter to continue...")
                world.condition_failed = True
                return
        world.condition_failed = False
        world.check_rules = False
        

    def config_skill(self, world):
        if world.skill == 'drive':
            world.wait_pos = None
            world.turn_area = None
        elif world.skill == 'wait':
            if not world.wait_pos:
                world.wait_pos = world.horizon_list[0]
        elif world.skill == 'check_priority':
            priority_vehicles = query_right_of(world.g, world.robot.uri)
            if priority_vehicles:
                world.skill = 'wait'
                if not world.wait_pos:
                    world.wait_pos = world.horizon_list[0]
            else:
                world.skill = 'drive'
        elif world.skill == 'switch_lane':
            if not world.turn_area:
                world.turn_area = world.horizon_list[-1]
                world.turn_pos = copy.copy(world.robot.pos)
                #world.turn_pos_left_lane = 
                world.plan[world.current_pos]['next_pos'] = URIRef("http://example.com/intersection/road_down/lane_left")
                
            if world.current_pos == URIRef("http://example.com/intersection/road_down/lane_left"):
                obstacles = query_obstructs(world.g, world.robot.uri)
                for obstacle in obstacles:
                    world.g.remove((obstacle, EX.obstructs, world.robot.uri))
           
            #world.plan, world.plan_left_lane = world.plan_left_lane, world.plan
            #world.right_lane, world.left_lane = world.left_lane, world.right_lane

    def execute_skill(self, world, control):
        #print(world.skill)
        if world.skill == 'drive':
            control.drive(world)
        elif world.skill == 'wait':
            if world.robot.poly.intersects(world.wait_pos) and world.robot.poly.intersection(world.wait_pos).area > 0.95 * world.robot.poly.area:
                control.stop(world)
            else:
                control.drive(world)
        elif world.skill == 'switch_lane':
            if not world.switch_phi:
                world.plan[world.current_pos]['phi'] += 0.5*math.pi
                world.switch_phi = True
            if world.robot.poly.intersects(world.turn_area) and world.robot.poly.intersection(world.turn_area).area > 0.5 * world.robot.poly.area:
                control.drive(world)
