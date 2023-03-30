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
        self.init_replan = True

    def check_conditions(self, world):
        ## Get current skill (here I assume skill is known and position needs to be queried from KG)
        #skill = world.plan[str(world.plan_step)]['skill']

        ## Check conditions:
        # todo: check the conditions of the skill, for I know I just preprogram it 
        
 
        ## First condition is being on an area that is drivable
        area_uri = query_is_on(world.g, world.robot.uri)
        affordances_current = query_affordances(world.g, area_uri)
        
        ## Second condition is that also the next/approaching area is drivable
        approaches_list = query_approaches(world.g, world.robot.uri)

        if approaches_list:
        #if world.horizon_dict:
            #last_area = world.horizon_dict[str(len(world.horizon_dict)-1)]['uri']

            waiting_number = 0
            drivable_number = 0
            #print(approaches_list)
            for area in approaches_list:
                next_affordances = query_affordances(world.g, area)

                print(f'area: {area}, next_affordances: {next_affordances}')

                if EX.waiting in next_affordances:
                    waiting_number+=1             

                elif not EX.waiting in next_affordances:
                    world.set_current_wait_pos = True

                if world.set_current_wait_pos and waiting_number==len(approaches_list):
                    world.set_next_wait_pos = True


                if EX.drivable in next_affordances:
                    drivable_number+=1

                elif not EX.drivable in next_affordances:
                    world.set_current_turn_pos = True
                    world.plan_configured = False
                    
                
                if world.set_current_turn_pos and drivable_number==len(approaches_list):
                    print('next drivable pos found')
                    world.set_next_turn_pos = True
                    

                if not EX.waiting in next_affordances or not EX.drivable in next_affordances:
                    world.extend_horizon.append(area)

                    #print(f'extend horizon: {world.extend_horizon}')

        ## later integrate the above with vehicles and obstacles that inhibit driveability.
        ## however for convenience first do this separetely to get things working
        
        # passing_robot = query_passes(world.g, world.robot.uri)
        # conflict_robot = query_conflict(world.g, world.robot.uri)
        # obstruct_robot = query_obstructs(world.g, world.robot.uri)

        if world.set_current_wait_pos and not world.set_next_wait_pos:
            world.skill = 'wait'

        elif world.set_current_turn_pos and not world.set_next_turn_pos:
            world.skill = 'wait'

        elif world.set_current_turn_pos and world.set_next_turn_pos:
            world.skill = 'replan'
            #world.skill = 'drive'

        # elif conflict_robot:
        #     #print(f'{world.robot.name}: conflict')
        #     world.skill = 'check_priority'


        else:
            world.skill = 'drive'
            world.wait_pos = None

        #print(f'skill before config: {world.skill}')

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
            
            # if not world.wait_pos:
            #     world.wait_pos = world.horizon_list[0]
            if world.current_wait_pos:
                world.wait_pos = world.current_wait_pos['polygon']
            else:
                world.skill = 'drive'
        elif world.skill == 'check_priority':
            priority_vehicles = query_right_of(world.g, world.robot.uri)
            if priority_vehicles:
                world.skill = 'wait'
                if not world.wait_pos:
                    world.wait_pos = world.horizon_list[0]
            else:
                world.skill = 'drive'
        elif world.skill == 'replan':
            #print(f'is the plan configured: {world.plan_configured}')
            if world.plan_configured:
                if not world.switch_phi:
                    world.plan[world.before_obstacle_rl['uri']]['phi'] += 0.5*math.pi
                    world.plan[world.after_obstacle_ll['uri']]['phi'] -= 0.5*math.pi
                    world.switch_phi = True
                world.skill = 'drive'
            elif not world.wait_pos:
                world.skill = 'wait'
                world.wait_pos = world.before_obstacle_rl['polygon']
            
            else:
                world.skill = 'wait'
                


    def execute_skill(self, world, control):
        print(f'Skill: {world.skill}')
        if world.skill == 'drive':
            control.drive(world)
        elif world.skill == 'wait':
            #print(world.robot.polygon.intersects(world.wait_pos))
            if world.robot.polygon.intersects(world.wait_pos) and world.robot.polygon.intersection(world.wait_pos).area > 0.95 * world.robot.polygon.area:
                control.stop(world)
            else:
                control.drive(world)
        # elif world.skill == 'switch_lane':
        #     if not world.switch_phi:
        #         #world.plan[world.current_pos]['phi'] += 0.5*math.pi
        #         world.switch_phi = True
        #     if world.robot.polygon.intersects(world.before_obstacle_rl['polygon']) and world.robot.polygon.intersection(world.before_obstacle_rl['polygon']).area > 0.5 * world.robot.polygon.area:
        #         control.drive(world)
