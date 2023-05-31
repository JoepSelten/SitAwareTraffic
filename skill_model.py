from queries import *
from simplegraph3 import EX
import shapely
import math
import matplotlib.pyplot as plt
from global_variables import w
from basic_functions import shift_line, extend_line


class Algorithm_TrafficRules():
    def __init__(self):
        self.condition_failed = False
        self.traffic_rules = None
        self.init_replan = True

    def run(self, world, control):
        self.check_pos(world)

        self.check_approaches(world)

        #self.check_conflict(world)

        self.config_skill(world)

        self.execute_skill(world, control)


    def check_pos(self, world):
        ## check if current location is drivable
        area_uri = query_is_on(world.g, world.robot.uri)
        self.affordances_current = query_affordances(world.g, area_uri)
        
    def check_approaches(self, world):
        approaches_list = query_approaches(world.g, world.robot.uri)

        if approaches_list:
            waiting_number = 0
            drivable_number = 0
            #print(approaches_list)
            for area in approaches_list:
                next_affordances = query_affordances(world.g, area)

                #print(f'area: {area}, next_affordances: {next_affordances}')

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
                    #print('next drivable pos found')
                    world.set_next_turn_pos = True
                    

                if not EX.waiting in next_affordances or not EX.drivable in next_affordances:
                    #if world.plan_configured
                    world.extend_horizon.append(area)

        
        passing_robot = query_passes(world.g, world.robot.uri)
        conflict_robot = query_conflict(world.g, world.robot.uri)
        obstruct_robot = query_obstructs(world.g, world.robot.uri)

        if obstruct_robot or not world.plan_configured:
            world.skill = 'replan'

        elif passing_robot:
            world.skill = 'wait'

        elif conflict_robot:
            if EX.waiting in self.affordances_current:
                world.skill = 'wait'
            right_of = query_right_of(world.g, world.robot.uri)
            if right_of:
                world.skill = 'wait'

        elif not world.set_next_wait_pos:
            pass

        else:
            world.skill = 'drive'
            world.wait_pos = None

        

    def config_skill(self, world):
        if world.skill == 'drive':
            world.wait_pos = None
            world.turn_area = None
        elif world.skill == 'wait':
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
            if world.after_obstacle_configured and not world.switch_phi:
                world.plan[world.before_obstacle_rl['uri']]['phi'] += 0.5*math.pi
                world.plan[world.after_obstacle_ll['uri']]['phi'] -= 0.5*math.pi
                world.switch_phi = True
            if world.current_wait_pos:
                world.wait_pos = world.current_wait_pos['polygon']
                world.skill = 'wait'
            else:
                world.skill = 'drive'


    def execute_skill(self, world, control):
        if world.skill == 'drive':
            control.drive(world)
        elif world.skill == 'wait':
            if world.robot.polygon.intersects(world.wait_pos) and world.robot.polygon.intersection(world.wait_pos).area > 0.75 * world.robot.polygon.area:
                control.stop(world)
            else:
                control.drive(world)