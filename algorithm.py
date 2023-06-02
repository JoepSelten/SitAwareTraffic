from queries import *
from simplegraph3 import EX
import shapely
import math
import matplotlib.pyplot as plt
from global_variables import w
from basic_functions import shift_line, extend_line


class Algorithm_TrafficRules():
    def __init__(self):
        pass

    def run(self, world, control):
        self.check_pos(world)

        self.check_approaches(world)

        self.check_conflict(world)

        self.config(world)

        self.execute(world, control)


    def check_pos(self, world):
        ## check if current location is drivable
        area_uri = query_is_on(world.g, world.robot.uri)
        self.affordances_current = query_affordances(world.g, area_uri)
        
    def check_approaches(self, world):
        approaches_list = query_approaches(world.g, world.robot.uri)
        approaching_AV = False

        if approaches_list:
            waiting_number = 0
            drivable_number = 0
            for area in approaches_list:

                type_AV = query_type_AV(world.g, area)
                if type_AV:
                    approaching_AV = True

                next_affordances = query_affordances(world.g, area)

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
                    world.set_next_turn_pos = True
                    

                if not type_AV and (not EX.waiting in next_affordances or not EX.drivable in next_affordances):
                    world.extend_horizon.append(area)

        if not world.plan_configured:
            world.behaviour = 'replan'

        elif approaching_AV:
            world.behaviour = 'wait'

        elif not world.set_next_wait_pos:
            pass

        else:
            world.behaviour = 'drive'
            world.wait_pos = None

    def check_conflict(self, world):
        conflict_robot = query_conflict(world.g, world.robot.uri)
        if conflict_robot:
            if EX.waiting in self.affordances_current:
                world.behaviour = 'wait'
            right_of = query_right_of(world.g, world.robot.uri)
            if right_of:
                world.behaviour = 'wait'

    def config(self, world):
        if world.behaviour == 'drive':
            world.wait_pos = None
            world.turn_area = None
        elif world.behaviour == 'wait':
            if world.current_wait_pos:
                world.wait_pos = world.current_wait_pos['polygon']
            else:
                world.behaviour = 'drive'

        elif world.behaviour == 'replan':
            if world.after_obstacle_configured and not world.switch_phi:
                world.plan[world.before_obstacle_rl['uri']]['phi'] += 0.5*math.pi
                world.plan[world.after_obstacle_ll['uri']]['phi'] -= 0.5*math.pi
                world.switch_phi = True
            if world.current_wait_pos:
                world.wait_pos = world.current_wait_pos['polygon']
                world.behaviour = 'wait'
            else:
                world.behaviour = 'drive'


    def execute(self, world, control):
        if world.behaviour == 'drive':
            control.drive(world)
        elif world.behaviour == 'wait':
            if world.robot.polygon.intersects(world.wait_pos) and world.robot.polygon.intersection(world.wait_pos).area > 0.75 * world.robot.polygon.area:
                control.stop(world)
            else:
                control.drive(world)