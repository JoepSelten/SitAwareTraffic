from queries import *
from simplegraph3 import EX
import shapely
import math
from global_variables import w
from basic_functions import shift_line, extend_line

class SkillModel():
    def __init__(self):
        pass

    def monitor_skills(self, world, control):
        if world.skill_finished and not world.same_situation:
            world.skill_finished = False
            self.select_skill(world)
            self.config_skill(world)
        self.execute_skill(world, control)

    def select_skill(self, world):
        world.robot_pos = query_is_on(world.kg, world.AV_uri)
        type_pos = query_type(world.kg, world.robot_pos)
        #print(type_pos)
        #if str(self.robot_pos) == "http://example.com/intersection/road_down/lane_right" or str(self.robot_pos) == "http://example.com/intersection/road_right/lane_left" or \
         #       str(self.robot_pos) == "http://example.com/intersection/road_up/lane_left" or str(self.robot_pos) == "http://example.com/intersection/road_left/lane_left":
        if str(type_pos) == "http://example.com/lane":
            world.skill = 'move_in_lane'
        elif str(type_pos) == "http://example.com/middle":
            world.skill = 'turn'
        else:
            world.skill = 'stop'
        
    
    def config_skill(self, world):
        world.skill_params = []
        if world.skill == 'move_in_lane':
            phi = world.map_dict[world.robot_pos].get('orientation')
            world.skill_params.append(phi)
            #print(f'{world.robot.name}: {phi}')
            #world.skill = world.skill
            
            # zou ik hier control direct uitvoeren. Denk t het wel, beetje raar denk ik om het eerst via world model te sturen
        
        if world.skill == 'turn':
            # desired orientation, and also when to turn
            # should I query the higher level plan or task here?
            if world.robot.task == 'down':
                goal_lane = URIRef("http://example.com/intersection/road_down/lane_left")
                goal_side = URIRef("http://example.com/intersection/road_down/side_left")
                
            if world.robot.task == 'right':
                goal_lane = URIRef("http://example.com/intersection/road_right/lane_left")
                goal_side = URIRef("http://example.com/intersection/road_right/side_left")

            if world.robot.task == 'up':
                goal_lane = URIRef("http://example.com/intersection/road_up/lane_left")
                goal_side = URIRef("http://example.com/intersection/road_up/side_left")

            if world.robot.task == 'left':
                goal_lane = URIRef("http://example.com/intersection/road_left/lane_left")
                goal_side = URIRef("http://example.com/intersection/road_left/side_left")

            phi = world.map_dict[goal_lane].get('orientation')

            side = world.map_dict[goal_side].get('poly')
            centerline = shift_line(side, -0.25*w)
            extended_centerline = extend_line(centerline, phi, w)

            world.skill_params.append(phi)
            world.skill_params.append(extended_centerline)
        print(f'{world.robot.name}: {world.skill}')

    def execute_skill(self, world, control):
        if world.skill == 'move_in_lane':
            control.move_in_lane(world, *world.skill_params)

        if world.skill == 'turn':
            control.turn(world, *world.skill_params)

        if world.skill == 'stop':
            control.stop(world)
