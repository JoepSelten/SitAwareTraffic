from queries import *
from simplegraph3 import EX
import shapely
import math
from global_variables import w
from basic_functions import shift_line

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
        self.robot_pos = query_is_on(world.kg, world.AV_uri)
        type_pos = query_type(world.kg, self.robot_pos)
        #print(type_pos)
        #if str(self.robot_pos) == "http://example.com/intersection/road_down/lane_right" or str(self.robot_pos) == "http://example.com/intersection/road_right/lane_left" or \
         #       str(self.robot_pos) == "http://example.com/intersection/road_up/lane_left" or str(self.robot_pos) == "http://example.com/intersection/road_left/lane_left":
        if str(type_pos) == "http://example.com/lane":
            self.skill = 'move_in_lane'
        elif str(type_pos) == "http://example.com/middle":
            self.skill = 'turn'
        else:
            self.skill = 'stop'
        
    
    def config_skill(self, world):
        self.params = []
        if self.skill == 'move_in_lane':
            phi = world.map_dict[self.robot_pos].get('orientation')
            self.params.append(phi)
            #world.skill = self.skill
            
            # zou ik hier control direct uitvoeren. Denk t het wel, beetje raar denk ik om het eerst via world model te sturen
        
        if self.skill == 'turn':
            # desired orientation, and also when to turn
            # should I query the higher level plan or task here?
            if world.robot.task == 'down':
                goal_lane = URIRef("http://example.com/intersection/road_down/lane_left")
                
            if world.robot.task == 'right':
                goal_lane = URIRef("http://example.com/intersection/road_right/lane_left")

            if world.robot.task == 'up':
                goal_lane = URIRef("http://example.com/intersection/road_up/lane_left")

            if world.robot.task == 'left':
                goal_lane = URIRef("http://example.com/intersection/road_left/lane_left")
                goal_side = URIRef("http://example.com/intersection/road_left/side_left")

            phi = world.map_dict[goal_lane].get('orientation')
            #rint(world.map_dict[goal_side].get('poly').coords[:])
            centerline = shift_line(world.map_dict[goal_side].get('poly').coords[:], -0.25*w)
            print(centerline.coords[:])
            lane = 0.5*(world.map_dict[goal_lane].get('poly').bounds[3]+world.map_dict[goal_lane].get('poly').bounds[1])
            self.params.append(phi)
            self.params.append(centerline)
            #print(lane)
        print(self.skill)

    def execute_skill(self, world, control):
        if self.skill == 'move_in_lane':
            control.move_in_lane(world, *self.params)

        if self.skill == 'turn':
            control.turn(world, *self.params)

        if self.skill == 'stop':
            control.stop(world)
