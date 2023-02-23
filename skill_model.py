from queries import *
from simplegraph3 import EX
import shapely
import math

class SkillModel():
    def __init__(self):
        pass

    def monitor_skills(self, world, control):
        if world.skill_finished:
            world.skill_finished = False
            self.select_skill(world)
            self.config_skill(world)
        self.execute_skill(world, control)

    def select_skill(self, world):
        self.robot_pos = query_current_pos(world.kg, world.AV_uri)
        type_pos = query_type(world.kg, self.robot_pos)

        
        if str(self.robot_pos) == "http://example.com/intersection/road_current/lane_right" or str(self.robot_pos) == "http://example.com/intersection/road_right/lane_left" or \
                str(self.robot_pos) == "http://example.com/intersection/road_up/lane_left" or str(self.robot_pos) == "http://example.com/intersection/road_left/lane_left":
            self.skill = 'move_in_lane'
        elif str(self.robot_pos) == "http://example.com/intersection/middle":
            self.skill = 'turn'
        elif str(self.robot_pos) == "http://example.com/intersection/road_left/lane_left":
            self.skill = 'move_in_lane'
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
            if world.task == 'left':
                goal_lane = URIRef("http://example.com/intersection/road_left/lane_left")
                

            if world.task == 'right':
                goal_lane = URIRef("http://example.com/intersection/road_right/lane_left")

            phi = world.map_dict[goal_lane].get('orientation')
            lane = 0.5*(world.map_dict[goal_lane].get('poly').bounds[3]+world.map_dict[goal_lane].get('poly').bounds[1])
            self.params.append(phi)
            self.params.append(lane)
        print(self.skill)

    def execute_skill(self, world, control):
        if self.skill == 'move_in_lane':
            control.move_in_lane(world, *self.params)

        if self.skill == 'turn':
            control.turn(world, *self.params)

        if self.skill == 'stop':
            control.stop(world)
