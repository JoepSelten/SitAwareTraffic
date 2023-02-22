from queries import *
from simplegraph3 import EX

class SkillModel():
    def __init__(self):
        pass

    def select_skill(self, world):
        robot_pos = query_current_pos(world.kg, world.AV_uri)
        type_pos = query_type(world.kg, robot_pos)

        
        if str(robot_pos) == "http://example.com/intersection/road_current/lane_right":
            self.skill = 'move_in_lane'
        elif str(robot_pos) == "http://example.com/intersection/middle":
            self.skill = 'turn'
        elif str(robot_pos) == "http://example.com/intersection/road_left/lane_left":
            self.skill = 'move_in_lane'
        else:
            self.skill = 'stop'
        
    
    def config_skill(self, world):
        # moet eigenlijk alles al configureren adh van de info
        #query_required_parameters
        #query_from_world_model
        # if not able to configure, then select new skill
        # moet ik hier ook al de snelheid van de robot configureren?
        world.skill = self.skill
        


