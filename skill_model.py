from queries import *
from simplegraph3 import EX
import shapely
import math
from global_variables import w
from basic_functions import shift_line, extend_line
from skills import MoveInLane, Turn

class SkillModel():
    def __init__(self):
        self.skill_dict = {'move_in_lane': MoveInLane(),
                            'turn': Turn()
        }
        self.condition_failed = False

    def check_conditions(self, world):
        ## het initializeren moet eigenlijk niet iedere keer opnieuw gebeuren. later beter mengen met configuration enzo
        skill_obj = self.skill_dict[world.skill]
        skill_obj.config_skill(world)

        # nu check ik alle conditions altijd, kan dit slimmer? Miss pas de external dingen checken als er een detectie is
        for condition in skill_obj.condition_list:
            check = self.check_condition(world, condition)
            #print(f'{condition.subject}, {condition.object}: {check}')
            if check == False:
                world.condition_failed = True
                self.failed_condition = condition
                return

        ## eerst effect checken, als het gesatisfied is set skill finished op true, dan moet ie in t skill model automatisch naar de volgende skill gaan
        ## als het effect nog niet gesatisfied is check het de guards
        ## bij de move in lane skill hoort ook n skill, ben er nog niet uit of ik de hele road check of alleen de lane
        ## eerst guard is of de robot is-on lane, zo niet vergroot scope, check waar je wel bent en redeneer hoe je weer terug bij je plan komt
        ## dit is miss iets te complex, dus ik ga in het begin uit dat dit altijd zal kloppen
        ## Dan check je de overige positionele conditions. eerst de mereology van de scope. en evt als het nodig is de topology
        ## Als ze niet gesatisfied zijn check je hoe je dit moet oplossen. bijv overtaken of achter de andere vehicle blijven
        ## wat precies hangt af van de overtake rules
        ## Als het wel satisfied is check je de traffic rules. wat hierbij is, if robot approaches middle, check corresponding rules
        ## dit houdt in pas oversteken als je in een beweging kunt oversteken, dit houdt vervolgens in dat de target lane vrij moet zijn en dat de robot voorrang moet hebben
        ## wat als het geen voorrang heeft? dan wachten. Wat als de target lane niet vrij is? Oplossing reasonen met combi van turning en switch lane

    def check_condition(self, world, condition):
        if not condition.for_all:
            check = query_check(world.kg, condition.subject, condition.relation, condition.object)
        elif condition.for_all:
            check = query_check_for_all(world.kg, condition.subject, condition.relation, condition.object)
       
        if condition.effect is not condition.negation:
            return not check
        return check


    def associate(self, world):
        pass

    def monitor_skills(self, world, control):
        #print(f'same situation, {world.robot.name}: {world.same_situation}')
        self.check_conditions(world)
        if world.condition_failed:
            self.select_skill(world)

        # print(world.skill_finished)
        # print(world.same_situation)
        #if world.skill_finished and not world.same_situation:
         #   world.skill_finished = False
            ## moet hier ook nog n losse check of de is_on ook echt anders is
          #  self.select_default_skill(world)
            

        self.config_skill(world)
        self.execute_skill(world, control)
        
    def select_skill(self, world):
        if self.failed_condition.effect:
            self.select_default_skill(world)
        if self.failed_condition.type == 'traffic_rule':
            #check_traffic_rules
            pass

    def select_default_skill(self, world):
        world.robot_pos = query_is_on(world.kg, world.robot.uri)
        type_pos = query_type(world.kg, world.robot_pos)
        #print(f'robot pos, {world.robot.name}: {world.robot_pos}')
        #print(f'robot current pos, {world.robot.name}: {world.current_pos}')
        #print(type_pos)
        #if str(self.robot_pos) == "http://example.com/intersection/road_down/lane_right" or str(self.robot_pos) == "http://example.com/intersection/road_right/lane_left" or \
         #       str(self.robot_pos) == "http://example.com/intersection/road_up/lane_left" or str(self.robot_pos) == "http://example.com/intersection/road_left/lane_left":
        if str(type_pos) == "http://example.com/lane":
            world.skill = 'move_in_lane'
        elif str(type_pos) == "http://example.com/middle":
            world.skill = 'turn'
        else:
            world.skill = 'stop'
        #print(f'skill, {world.robot.name}: {world.skill}')
    
    def config_skill(self, world):
        world.skill_params = []
        if world.skill == 'move_in_lane':
            phi = world.map_dict[world.current_pos].get('orientation')
            world.skill_params.append(phi)
            #print(f'{world.robot.name}: {phi}')
            #world.skill = world.skill
            
            # zou ik hier control direct uitvoeren. Denk t het wel, beetje raar denk ik om het eerst via world model te sturen
        
        if world.skill == 'turn':
            # desired orientation, and also when to turn
            # should I query the higher level plan or task here?

            ## dit moet later via de graph gequeried worden
            if world.robot.task == 'down':
                side_uri = URIRef("http://example.com/intersection/road_down/side_left")
            if world.robot.task == 'right':
                side_uri = URIRef("http://example.com/intersection/road_right/side_left")
            if world.robot.task == 'up':
                side_uri = URIRef("http://example.com/intersection/road_up/side_left")
            if world.robot.task == 'left':
                side_uri = URIRef("http://example.com/intersection/road_left/side_left")

            phi = world.map_dict[world.goal].get('orientation')
            #side_uri = query_side_from_lane(world.kg, world.goal)
            side = world.map_dict[side_uri].get('poly')
            centerline = shift_line(side, -0.25*w)
            extended_centerline = extend_line(centerline, phi, w)

            world.skill_params.append(phi)
            world.skill_params.append(extended_centerline)
        #print(f'{world.robot.name}: {world.skill}')

    def execute_skill(self, world, control):
        #print(f'omega {world.robot.name}: {world.omega}')
        if world.skill == 'move_in_lane':
            control.move_in_lane(world, *world.skill_params)

        if world.skill == 'turn':
            control.turn(world, *world.skill_params)

        if world.skill == 'stop':
            control.stop(world)
