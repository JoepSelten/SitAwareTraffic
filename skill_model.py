from queries import *
from simplegraph3 import EX
import shapely
import math
import matplotlib.pyplot as plt
from global_variables import w
from basic_functions import shift_line, extend_line
from skills import MoveInLane, Turn, SlowDownAndStop
from traffic_rules import ApproachingMiddle, Priority

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
        ## het initializeren moet eigenlijk niet iedere keer opnieuw gebeuren. later beter mengen met configuration enzo
        skill_obj = self.skill_dict.get(world.skill)
        if skill_obj:
            skill_obj.config_skill(world)

            # nu check ik alle conditions altijd, kan dit slimmer? Miss pas de external dingen checken als er een detectie is
            for condition in skill_obj.condition_list:
                check = self.check_condition(world, condition)
                #print(f'{condition.subject}, {condition.object}: {check}')
                if check == False:
                    world.condition_failed = True
                    world.failed_condition = condition
                    return
        world.condition_failed = False

        

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
            check = query_check(g, condition.subject, condition.relation, condition.object)
        elif condition.for_all:
            check = query_check_for_all(g, condition.subject, condition.relation, condition.object)
       
        if condition.effect is not condition.negation:
            return not check
        return check

    def monitor_skills(self, world, control):
        self.check_conditions(world)
        #self.select_skill(world) 
        while world.condition_failed:
            self.select_skill(world)
            #self.check_conditions(world)        
        
        #self.select_default_skill(world)
            self.config_skill(world)
        self.execute_skill(world, control)
        
    def select_skill(self, world):
        ## miss moet dit gwn n reasoning functie worden. Dan ga je echt adh van failed conditions iets queryen. 
        if world.failed_condition.effect:
            self.select_default_skill(world)
        if world.failed_condition.type == 'check_rules':
            world.check_rules = True
        while world.check_rules:
            if world.failed_condition.object == URIRef("http://example.com/intersection/middle") and world.failed_condition.relation == URIRef("http://example.com/approaches"):
                if world.failed_condition.subject == world.robot.uri:
                    self.traffic_rules = 'approaching_middle'
                elif world.failed_condition.subject == URIRef("http://example.com/vehicle") and world.failed_condition.relation == URIRef("http://example.com/approaches"):
                    self.traffic_rules = 'priority'
            elif world.failed_condition.subject == URIRef("http://example.com/vehicle") and world.failed_condition.relation == URIRef("http://example.com/right_of"):
                #print(f'{world.robot.name}, give priority!!!')
                plt.text(65, 22, f'{world.robot.name}, give priority!!!' , fontsize = 16)
                world.skill = 'stop'

                world.check_rules == False
                world.condition_failed = False
                return
            self.check_traffic_rules(world, self.traffic_rules)
        world.condition_failed = False

    def select_default_skill(self, world):
        ## ipv queryen kun je ook gwn de context meenemen, bijv als de move in lane succesvol is dan weet je waar het nu is
        world.robot_pos = query_is_on(g, world.robot.uri)
        type_pos = query_type(g, world.robot_pos)
     
        if str(type_pos) == "http://example.com/lane":
            world.skill = 'move_in_lane'
        elif str(type_pos) == "http://example.com/middle":
            world.skill = 'turn'
        else:
            world.skill = 'stop'
        #print(f'skill, {world.robot.name}: {world.skill}')

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
                return
        world.check_rules = False

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

        if world.skill == 'stop':
            pass

    def execute_skill(self, world, control):
        #print(f'omega {world.robot.name}: {world.omega}')
        skill = world.plan[str(world.plan_step)]['skill']
        if skill == 'move_in_lane':
            control.move_in_lane(world)

        if skill == 'turn':
            control.turn(world)

        if skill == 'stop':
            control.stop(world)
