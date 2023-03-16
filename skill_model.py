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
        ## Get current skill (here I assume skill is known and position needs to be queried from KG)
        skill = world.plan[str(world.plan_step)]['skill']

        ## Check conditions:
        # todo: check the conditions of the skill, for I know I just preprogram it 
        
 
        ## First condition is being on an area that is driveable
        area_uri = query_is_on(world.g, world.robot.uri)
        affordances_current = query_affordances(world.g, area_uri)
        
        ## Second condition is that also the next/approaching area is driveable
        approaches_list = query_approaches(world.g, world.robot.uri)
        
        if approaches_list:
            next_affordances = query_affordances(world.g, approaches_list[-1])
           
            if not next_affordances:
                self.condition_failed = True
                print('NO SKILL POSSIBLE')
    
            ## Due to the rule that there always should be a skill available, when you cannot wait on the next position,
            ## requires also the next position to be driveable, which in turn requires looking further by increasing the horizon
            elif not EX.waiting in next_affordances:   
                world.horizon_length += 1

        ## later integrate the above with vehicles and obstacles that inhibit driveability.
        ## however for convenience first do this separetely to get things working

        passing_robot = query_passes(world.g, world.robot.uri)
        conflict_robot = query_conflict(world.g, world.robot.uri)

        if passing_robot:
            ## stop in first area of horizon list.
            #if world.robot.name == 'AV1':
            #print(f'{world.robot.name}: passing')
            world.skill = 'wait'

        elif conflict_robot:
            #print(f'{world.robot.name}: conflict')
            world.skill = 'check_priority'

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


    def select_skill2(self, world):
        world.skill = 'drive'
        
    def select_skill3(self, world):
        ## miss moet dit gwn n reasoning functie worden. Dan ga je echt adh van failed conditions iets queryen. 
        #print(world.failed_condition.type)
        if world.failed_condition.effect:
            self.select_default_skill(world)
        if world.failed_condition.type == 'check_rules':
            self.traffic_rules = 'priority'
            self.check_traffic_rules(world, self.traffic_rules)
            #print(world.condition_failed)
            if world.condition_failed:
                #world.skill = 'stop'
                world.wait = True
                
            
        world.check_rules = False
        while world.check_rules:
            ## check rules die bij het overlappende deel horen
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
            
        world.condition_failed = False

    def select_default_skill2(self, world):
        ## ipv queryen kun je ook gwn de context meenemen, bijv als de move in lane succesvol is dan weet je waar het nu is
        world.robot_pos = query_is_on(world.g, world.robot.uri)
        type_pos = query_type(world.g, world.robot_pos)
     
        if str(type_pos) == "http://example.com/lane":
            world.skill = 'move_in_lane'
        elif str(type_pos) == "http://example.com/middle":
            world.skill = 'turn'
        else:
            world.skill = 'stop'
        #print(f'skill, {world.robot.name}: {world.skill}')

    def select_default_skill(self, world):
        world.plan_step += 1


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
            pass
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


    def config_skill2(self, world):
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
        #print(f'{world.robot.name}: {world.skill}: {world.plan_step}')
        if world.skill == 'drive':
            control.drive(world)
        elif world.skill == 'wait':
            if world.robot.poly.intersects(world.wait_pos) and world.robot.poly.intersection(world.wait_pos).area > 0.95 * world.robot.poly.area:
                control.stop(world)
            else:
                control.drive(world)



    def execute_skill2(self, world, control):
        #print(f'omega {world.robot.name}: {world.omega}')
        skill = world.plan[str(world.plan_step)]['skill']

        if world.wait:
            distance = 40-world.robot.pos[1]
            if distance < 0.5*world.robot.length + 2:
                world.plan[str(world.plan_step)]['parameters']['velocity'] = 0
        else:
            world.plan[str(world.plan_step)]['parameters']['velocity'] = world.robot.velocity_max

        #print(skill)
        if skill == 'move_in_lane':
            control.drive(world)

        if skill == 'turn':
            #control.turn(world)
            control.drive(world)

        if skill == 'stop':
            control.stop(world)
        #print(world.omega)

    
