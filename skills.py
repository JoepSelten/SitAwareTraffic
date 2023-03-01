from rdflib import URIRef

class MoveInLane():
    def __init__(self):
        pass

    def config_skill(self, world):
        self.robot = world.robot
        self.lane = world.current_pos
        self.condition_list = []
        self.init_effects()
        self.init_conditions()
        
    def init_effects(self):
        self.effect = Condition('positional', effect=True)
        self.effect.subject = self.robot.uri
        self.effect.relation = URIRef("http://example.com/is_on")
        self.effect.object = URIRef("http://example.com/intersection/middle")
        self.condition_list.append(self.effect)
                
    def init_conditions(self):
        self.condition1 = Condition('positional')
        self.condition1.subject = self.robot.uri
        self.condition1.relation = URIRef("http://example.com/is_on")
        self.condition1.object = self.lane
        self.condition_list.append(self.condition1)

        ## deze alleen checken bij nieuwe detection?
        self.condition2 = Condition('external', negation=True, for_all=True)
        self.condition2.subject = URIRef("http://example.com/obstacle")
        self.condition2.relation = URIRef("http://example.com/in_front_of")
        self.condition2.object = self.robot.uri
        self.condition_list.append(self.condition2)

        self.condition3 = Condition('external', negation=True, for_all=True)
        self.condition3.subject = URIRef("http://example.com/vehicle")
        self.condition3.relation = URIRef("http://example.com/in_front_of")
        self.condition3.object = self.robot.uri
        self.condition_list.append(self.condition3)

        ## of hoort dit bij traffic rules?
        self.condition4 = Condition('check_rules', negation=True)
        self.condition4.subject = self.robot.uri
        self.condition4.relation = URIRef("http://example.com/approaches")
        self.condition4.object = URIRef("http://example.com/intersection/middle")
        self.condition_list.append(self.condition4)

class Turn():
    def __init__(self):
        pass

    def config_skill(self, world):
        self.robot = world.robot
        self.task = world.goal
        self.lane = world.current_pos
        self.condition_list = []
        self.init_effects()
        self.init_conditions()

    def init_effects(self):
        self.effect = Condition('positional', effect=True)
        self.effect.subject = self.robot.uri
        self.effect.relation = URIRef("http://example.com/is_on")
        self.effect.object = self.task
        self.condition_list.append(self.effect)
        
    def init_conditions(self):
        self.condition1 = Condition('positional')
        self.condition1.subject = self.robot.uri
        self.condition1.relation = URIRef("http://example.com/is_on")
        self.condition1.object = URIRef("http://example.com/intersection/middle")
        self.condition_list.append(self.condition1)

        ## deze alleen checken bij nieuwe detection?
        self.condition2 = Condition('external', negation=True, for_all=True)
        self.condition2.subject = URIRef("http://example.com/obstacle")
        self.condition2.relation = URIRef("http://example.com/in_front_of")
        self.condition2.object = self.robot.uri
        self.condition_list.append(self.condition2)

        self.condition3 = Condition('external', negation=True, for_all=True)
        self.condition3.subject = URIRef("http://example.com/vehicle")
        self.condition3.relation = URIRef("http://example.com/in_front_of")
        self.condition3.object = self.robot.uri
        self.condition_list.append(self.condition3)

class SlowDownAndStop():
    def __init__(self):
        pass

    def config_skill(self, world):
        self.robot = world.robot
        self.task = world.goal
        self.lane = world.current_pos
        self.condition_list = []
        self.init_effects()
        self.init_conditions()

    def init_effects(self):
        self.effect = Condition('positional', effect=True)
        self.effect.subject = self.robot.uri
        self.effect.relation = URIRef("http://example.com/is_on")
        self.effect.object = self.task
        self.condition_list.append(self.effect)

class Condition():
    def __init__(self, type, effect=False, negation=False, for_all=False):
        self.type = type
        self.effect = effect
        self.negation = negation
        self.for_all = for_all
        
class MultipleConditions():
    def __init__(self):
        pass
