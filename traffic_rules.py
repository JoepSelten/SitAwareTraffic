from rdflib import URIRef
from skills import Condition



class ApproachingMiddle():
    def __init__(self):
        pass

    def config_rules(self, world):
        self.condition_list = []
        self.init_rules(world)

    def init_rules(self, world):
        self.rule1 = Condition('external', negation=True, for_all=True)
        self.rule1.subject = URIRef("http://example.com/obstacle")
        self.rule1.relation = URIRef("http://example.com/is_on")
        self.rule1.object = URIRef("http://example.com/intersection/middle")
        self.condition_list.append(self.rule1)

        # deze twee conditions mogen alleen niet gelden voor hetzelfde obstacle. Of moet het telkens maar een condition checken
        self.rule2 = Condition('external', negation=True, for_all=True)
        self.rule2.subject = URIRef("http://example.com/obstacle")
        self.rule2.relation = URIRef("http://example.com/is_on")
        self.rule2.object = world.goal
        self.condition_list.append(self.rule2)

        self.rule3 = Condition('external', negation=True, for_all=True)
        self.rule3.subject = URIRef("http://example.com/obstacle")
        self.rule3.relation = URIRef("http://example.com/after")
        self.rule3.object = URIRef("http://example.com/intersection/middle")
        self.condition_list.append(self.rule3)

        self.rule4 = Condition('check_rules', negation=True, for_all=True)
        self.rule4.subject = URIRef("http://example.com/vehicle")
        self.rule4.relation = URIRef("http://example.com/approaches")
        self.rule4.object = URIRef("http://example.com/intersection/middle")
        self.condition_list.append(self.rule4)

        self.rule5 = Condition('external', negation=True, for_all=True)
        self.rule5.subject = URIRef("http://example.com/vehicle")
        self.rule5.relation = URIRef("http://example.com/is_on")
        self.rule5.object = URIRef("http://example.com/intersection/road")
        self.condition_list.append(self.rule5)

class Priority():
    def __init__(self):
        pass

    def config_rules(self, world):
        self.condition_list = []
        self.init_rules(world)

    def init_rules(self, world):
        self.rule1 = Condition('external', negation=True, for_all=True)
        self.rule1.subject = URIRef("http://example.com/vehicle")
        self.rule1.relation = URIRef("http://example.com/right_of")
        self.rule1.object = world.robot.uri
        self.condition_list.append(self.rule1)
