from rdflib import URIRef
from skills import Condition

class TrafficRules():
    def __init__(self):
        pass

    def config_rules(self, world):
        self.rules_approaching_middle = []
        self.init_rules_approaching_middle(world)

    def get_rules(self, condition):
        if condition.object == URIRef("http://example.com/intersection/middle") and condition.relation == URIRef("http://example.com/approaches"):
            return self.rules_approaching_middle

    def init_rules_approaching_middle(self, world):
        self.rule1 = Condition('external', negation=True, for_all=True)
        self.rule1.subject = URIRef("http://example.com/obstacle")
        self.rule1.relation = URIRef("http://example.com/is_on")
        self.rule1.object = URIRef("http://example.com/intersection/middle")
        self.rules_approaching_middle.append(self.rule1)

        # deze twee conditions mogen alleen niet gelden voor hetzelfde obstacle
        self.rule2 = Condition('external', negation=True, for_all=True)
        self.rule2.subject = URIRef("http://example.com/obstacle")
        self.rule2.relation = URIRef("http://example.com/is_on")
        self.rule2.object = world.goal
        self.rules_approaching_middle.append(self.rule2)

        self.rule3 = Condition('external', negation=True, for_all=True)
        self.rule3.subject = URIRef("http://example.com/obstacle")
        self.rule3.relation = URIRef("http://example.com/after")
        self.rule3.object = URIRef("http://example.com/intersection/middle")
        self.rules_approaching_middle.append(self.rule3)

        self.rule4 = Condition('external', negation=True, for_all=True)
        self.rule4.subject = URIRef("http://example.com/vehicle")
        self.rule4.relation = URIRef("http://example.com/approaches")
        self.rule4.object = URIRef("http://example.com/intersection/middle")
        self.rules_approaching_middle.append(self.rule4)

        self.rule5 = Condition('external', negation=True, for_all=True)
        self.rule5.subject = URIRef("http://example.com/vehicle")
        self.rule5.relation = URIRef("http://example.com/is_on")
        self.rule5.object = URIRef("http://example.com/intersection/road")
        self.rules_approaching_middle.append(self.rule5)

        self.rule6 = Condition('external', negation=True, for_all=True)
        self.rule6.subject = URIRef("http://example.com/intersection/road") # from rule 5
        self.rule6.relation = URIRef("http://example.com/right_of")
        self.rule6.object = URIRef("http://example.com/intersection/road") # current road
        self.rules_approaching_middle.append(self.rule6)