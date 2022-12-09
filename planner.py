from global_variables import g
from queries import query_connectivity, query_driveable_location, query_plan, query_mereology
## hier moeten de resources van de robot, en de affordances gematched worden tot een plan

class Planner():
    def __init__(self):
        pass
 
    def meta_plan(self, monitor, world, robot, goal): # dit voelt zo niet composable, ik neem iig aan dat de goal is naar een bepaalde plek te gaan, maar hier zou nog een abstractie laag op kunnen
        #self.goal_area = query_driveable_location(goal)
        self.goal = goal
        self.current_area = world.current_area(robot) # too concrete
        ## robot first needs to understand where am I, and where is the goal
        self.current_pos = query_mereology(self.goal, self.current_area)
        monitor.set_meta_plan([self.current_pos, self.goal])
        print([self.current_pos, self.goal])
        
    def plan(self, monitor, world, robot):
        self.connectivity = query_connectivity(self.current_pos, self.goal)
        monitor.set_plan(self.connectivity)
        print(self.connectivity)
        # how do you go from one part to the next
        #self.meta_plan = query_plan(connectivity)
        