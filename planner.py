from global_variables import g
from queries import query_connectivity, query_driveable_location, query_plan
## hier moeten de resources van de robot, en de affordances gematched worden tot een plan

class Planner():
    def __init__(self):
        pass
 
    def plan(self, world, robot):
        goal_area = query_driveable_location(robot.goal)
        current_area = world.current_area(robot)       # query areas of current situation
        #current_meta_area = query_part_of(current_area)
        connectivity = query_connectivity(current_area, goal_area)
        # how do you go from one part to the next
        plan = query_plan(connectivity)
        print(plan)