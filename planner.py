from global_variables import g
from queries import query_connectivity, query_driveable_location, query_plan, query_mereology, query_line_connection, has_a, query_if_polygon, query_if_line, query_common_whole, query_driveable_part
## hier moeten de resources van de robot, en de affordances gematched worden tot een plan
from rdflib import URIRef

class Planner():
    def __init__(self):
        self.plan = {}
        self.plan_number = 0

    def set_plan(self, world):  # most abstract plan
        self.plan[str(self.plan_number)] = [world.start, world.goal]
        #self.plan_number += 1

    def iterative_planning(self, world):
        ## keeps planning untill actionable by control
        if not self.plan:
            print("Error plan not set")

        current_start = self.plan[str(self.plan_number)][0]
        current_goal = self.plan[str(self.plan_number)][1]


        self.plan_finished = False
        polygons = False
        polygon1 = query_if_polygon(current_start)
        polygon2 = query_if_polygon(current_goal)
        if polygon1 and polygon2:
            polygon = True
        
        new_start = current_start
        new_goal = current_goal


        while not self.plan_finished: # moet hier miss ook de if adjacency query?
            new_plan = self.skill(new_start, new_goal)
            #print(new_plan)
            new_start = new_plan[0]
            new_goal = new_plan[1]

            # polygon1 = query_if_polygon(new_start)
            # polygon2 = query_if_polygon(new_goal)
            # if polygon1 and polygon2:
            #     polygons = True

            world.add_robot_pos(new_plan[0])
            
            self.plan_number += 1
            self.plan[str(self.plan_number)] = new_plan

        
        #print(self.plan)

    def skill(self, start, goal):        # plan is nu een dictionary, maar moet miss anders        
        # common whole query
        common_whole = query_common_whole(start, goal)  # moet dit miss alleen bij de eerste keer, later kun je ervanuit gaan omdat je van boven komt
        parts = has_a(common_whole)
        #for part in parts:

        #plan = query_topology(whole, start, goal)
        adjacency = False
        plan = query_connectivity(start, goal)
        if len(plan) == 2:
            adjacency = True

        if adjacency:   ## requires more information from world model/perception
            polygon_start = query_if_polygon(start)
            if not polygon_start:
                sub_parts_start = has_a(start)
                #sub_start = query_sub_start(sub_parts_start)
                sub_start = query_driveable_part(sub_parts_start)
            else:
                #sub_parts_start = [start]
                sub_start = start
            polygon_goal = query_if_polygon(goal)
            if not polygon_goal:
                sub_parts_goal = has_a(goal)
                #sub_goal = query_sub_goal(sub_parts_goal)
                sub_goal = query_driveable_part(sub_parts_goal)
            else:
                #sub_parts_goal = [goal]
                sub_goal = goal

            if polygon_start and polygon_goal:
                plan = query_line_connection(sub_start, sub_goal)
                self.plan_finished = True
            else:
                plan = query_connectivity(sub_start, sub_goal)

        #print(plan)
        return plan

    def meta_plan(self, monitor, world, robot, goal): # dit voelt zo niet composable, ik neem iig aan dat de goal is naar een bepaalde plek te gaan, maar hier zou nog een abstractie laag op kunnen
        #self.goal_area = query_driveable_location(goal)
        self.goal = goal
        self.current_area = world.current_area(robot) # too concrete
        ## robot first needs to understand where am I, and where is the goal
        self.current_pos = query_mereology(self.goal, self.current_area)
        monitor.set_meta_plan([self.current_pos, self.goal])
        #print([self.current_pos, self.goal])
        
    def plan1(self, monitor, world, robot):
        self.connectivity = query_connectivity(self.current_pos, self.goal)
        monitor.set_plan(self.connectivity)
        #print(self.connectivity)
        # how do you go from one part to the next
        #self.meta_plan = query_plan(connectivity)

    def sub_plan1(self, monitor):
        sub_start = self.connectivity[0]
        sub_goal = self.connectivity[1]
        self.line_connection = query_line_connection(sub_start, sub_goal)
        monitor.set_direction(self.line_connection)
        #print(self.line_connection)


        