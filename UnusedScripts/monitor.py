from shapely.geometry import LineString
from queries import *

class Monitor():
    def __init__(self):
        self.previous_area = 0
        self.control_task = 0
        self.previous_inputs = 0

    def monitor(self, world):
        ## sets booleans in world model, discrete decision making
        self.check_area(world)
        #self.check_inputs(world)
        self.check_config(world)
        
    def check_area(self, world):
        world.different_area = False
        previous_area = world.area
        current_area = world.check_area()
        if previous_area != current_area:
            world.different_area = True

    def check_inputs(self, world):
        pass

    def check_config(self, world):
        world.map_configured = False
        parts = has_first_layer_geometries(world.situation)
        print(f'parts: {len(parts)}')
        print(world.number_of_absolute_areas)
        if len(parts) == world.number_of_absolute_areas:
            world.map_configured = True

    def run(self, simulator, world, planner, perception, control):
        ## init conditions
        self.diff_area = False
        self.plan = False
        self.perception_query = False
        self.perception_geom = False

        robot = simulator.robots[0]

        self.current_area = world.check_area()
        if self.previous_area != self.current_area:
            self.diff_area = True
            self.previous_area = self.current_area 

        self.plan_monitor(planner)
        if self.plan:
            self.control_task = planner.iterative_planning(world)

        self.perception_monitor(perception)
        if self.perception_query:
            perception.perceive_query(world, perception.inputs)
        if self.perception_geom:
            perception.perceive_geom(world, simulator, perception.inputs)
        self.previous_inputs = len(perception.inputs)
        
        if self.control_task:
            #print(self.control_task)
            ## hier gebeurd pas de associatie met geometrie
            whole = query_part_of(self.control_task)
            #print(f'whole: {whole}')
            lines = world.relative_areas[whole].boundary
            control_line = LineString((lines.coords[1], lines.coords[2]))
        
            world.set_subgoal(control_line)
            world.plot_relative_areas()
            simulator.robots[0].plot_rel_robot()
            control.move(simulator.robots[0], world, simulator)

    def plan_monitor(self, planner):
        ## If not planned yet
        if not planner.plan:
            self.plan = True

        if self.diff_area:
            self.plan = True

        ## Other conditions, eg when things change or when goal is reached, or when more information from perception is available

    def perception_monitor(self, perception):
        if self.diff_area:
            self.perception_query = True
        if len(perception.inputs) != self.previous_inputs:   ## later iets mooier doen, bijv daadwerkelijk inputs vergelijken
            self.perception_query = True
        self.perception_geom = True


    def set_meta_plan(self, meta_plan):
        self.meta_plan = meta_plan
        self.meta_task_number = 0
        self.meta_subtask = meta_plan[self.meta_task_number]

    def set_plan(self, plan):
        self.plan = plan
        self.task_number = 0
        self.subtask = self.plan[self.task_number]

    def set_direction(self, direction):
        self.direction = direction

    def meta_monitor(self, world): # check if mereology changes or if goal succesful
        pass

    def monitor2(self, world, robot): # check if topology changes or if next step in connectivity plan
        current_area = world.current_area(robot)
        if current_area == self.subtask:
            print("Still going good, keep going!")

        elif current_area != self.subtask:
            if current_area == self.plan[self.task_number+1]:
                print("Subtask succesful")
            else:
                print("ERROR: requires replanning")
            


    # def check_area(self, robot, world):             # only needs a few if statements to due hierarchical structure
    #     if robot.intersects(world.behaviour_map.box):
    #         if robot.intersects(world.behaviour_map.lane.forward_area.box):
    #             self.subtask = 'Drive forward'
    #         elif robot.intersects(world.behaviour_map.lane.side_of_lane.box):
    #             self.subtask = 'Correct to the left'
    #         elif robot.intersects(world.behaviour_map.other_side_of_lane.box):
    #             self.subtask = 'Correct to the right'
    #     else:
    #         print("Out of behaviour map!!!")

