

class Monitor():
    def __init__(self):
        pass

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

    def monitor(self, world, robot): # check if topology changes or if next step in connectivity plan
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

