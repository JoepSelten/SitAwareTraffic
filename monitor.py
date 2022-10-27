

class Monitor():
    def __init__(self):
        pass

    def check_area(self, robot_box, world):             # only needs a few if statements to due hierarchical structure
        if robot_box.intersects(world.behaviour_map.box):
            if robot_box.intersects(world.behaviour_map.lane.forward_area.box):
                self.subtask = 'Drive forward'
            elif robot_box.intersects(world.behaviour_map.lane.side_of_lane.box):
                self.subtask = 'Correct to the left'
            elif robot_box.intersects(world.behaviour_map.other_side_of_lane.box):
                self.subtask = 'Correct to the right'
        else:
            print("Out of behaviour map!!!")

