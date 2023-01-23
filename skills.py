

def traverse_intersection():
    pass

def go_left():
    # conditions: robot_pos road_current & robot_pos intersection, task: left, effect: robot_pos 
    skill = ['traverse road','turn left','traverse road']
    return skill

def traverse_road():
    pass

def move_in_lane():
    # conditions: robot_pos lane, affordance: driveability, sub skill: turn and drive, no obstacle on lane in front of robot, effect: robot_pos end of road
    phi_lane = 0
    skill = turn_drive(phi_des=phi_lane)
    return skill


def turn_drive(phi_des):
    return 0
