from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from worldmodel2 import WorldModel, Robot, GlobalPlanner
import random
import json

with open('conf.json') as f:
    config = json.load(f)
    f.close

TURTLE_LENGTH = config['turtle_length']
TURTLE_WIDTH = config['turtle_width']
TURTLE_VELOCITY = config['turtle_velocity']
dt = config['dt']
second_robot = False

## Create map
sit = "Intersection"
world = WorldModel(sit)              # Intersection or Road
directions = ['down', 'up', 'right', 'left']

## Create turtles with initial position
start = 'down'
goal = 'up'
turtle1 = Robot(start, goal, TURTLE_LENGTH, TURTLE_WIDTH, 'turtle1', world, color='magenta')
world.add_robot(turtle1)

if second_robot:
    start = 'right'
    goal = 'left'
    turtle2 = Robot(start, goal, TURTLE_LENGTH, TURTLE_WIDTH, 'turtle2', world)
    world.add_robot(turtle2)

## Create goal and trajectory to goal

#turtle1.create_goal(goal)


#turtle2.create_goal(goal)
# turtle1.plot_constraints()
# plt.show()
## Simulate

#figure(num=1, figsize=(1, 6), dpi=80)
waiting = False
while True:
    plt.cla()
    world.plot_map()
    
    #world.plot_road()
    

    if turtle1.finished:
        world.remove_robot('turtle1')
        start, goal = random.sample(directions, k=2)
        #start = 'right'
        #goal = 'left'
        turtle1 = Robot(start, goal, TURTLE_LENGTH, TURTLE_WIDTH, 'turtle1', world, color='magenta')
        #turtle1.create_goal(goal)
        world.add_robot(turtle1)
    if second_robot:
        if turtle2.finished:
            world.remove_robot('turtle2')
            start, goal = random.sample(directions, k=2)
            turtle2 = Robot(start, goal, TURTLE_LENGTH, TURTLE_WIDTH, 'turtle2', world)
            #turtle2.create_goal(goal)
            world.add_robot(turtle2)
    
    turtle1.plot_robot()
    turtle1.move()
    #turtle1.plot_constraints()

    if second_robot:
        turtle2.plot_robot()
        turtle2.move()


    plt.text(0.05, 20, "Turtle (magenta): ")
    in_areas = []
    for key, value in turtle1.current_areas.items():
        if turtle1.current_areas[key] > 0.5*turtle1.rob_area:
            in_areas.append(key)
    plt.text(0.05, 16, in_areas)

    # plt.text(0.05, 10, "Turtle (cyan): ")
    # in_areas = []
    # for key, value in turtle2.current_areas.items():
    #     if turtle2.current_areas[key] > 0.5*turtle2.rob_area:
    #         in_areas.append(key)
    # plt.text(0.05, 6, in_areas)
    
    turtle1.check_sit()
    turtle1.plot_situation()

    plt.pause(dt/100)
    
    if waiting:
        plt.waitforbuttonpress()
        waiting = True
    