import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import json
from rdflib import URIRef
from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
from basic_functions import trans, abs_to_rel, coordinate_transform_polygon, move_figure
from perception import Perception
from worldmodel import Worldmodel
from planner import Planner
from simulator import Simulator
from monitor import Monitor
from control import Control
from global_variables import g, dt, TURTLE_LENGTH, TURTLE_WIDTH, TURTLE_VELOCITY, EX     
from queries import *      
import time

## Situation to test
sit = 'intersection'

## initialize objects
world = Worldmodel()
world.set_goal('left')
simulator = Simulator(sit)       # configure global map
simulator.set_map(sit)

goal = URIRef("http://example.com/intersection/road4") # "At the intersection go left"
simulator.add_robot('turtle1', goal, TURTLE_LENGTH, TURTLE_WIDTH, TURTLE_VELOCITY, 0.3, 'down')
add_robot('turtle1', 'laser_range_finder', 'velocity_control', 'encoders')

perception = Perception()
planner = Planner()
monitor = Monitor()
control = Control()


figure(num=1, figsize=(6, 6), dpi=80)
fig = figure(1)
move_figure(fig, 200, 400)
figure(num=2, figsize=(6, 6), dpi=80)
fig = figure(2)
move_figure(fig, 700, 400)
figure(num=3, figsize=(6, 6), dpi=80)
fig = figure(3)
move_figure(fig, 1200, 400)

planner.set_plan(world)


t = 0
waiting = True
while True:
    plt.figure(1)   
    plt.cla()
    plt.title('Simulator')
    simulator.simulate()

    plt.figure(2)
    plt.cla()
    plt.title('Perception inputs')
    ## Assumed input feature

    #start = time.time()
    perception_inputs = simulator.perceived_features()
    
    simulator.plot_input_features()
    #end = time.time()
    #print(f'Time to run perception inputs: {end-start}')

    plt.figure(3)
    plt.cla()
    plt.title('World model')
    plt.xlim(-15,15)
    plt.ylim(-10,40)

    #start = time.time()
    
    
    #end = time.time()
    #print(f'Time to run control/planning: {end-start}')
    if round(t/dt) % 10 == 0:
        control_task = planner.iterative_planning(world)

    
    #if len(perception_inputs) == 2:
    perception.perceive(simulator, world, perception_inputs)
    
    #perception.perceive_intersection(simulator, world, perception_inputs)
        
    
    whole = query_part_of(control_task)
    lines = world.areas[whole].boundary
    #print(lines)
    control_line = LineString((lines.coords[1], lines.coords[2]))
    #print(control_line)
    world.set_subgoal(control_line)
    world.plot_areas()


    simulator.robots[0].plot_rel_robot()

    control.move(simulator.robots[0], world, simulator)

    plt.pause(dt)
    #round(t/dt) % 10
    t += dt


    if waiting:
        plt.waitforbuttonpress()
        waiting = False