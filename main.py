import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import json
from rdflib import URIRef
from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
from basic_functions import *
from perception import Perception
from worldmodel import Worldmodel
from reasoner import Reasoner
from simulator import Simulator
from monitor import Monitor
from control import Control
from global_variables import g, dt, TURTLE_LENGTH, TURTLE_WIDTH, TURTLE_VELOCITY, EX     
from queries import *      
import time
from skills import *

## Situation to test
sit = 'intersection'

## initialize objects
simulator = Simulator(sit)       # configure global map
simulator.set_map(sit)

simulator.add_robot('turtle1', TURTLE_LENGTH, TURTLE_WIDTH, TURTLE_VELOCITY, 0.3, 'down')
add_robot('turtle1', 'laser_range_finder', 'velocity_control', 'encoders')  # prior knowledge for kg

world = Worldmodel(simulator.robots[0])
world.set_goal('left')

perception = Perception()
reasoner = Reasoner()
monitor = Monitor()
control = Control()


figure(num=1, figsize=(6, 6), dpi=80)
fig = figure(1)
move_figure(fig, 200, 400)
# figure(num=2, figsize=(6, 6), dpi=80)
# fig = figure(2)
# move_figure(fig, 700, 400)
# figure(num=3, figsize=(6, 6), dpi=80)
# fig = figure(3)
# move_figure(fig, 1200, 400)

# figure(num=4, figsize=(6, 6), dpi=80)
# fig = figure(4)
# move_figure(fig, 1700, 400)


#planner.set_plan(world)


t = 0
waiting = True
while True:
    plt.figure(1)   
    plt.cla()
    plt.title('Simulator')
    simulator.simulate()

    #plt.figure(2)
    #plt.cla()
    #plt.title('Perception inputs')
    inputs = simulator.perceived_features()
    simulator.plot_input_features()


    ## From here starts the robot software:
    
    #plt.figure(3)
    #plt.cla()
    #plt.title('Relative world model')
    #plt.xlim(-15,15)
    #plt.ylim(-10,40)

    #start = time.time()
    
    
    #end = time.time()
    #print(f'Time to run control/planning: {end-start}')
    #perception.perceive(world, inputs)

    #monitor.monitor(world)
    reasoner.reason(world)
    #world.plot_relative_areas()
    #world.robot.plot_rel_robot()
    #control.move(world, simulator)

    #perception.check_worldmodel(world, perception.inputs)
    #monitor.run(simulator, world, planner, perception, control)

    # if round(t/dt) % 10 == 0:
    #     control_task = planner.iterative_planning(world)
    # perception.perceive(simulator, world, perception.inputs)
        
    # ## hier gebeurd pas de associatie met geometrie
    # whole = query_part_of(control_task)
    # lines = world.relative_areas[whole].boundary
    # control_line = LineString((lines.coords[1], lines.coords[2]))
    
    
    # world.set_subgoal(control_line)
    # world.plot_relative_areas()
    # simulator.robots[0].plot_rel_robot()
    # control.move(simulator.robots[0], world, simulator)

    #plt.figure(4)
    #plt.cla()
    #plt.title('Absolute world model')
    #world.plot_absolute_areas()

    

    plt.pause(dt)
    #round(t/dt) % 10
    t += dt


    if waiting:
        plt.waitforbuttonpress()
        waiting = False