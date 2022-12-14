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
from queries import add_robot, uri_from_label         
import time

## Situation to test
sit = 'intersection'

## initialize objects
world = Worldmodel()
world.set_goal('left')
simulator = Simulator(sit)       # configure global map
simulator.set_map(sit)

goal = URIRef("http://example.com/intersection/road4") # "At the intersection go left"
simulator.add_robot('turtle1', goal, TURTLE_LENGTH, TURTLE_WIDTH, TURTLE_VELOCITY, 'down')
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

waiting = True
while True:
    plt.figure(1)   
    plt.cla()
    plt.title('Simulator')
    simulator.simulate()

    plt.figure(2)
    plt.cla()
    plt.title('Perception')
    ## Assumed input feature

    #p = LineString([(0, 0), (1, 0)])
    #plt.plot(*p.xy)
    #start = time.time()
    perception_inputs = simulator.perceived_features()
    #end = time.time()
    #print(f'Time to run perception inputs: {end-start}')
    simulator.plot_input_features()

    plt.figure(3)
    plt.cla()
    plt.title('Robot')
    plt.xlim(-15,15)
    plt.ylim(-10,40)
    
    #side_abs_pos = np.array([60.0, 40.0])
    #side = abs_to_rel(simulator.robots[0], side_abs_pos, 80, 1)     # moet deze het niet met een polygon doen ipv alleen de positie. middenpunt is natuurlijk wel nodig als je m wilt draaien (klopt dit?)

    #print(simulator.map.map_dict['0']['poly'])
    #simulator.plot_rel_obj(side)
    #print(side['polygon'])
    #perception.perceive(simulator, world, sit, side['polygon'])
    

    #planner.set_plan(world)
    #planner.iterative_planning(world)
    perception.perceive(simulator, world, perception_inputs)

    simulator.robots[0].plot_rel_robot()
    #planner.skill(world, simulator.robots[0], simulator.robots[0].goal)
    #planner.meta_plan(monitor, world, simulator.robots[0], simulator.robots[0].goal)
    #planner.plan(monitor, world, simulator.robots[0])
    #planner.sub_plan(monitor)
    #control.move(simulator.robots[0], monitor)
    simulator.robots[0].yaw += 0.01*math.pi
    plt.pause(dt/100)
    
    if waiting:
        plt.waitforbuttonpress()
        waiting = False