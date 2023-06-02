import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import json
from rdflib import URIRef
from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
from basic_functions import *
from worldmodel import WorldModel
from algorithm import Algorithm_TrafficRules
from simulator import Simulator
from control import Control
from global_variables import g, dt, AV_LENGTH, AV_WIDTH, AV_VELOCITY, AV_OMEGA, w, l, EX    
from queries import *      
import time
from sys import argv

## Situation to test
map = 'two-lane_intersection'

## initialize objects
simulator = Simulator()
simulator.set_map(map)



simulator.add_robot(EX.AV1, AV_LENGTH, AV_WIDTH, AV_VELOCITY, AV_OMEGA, start='down', task='left', delay=0)
simulator.add_robot(EX.AV2, AV_LENGTH, AV_WIDTH, AV_VELOCITY, AV_OMEGA, start='right', task='left', delay=0)
simulator.add_robot(EX.AV3, AV_LENGTH, AV_WIDTH, AV_VELOCITY, AV_OMEGA, start='left', task='down', delay=0)

simulator.add_obstacle(EX.obstacle1, np.array([55, 20]))
simulator.add_obstacle(EX.obstacle2, np.array([55, 25]))
simulator.add_obstacle(EX.obstacle3, np.array([55, 35]))
#simulator.add_obstacle(EX.obstacle4, np.array([55, 45]))
#simulator.add_obstacle(EX.obstacle5, np.array([55, 55]))
#simulator.add_obstacle(EX.obstacle6, np.array([35, 55]))
#simulator.add_obstacle(EX.obstacle7, np.array([65, 45]))

AV1_world = WorldModel(simulator.robots[EX.AV1], simulator.map)
AV2_world = WorldModel(simulator.robots[EX.AV2], simulator.map)
AV3_world = WorldModel(simulator.robots[EX.AV3], simulator.map)

algorithm = Algorithm_TrafficRules()
control = Control()

figure(num=1, figsize=(8, 8), dpi=80)
fig = figure(1)
move_figure(fig, 1100, 150)

#figure(num=2, figsize=(8, 8), dpi=80)
#fig = figure(1)
#move_figure(fig, 1300, 150)


t = 0
waiting = True

while True:
    plt.figure(1)   
    plt.cla()
    plt.xlim(0, 2*l+w)
    plt.ylim(0, 2*l+w)
    plt.axis('equal')
    plt.title('Simulator')
    
    simulator.simulate()

    # plt.text(0.05, 22, "AV1: ", fontsize = 16)
    # plt.text(0.05, 18, AV1_world.robot.task, fontsize = 16)
    # plt.text(0.05, 12, "AV2: ", fontsize = 16)
    # plt.text(0.05, 8, AV2_world.robot.task, fontsize = 16)
    
    algorithm.run(AV1_world, control)
    algorithm.run(AV2_world, control) 
    algorithm.run(AV3_world, control) 

    #if t > 0.3:
    control.actuate(AV1_world, simulator, t)
    control.actuate(AV2_world, simulator, t)
    control.actuate(AV3_world, simulator, t)

    AV1_world.update(simulator)
    AV2_world.update(simulator)
    AV3_world.update(simulator)

    plt.pause(dt/10)
    #round(t/dt) % 10
    t += dt

    if waiting:
        plt.waitforbuttonpress()
        if 'wait' not in argv:
            waiting = False