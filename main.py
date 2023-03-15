import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import json
from rdflib import URIRef
from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
from basic_functions import *
from perception import Perception
from worldmodel import WorldModel
from skill_model import SkillModel
from reasoner import Reasoner
from simulator import Simulator
from monitor import Monitor
from control import Control
from global_variables import g, dt, AV_LENGTH, AV_WIDTH, AV_VELOCITY, AV_OMEGA, w, l, EX    
from queries import *      
import time
from skills import *
from sys import argv

## Situation to test
map = 'two-lane_intersection'

## initialize objects
simulator = Simulator()
simulator.set_map(map)

simulator.add_robot('AV1', AV_LENGTH, AV_WIDTH, AV_VELOCITY, AV_OMEGA, start='down', task='left')
simulator.add_robot('AV2', AV_LENGTH, AV_WIDTH, AV_VELOCITY, AV_OMEGA, start='right', task='up', color='purple')

#simulator.add_obstacle('obstacle1', np.array([55, 20]))

AV1_world = WorldModel(g, simulator.robots['AV1'], simulator.map)
AV2_world = WorldModel(g, simulator.robots['AV2'], simulator.map)



skill_model = SkillModel()
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

    plt.text(0.05, 22, "AV1: ", fontsize = 16)
    plt.text(0.05, 18, AV1_world.robot.task, fontsize = 16)
    plt.text(0.05, 12, "AV2: ", fontsize = 16)
    #plt.text(0.05, 8, AV2_world.robot.task, fontsize = 16)

    #plt.figure(2)
    #plt.fill(*AV1_world.right_lane.exterior.xy, color='red')
    
    
    #print(query_current_pos(world.kg, world.AV_uri))
    
    skill_model.monitor_skills(AV1_world, control)
    skill_model.monitor_skills(AV2_world, control) 

    #control.execute_skill(world)
    control.actuate(AV1_world, simulator)
    control.actuate(AV2_world, simulator)

    AV1_world.update(simulator)
    AV2_world.update(simulator)

    plt.pause(dt/10)
    #round(t/dt) % 10
    t += dt

    if waiting:
        plt.waitforbuttonpress()
        if 'wait' not in argv:
            waiting = False