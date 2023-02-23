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
from global_variables import g, dt, TURTLE_LENGTH, TURTLE_WIDTH, TURTLE_VELOCITY, EX     
from queries import *      
import time
from skills import *

## Situation to test
map = 'two-lane_intersection'

## initialize objects
simulator = Simulator()       # configure global map
simulator.set_map(map)

simulator.add_robot('AV', TURTLE_LENGTH, TURTLE_WIDTH, TURTLE_VELOCITY, 1, start='down', task='left')
#simulator.robots[0].yaw += 0.1

#world = Worldmodel(simulator.robots[0])
world = WorldModel(simulator.robots[0])
world.init_geometric_map(simulator.map)
world.init_kg(g)

skill_model = SkillModel()
control = Control()

#world.set_goal('left')
#world.update()


figure(num=1, figsize=(8, 8), dpi=80)
fig = figure(1)
move_figure(fig, 1100, 150)
# figure(num=2, figsize=(6, 6), dpi=80)
# fig = figure(2)
# move_figure(fig, 700, 400)

t = 0
waiting = True
while True:
    plt.figure(1)   
    plt.cla()
    plt.title('Simulator')
    simulator.simulate()
    
    world.update(simulator)

    #print(query_current_pos(world.kg, world.AV_uri))
    
    skill_model.monitor_skills(world, control)
    
    #control.execute_skill(world)
    control.actuate(simulator)

    plt.pause(dt)
    #round(t/dt) % 10
    t += dt


    if waiting:
        plt.waitforbuttonpress()
        waiting = False