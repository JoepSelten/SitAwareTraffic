from perception import Perception
from worldmodel import Worldmodel
from monitor import Monitor
from control import Control
from simulator import Simulator, RobotSim

import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import json
from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
from shapely import affinity
from rdflib import Graph
import pprint

from basic_areas import *
from robot import *
from traffic_areas import *
from basic_functions import trans

with open('conf.json') as f:
    config = json.load(f)
    f.close

TURTLE_LENGTH = config['turtle_length']
TURTLE_WIDTH = config['turtle_width']
TURTLE_VELOCITY = config['turtle_velocity']
ROAD_LENGTH = config['road_length']
ROAD_WIDTH = config['road_width']
dt = config['dt']
prediction_horizon = config['prediction_horizon']
H = round(prediction_horizon/dt)
b = config['boundary_thickness']


perception = Perception()
world = Worldmodel()
monitor = Monitor()
control= Control(dt, H)
simulator = Simulator("road")
g = Graph()
g.parse("kg.json", format="json-ld")
# for stmt in g:
#     pprint.pprint(stmt)


#rob_pos = [ROAD_LENGTH+0.75*ROAD_WIDTH, TURTLE_LENGTH]
rob_pos = np.array([0.0, 0.0])

rob = Robot(rob_pos, 0.7*math.pi, TURTLE_VELOCITY, TURTLE_LENGTH, TURTLE_WIDTH) 
rob.add_resource('velocity control')

#intersection = Intersection(np.array([ROAD_LENGTH+0.5*ROAD_WIDTH, ROAD_LENGTH+0.5*ROAD_WIDTH]), ROAD_LENGTH, ROAD_WIDTH)
#road = OneLaneRoad(np.array([0,0.5*ROAD_LENGTH]), ROAD_LENGTH, ROAD_WIDTH)

figure(num=1, figsize=(1, 6), dpi=80)
#figure(num=2, figsize=(1, 6), dpi=80)
figure(num=2, figsize=(1,6), dpi=80)
waiting = True
while True:
    plt.figure(1)
    plt.cla()
    simulator.simulate()
    simulator.robot.yaw +=0.01*math.pi
    #intersection.plot_area()

    plt.figure(2)
    plt.cla()
    side = simulator.simulate_relative()

    simulator.robot.get_robot_box()
    perception.recognize_sit('road', side)      # traffic sign + sides of the road
    perception.query_relations(g)
    perception.configure_map(g)

    #world.set_behaviour_map(rob)
    # rob.get_robot_box()
    # monitor.check_area(rob.box, world)
    # control.move(rob, monitor, world)


    # #road.plot_area()
    # world.behaviour_map.plot_area()
    # rob.plot_robot()

    # #if rob.box.intersects(road.box):
    #  #   print("True")
    

    plt.pause(dt/100)
    
    if waiting:
        plt.waitforbuttonpress()
        waiting = False