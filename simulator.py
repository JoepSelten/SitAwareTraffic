from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box, MultiPolygon
from shapely import affinity
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import json
from traffic_areas import *
from basic_functions import *
from global_variables import *
## This script should be replaceble by real robots


class Simulator():
    def __init__(self, sit="intersection"):
        self.robots = []
        self.num_robots = 0
        #self.input_features = []

    def set_map(self, sit):
        self.map = Map(sit)
        
    def add_robot(self, turtle_name, l, w, vel, omega_max, start='down'):
        robot = Robot(turtle_name, l, w, vel, omega_max, start)
        self.robots.append(robot)
        self.num_robots += 1

    def simulate(self):
        self.map.plot_map()
        self.plot_robots()
        
    def plot_robots(self):
        for robot in self.robots:
            robot.plot_robot()

    def simulate_relative_obj(self, obj_pos, l, b):     
        rel_pos = obj_pos - self.robot.pos
        yaw = -0.5*math.pi
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                        [-math.sin(yaw), math.cos(yaw)]])

        rel_pos_rot = rel_pos.dot(Rot1)
        rel_yaw = 0.5*math.pi-self.robot.yaw
        
        rel_object = trans(np.array([0,0]), 0.5*math.pi+rel_yaw, l, b, rel_pos_rot)
        plt.xlim([-20, 20])
        plt.ylim([-20, 20])
        plt.fill(*rel_object.exterior.xy)

        return rel_object

    def perceived_features(self):   # gives all features inside a certain radius, maybe later use a part of the circle
        robot = self.robots[0]
        perception_radius = PERCEPTION_RADIUS
        p = Point(robot.pos)
        perception_area = p.buffer(perception_radius)
        self.input_features = []
        
        #map_features = MultiPolygon(self.map.polygon_list)
        for polygon in self.map.polygon_list:
            #if polygon.geom_type=='LineString':
            overlap = polygon.intersection(perception_area)
            if overlap:
                overlap_rel = coordinate_transform_abs_to_rel(robot, overlap)
                self.input_features.append(overlap_rel)            
        return self.input_features

    def plot_input_features(self):
        for feature in self.input_features:
            if feature.geom_type=='Polygon':
                plt.fill(*feature.exterior.xy, color='gray')
            elif feature.geom_type=='LineString':
                plt.plot(*feature.xy, 'black')

    def move_robot(self, robot, omega):
        #robot = self.robots[0]
        robot.yaw += omega*dt
        robot.pos[0] += robot.velocity*math.cos(robot.yaw)*dt
        robot.pos[1] += robot.velocity*math.sin(robot.yaw)*dt
        
class Map():
    def __init__(self, Traffic_Situation = "intersection"):
        if Traffic_Situation == "intersection" :
            self.map_dict =  {'0': {'type': 'on intersection',
                            'poly': Polygon([(l, l), (l+w, l), (l+w, l+w), (l, l+w)]), 'color': 'lightblue', 'transparency': 1},
                        '1': {'type': 'lane', 'location': 'down', 'poly': Polygon([(l, 0), (l+w, 0), (l+w, l), (l, l)]), 'color': 'lightblue', 'transparency': 1},
                        '2': {'type': 'lane', 'location': 'up', 'poly': Polygon([(l, l+w), (l+w, l+w), (l+w, 2*l+w), (l, 2*l+w)]), 'color': 'lightblue', 'transparency': 1},
                        '3': {'type': 'lane', 'location': 'left', 'poly': Polygon([(0, l), (l, l), (l, l+w), (0, l+w)]), 'color': 'lightblue', 'transparency': 1},
                        '4': {'type': 'lane', 'location': 'right', 'poly': Polygon([(l+w, l), (2*l+w, l), (2*l+w, l+w), (l+w, l+w)]), 'color': 'lightblue', 'transparency': 1},
                        '5': {'type': 'boundary', 'location': 'down', 
                            'poly': LineString([(l, 0), (l, l)]), 'color': 'red', 'transparency': 1},
                        '6': {'type': 'boundary', 'location': 'down', 
                            'poly': LineString([(l+w, 0), (l+w, l)]), 'color': 'red', 'transparency': 1},
                        '7': {'type': 'boundary', 'location': 'up', 
                            'poly': LineString([(l, l+w), (l, 2*l+w)]), 'color': 'red', 'transparency': 1},
                        '8': {'type': 'boundary', 'location': 'up', 
                            'poly': LineString([(l+w, l+w), (l+w, 2*l+w)]), 'color': 'red', 'transparency': 1},
                        '9': {'type': 'boundary', 'location': 'left', 
                            'poly': LineString([(0, l), (l, l)]), 'color': 'red', 'transparency': 1},
                        '10': {'type': 'boundary', 'location': 'left', 
                            'poly': LineString([(0, l+w), (l, l+w)]), 'color': 'red', 'transparency': 1},
                        '11': {'type': 'boundary', 'location': 'right', 
                            'poly': LineString([(l+w, l), (2*l+w, l)]), 'color': 'red', 'transparency': 1},
                        '12': {'type': 'boundary', 'location': 'right', 
                            'poly': LineString([(l+w, l+w), (2*l+w, l+w)]), 'color': 'red', 'transparency': 1},
            }

        if Traffic_Situation == "intersection2" :
            self.map_dict =  {'0': {'type': 'on intersection',
                            'geometry': np.array([l, l+w, l+w, l, l, l, l+w, l+w]), 'color': 'lightblue', 'transparency': 1},
                        '1': {'type': 'lane', 'location': 'down', 'geometry': np.array([l, l+w, l+w, l, 0, 0, l, l]), 'color': 'lightblue', 'transparency': 1},
                        '2': {'type': 'lane', 'location': 'up', 'geometry': np.array([l, l+w, l+w, l, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'lightblue', 'transparency': 1},
                        '3': {'type': 'lane', 'location': 'left', 'geometry': np.array([0, l, l, 0, l, l, l+w, l+w]), 'color': 'lightblue', 'transparency': 1},
                        '4': {'type': 'lane', 'location': 'right', 'geometry': np.array([l+w, 2*l+w, 2*l+w, l+w, l, l, l+w, l+w]), 'color': 'lightblue', 'transparency': 1},
                        '5': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l-0.5*b, l+0.5*b, l+0.5*b, l-0.5*b, 0, 0, l, l]), 'color': 'red', 'transparency': 1},
                        '6': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l+w-0.5*b, l+w+0.5*b, l+w+0.5*b, l+w-0.5*b, 0, 0, l, l]), 'color': 'red', 'transparency': 1},
                        '7': {'type': 'boundary', 'location': 'up', 
                            'geometry': np.array([l-0.5*b, l+0.5*b, l+0.5*b, l-0.5*b, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'red', 'transparency': 1},
                        '8': {'type': 'boundary', 'location': 'up', 
                            'geometry': np.array([l+w-0.5*b, l+w+0.5*b, l+w+0.5*b, l+w-0.5*b, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'red', 'transparency': 1},
                        '9': {'type': 'boundary', 'location': 'left', 
                            'geometry': np.array([0, l, l, 0, l-0.5*b, l-0.5*b, l+0.5*b, l+0.5*b]), 'color': 'red', 'transparency': 1},
                        '10': {'type': 'boundary', 'location': 'left', 
                            'geometry': np.array([0, l, l, 0, l+w-0.5*b, l+w-0.5*b, l+w+0.5*b, l+w+0.5*b]), 'color': 'red', 'transparency': 1},
                        '11': {'type': 'boundary', 'location': 'right', 
                            'geometry': np.array([l+w, 2*l+w, 2*l+w, l+w, l-0.5*b, l-0.5*b, l+0.5*b, l+0.5*b]), 'color': 'red', 'transparency': 1},
                        '12': {'type': 'boundary', 'location': 'right', 
                            'geometry': np.array([l+w, 2*l+w, 2*l+w, l+w, l+w-0.5*b, l+w-0.5*b, l+w+0.5*b, l+w+0.5*b]), 'color': 'red', 'transparency': 1},
            }

        elif Traffic_Situation == "road":

            self.map_dict =  {'0': {'type': 'lane', 'location': 'down', 'direction': [0, -1],
                            'geometry': np.array([l, l+w, l+w, l, 0, 0, 2*l, 2*l]), 'color': 'lightblue', 'transparency': 1},
                        '1': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l-0.5*b, l+0.5*b, l+0.5*b, l-0.5*b, 0, 0, 2*l, 2*l]), 'color': 'red', 'transparency': 1},
                        '2': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l+w-0.5*b, l+w+0.5*b, l+w+0.5*b, l+w-0.5*b, 0, 0, 2*l, 2*l]), 'color': 'red', 'transparency': 1},
            }

            
        self.polygon_list = []
        for key, value in self.map_dict.items():
            #self.map_dict[key]['poly'] = convert_nparray_to_polygon(self.map_dict[key]['geometry'])
            #print(self.map_dict[key]['geometry'])
            #print(self.map_dict[key]['poly'])
            self.polygon_list.append(self.map_dict[key]['poly'])

        self.turtles = []
        self.number_of_turtles = 0

    def plot_map(self):
        for key, value in self.map_dict.items():
            if self.map_dict[key]['poly'].geom_type=='Polygon':
                plt.fill(*self.map_dict[key]['poly'].exterior.xy, color=self.map_dict[key]['color'], alpha=self.map_dict[key]['transparency'])
            elif self.map_dict[key]['poly'].geom_type=='LineString':
                plt.plot(*self.map_dict[key]['poly'].xy, color=self.map_dict[key]['color'], alpha=self.map_dict[key]['transparency'])

    def add_robot(self, turtle):
        self.turtles.append(turtle)
        self.number_of_turtles = len(self.turtles)

    def remove_robot(self, turtle_name):
        self.turtles = list(filter(lambda i: i.name != turtle_name, self.turtles))
    
    def print_current_areas(self):
        print(self.turtles[0].current_areas)
    

class Robot():
    def __init__(self, name, length, width, vel, omega_max, start, color='orange'):
        self.name = name
        self.length = length
        self.width = width
        self.velocity = vel
        self.start = start
        self.color = color
        self.omega_max = omega_max
        
        if start == 'down':
            self.pos = np.array([l+0.5*w, self.length/2])
            self.yaw = 0.5*math.pi
            self.lane_id_start = '2'
        elif start == 'up':
            self.pos = np.array([l+0.5*w,2*l+w-self.length/2])
            self.yaw = -0.5*math.pi
            self.lane_id_start = '3'
        elif start == 'left':
            self.pos = np.array([self.length/2,l+0.5*w])
            self.yaw = 0
            self.lane_id_start = '5'
        elif start == 'right':
            self.pos = np.array([2*l+w-self.length/2,l+0.5*w])
            self.yaw = math.pi
            self.lane_id_start = '8'
        self.box = trans(self.pos, self.yaw, self.length, self.width)
        self.rel_box = trans(np.array([0,0]), 0.5*math.pi, self.length, self.width)
        #print(self.yaw)

    def plot_robot(self):
        self.box = trans(self.pos, self.yaw, self.length, self.width)
        plt.fill(*self.box.exterior.xy, color=self.color)

    def plot_rel_robot(self):
        plt.fill(*self.rel_box.exterior.xy, color=self.color)