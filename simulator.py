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
import random
from rdflib import URIRef
## This script should be replaceble by real robots


class Simulator():
    def __init__(self):
        self.robots = {}
        self.num_robots = 0
        self.obstacles = {}
        #self.input_features = []

    def set_map(self, sit):
        self.map = Map(sit)
        
    def add_robot(self, name, l, w, vel, omega_max, start='down', task='left', color='blue'):
        robot = Robot(name, l, w, vel, omega_max, start, task, color)
        self.robots[name] = robot
        self.num_robots += 1

    def add_obstacle(self, name, pos):
        obstacle = Obstacle(name, pos)
        self.obstacles[name] = obstacle
    
    def simulate(self):
        self.map.plot_map()
        self.plot_robots()
        self.plot_obstacles()

    def plot_obstacles(self):
        for obstacle in self.obstacles.values():
            obstacle.plot()

    def plot_robots(self):
        for robot in self.robots.values():
            robot.plot()
            if not robot.name == 'AV1':
                break
            
            #free_horizon = robot.horizon
            #if robot.approaching_horizon:
            #   free_horizon.symmetric_difference(robot.horizon)
                
            if robot.horizon and robot.horizon.geom_type=='Polygon':
                plt.fill(*robot.horizon.exterior.xy, color='green', alpha=0.3)
            if robot.approaching_horizon and robot.approaching_horizon.geom_type=='Polygon':
                plt.fill(*robot.approaching_horizon.exterior.xy, color='yellow', alpha=0.7)
            if robot.obstructed_area and robot.obstructed_area.geom_type=='Polygon':
                plt.fill(*robot.obstructed_area.exterior.xy, color='red', alpha=0.7)


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

    def move_robot(self, robot, velocity, omega):
        robot.yaw += omega*dt
        robot.pos[0] += velocity*math.cos(robot.yaw)*dt
        robot.pos[1] += velocity*math.sin(robot.yaw)*dt
        robot.point = Point(robot.pos[0], robot.pos[1])
        
class Map():
    def __init__(self, traffic_situation = "two_lane-intersection2"):
        self.traffic_situation = traffic_situation
        if traffic_situation == "one-lane_intersection" :
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

        if traffic_situation == "two-lane_intersection" :
            self.map_dict =  {'0': {'type': 'middle',
                            'poly': Polygon([(l, l), (l+w, l), (l+w, l+w), (l, l+w)]), 'color': 'darkgray', 'transparency': 1},
                        '1': {'type': 'lane', 'location': 'down', 'poly': Polygon([(l+0.5*w, 0), (l+w, 0), (l+w, l), (l+0.5*w, l)]), 'color': 'lightblue', 'transparency': 1},
                        '2': {'type': 'lane', 'location': 'down', 'poly': Polygon([(l, 0), (l+0.5*w, 0), (l+0.5*w, l), (l, l)]), 'color': 'orange', 'transparency': 1},
                        '3': {'type': 'lane', 'location': 'right', 'poly': Polygon([(l+w, l+0.5*w), (2*l+w, l+0.5*w), (2*l+w, l+w), (l+w, l+w)]), 'color': 'lightblue', 'transparency': 1},
                        '4': {'type': 'lane', 'location': 'right', 'poly': Polygon([(l+w, l), (2*l+w, l), (2*l+w, l+0.5*w), (l+w, l+0.5*w)]), 'color': 'orange', 'transparency': 1},
                        '5': {'type': 'lane', 'location': 'up', 'poly': Polygon([(l, l+w), (l+0.5*w, l+w), (l+0.5*w, 2*l+w), (l, 2*l+w)]), 'color': 'lightblue', 'transparency': 1},
                        '6': {'type': 'lane', 'location': 'up', 'poly': Polygon([(l+0.5*w, l+w), (l+w, l+w), (l+w, 2*l+w), (l+0.5*w, 2*l+w)]), 'color': 'orange', 'transparency': 1},
                        '7': {'type': 'lane', 'location': 'left', 'poly': Polygon([(0, l), (l, l), (l, l+0.5*w), (0, l+0.5*w)]), 'color': 'lightblue', 'transparency': 1},
                        '8': {'type': 'lane', 'location': 'left', 'poly': Polygon([(0, l+0.5*w), (l, l+0.5*w), (l, l+w), (0, l+w)]), 'color': 'orange', 'transparency': 1},
                        
                        '9': {'type': 'boundary', 'location': 'down', 
                            'poly': LineString([(l+w, 0), (l+w, l)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '10': {'type': 'boundary', 'location': 'down', 
                            'poly': LineString([(l, 0), (l, l)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '11': {'type': 'boundary', 'location': 'right', 
                            'poly': LineString([(2*l+w, l+w), (l+w, l+w)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '12': {'type': 'boundary', 'location': 'right', 
                            'poly': LineString([(2*l+w, l), (l+w, l)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '13': {'type': 'boundary', 'location': 'up', 
                            'poly': LineString([(l, 2*l+w), (l, l+w)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '14': {'type': 'boundary', 'location': 'up', 
                            'poly': LineString([(l+w, 2*l+w), (l+w, l+w)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '15': {'type': 'boundary', 'location': 'left', 
                            'poly': LineString([(0, l), (l, l)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '16': {'type': 'boundary', 'location': 'left', 
                            'poly': LineString([(0, l+w), (l, l+w)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '17': {'type': 'centerline', 'location': 'down', 
                            'poly': LineString([(l+0.5*w, 0), (l+0.5*w, l)]), 'color': 'black', 'linestyle': 'dashed', 'transparency': 1},
                        '18': {'type': 'centerline', 'location': 'right', 
                            'poly': LineString([(l+w, l+0.5*w), (2*l+w, l+0.5*w)]), 'color': 'black', 'linestyle': 'dashed', 'transparency': 1},
                        '19': {'type': 'centerline', 'location': 'up', 
                            'poly': LineString([(l+0.5*w, l+w), (l+0.5*w, 2*l+w)]), 'color': 'black', 'linestyle': 'dashed', 'transparency': 1},
                        '20': {'type': 'centerline', 'location': 'left', 
                            'poly': LineString([(0, l+0.5*w), (l, l+0.5*w)]), 'color': 'black', 'linestyle': 'dashed', 'transparency': 1}
            }

        if traffic_situation == "two-lane_intersection2" :
            self.map_dict =  {'0': {'type': 'middle_dr', 'poly': Polygon([(l+0.5*w, l), (l+w, l), (l+w, l+0.5*w), (l+0.5*w, l+0.5*w)]), 'color': 'darkgray', 'transparency': 1},
                        '1': {'type': 'middle_ur', 'poly': Polygon([(l+0.5*w, l+0.5*w), (l+w, l+0.5*w), (l+w, l+w), (l+0.5*w, l+w)]), 'color': 'darkgray', 'transparency': 1},
                        '2': {'type': 'middle_ul', 'poly': Polygon([(l, l+0.5*w), (l+0.5*w, l+0.5*w), (l+0.5*w, l+w), (l, l+w)]), 'color': 'darkgray', 'transparency': 1},
                        '3': {'type': 'middle_dl', 'poly': Polygon([(l, l), (l+0.5*w, l), (l+0.5*w, l+0.5*w), (l, l+0.5*w)]), 'color': 'darkgray', 'transparency': 1},
                        '4': {'type': 'lane', 'location': 'down', 'poly': Polygon([(l+0.5*w, 0), (l+w, 0), (l+w, l), (l+0.5*w, l)]), 'color': 'lightblue', 'transparency': 1},
                        '5': {'type': 'lane', 'location': 'down', 'poly': Polygon([(l, 0), (l+0.5*w, 0), (l+0.5*w, l), (l, l)]), 'color': 'orange', 'transparency': 1},
                        '6': {'type': 'lane', 'location': 'right', 'poly': Polygon([(l+w, l+0.5*w), (2*l+w, l+0.5*w), (2*l+w, l+w), (l+w, l+w)]), 'color': 'lightblue', 'transparency': 1},
                        '7': {'type': 'lane', 'location': 'right', 'poly': Polygon([(l+w, l), (2*l+w, l), (2*l+w, l+0.5*w), (l+w, l+0.5*w)]), 'color': 'orange', 'transparency': 1},
                        '8': {'type': 'lane', 'location': 'up', 'poly': Polygon([(l, l+w), (l+0.5*w, l+w), (l+0.5*w, 2*l+w), (l, 2*l+w)]), 'color': 'lightblue', 'transparency': 1},
                        '9': {'type': 'lane', 'location': 'up', 'poly': Polygon([(l+0.5*w, l+w), (l+w, l+w), (l+w, 2*l+w), (l+0.5*w, 2*l+w)]), 'color': 'orange', 'transparency': 1},
                        '10': {'type': 'lane', 'location': 'left', 'poly': Polygon([(0, l), (l, l), (l, l+0.5*w), (0, l+0.5*w)]), 'color': 'lightblue', 'transparency': 1},
                        '11': {'type': 'lane', 'location': 'left', 'poly': Polygon([(0, l+0.5*w), (l, l+0.5*w), (l, l+w), (0, l+w)]), 'color': 'orange', 'transparency': 1},
                        
                        '12': {'type': 'boundary', 'location': 'down', 
                            'poly': LineString([(l+w, 0), (l+w, l)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '13': {'type': 'boundary', 'location': 'down', 
                            'poly': LineString([(l, 0), (l, l)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '14': {'type': 'boundary', 'location': 'right', 
                            'poly': LineString([(2*l+w, l+w), (l+w, l+w)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '15': {'type': 'boundary', 'location': 'right', 
                            'poly': LineString([(2*l+w, l), (l+w, l)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '16': {'type': 'boundary', 'location': 'up', 
                            'poly': LineString([(l, 2*l+w), (l, l+w)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '17': {'type': 'boundary', 'location': 'up', 
                            'poly': LineString([(l+w, 2*l+w), (l+w, l+w)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '18': {'type': 'boundary', 'location': 'left', 
                            'poly': LineString([(0, l), (l, l)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '19': {'type': 'boundary', 'location': 'left', 
                            'poly': LineString([(0, l+w), (l, l+w)]), 'color': 'red', 'linestyle': 'solid', 'transparency': 1},
                        '20': {'type': 'centerline', 'location': 'down', 
                            'poly': LineString([(l+0.5*w, 0), (l+0.5*w, l)]), 'color': 'black', 'linestyle': 'dashed', 'transparency': 1},
                        '21': {'type': 'centerline', 'location': 'right', 
                            'poly': LineString([(l+w, l+0.5*w), (2*l+w, l+0.5*w)]), 'color': 'black', 'linestyle': 'dashed', 'transparency': 1},
                        '22': {'type': 'centerline', 'location': 'up', 
                            'poly': LineString([(l+0.5*w, l+w), (l+0.5*w, 2*l+w)]), 'color': 'black', 'linestyle': 'dashed', 'transparency': 1},
                        '23': {'type': 'centerline', 'location': 'left', 
                            'poly': LineString([(0, l+0.5*w), (l, l+0.5*w)]), 'color': 'black', 'linestyle': 'dashed', 'transparency': 1}
            }

        elif traffic_situation == "road":

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
        #print(self.polygon_list)
        self.turtles = []
        self.number_of_turtles = 0

    def plot_map(self):
        for key, value in self.map_dict.items():
            if self.map_dict[key]['poly'].geom_type=='Polygon':
                plt.fill(*self.map_dict[key]['poly'].exterior.xy, color=self.map_dict[key]['color'], alpha=self.map_dict[key]['transparency'])
            elif self.map_dict[key]['poly'].geom_type=='LineString':
                plt.plot(*self.map_dict[key]['poly'].xy, color=self.map_dict[key]['color'], linestyle=self.map_dict[key]['linestyle'] , alpha=self.map_dict[key]['transparency'])

    def add_robot(self, turtle):
        self.turtles.append(turtle)
        self.number_of_turtles = len(self.turtles)

    def remove_robot(self, turtle_name):
        self.turtles = list(filter(lambda i: i.name != turtle_name, self.turtles))
    
    def print_current_areas(self):
        print(self.turtles[0].current_areas)
    

class Robot():
    def __init__(self, name, length, width, velocity_max, omega_max, start, task, color):
        self.name = name
        self.length = length
        self.width = width
        self.velocity_max = velocity_max
        self.color = color
        self.omega_max = omega_max
        self.reset(start, task)
        self.point = Point(self.pos[0], self.pos[1])
        self.uri = URIRef("http://example.com/" + self.name)
        self.horizon = None
        self.approaching_horizon = None
        self.obstructed_area = None
        

    def plot(self):
        self.poly = trans(self.pos, self.yaw, self.length, self.width)
        plt.fill(*self.poly.exterior.xy, color=self.color)

    def random_reset(self):
        start_task_list = ['down', 'right', 'up', 'left']
        start, task = random.sample(start_task_list, 2)
        self.reset(start, task)

    def reset(self, start, task):
        self.start = start
        self.task = task
        if start == 'down':
            self.pos = np.array([l+0.75*w, self.length/2])
            self.yaw = 0.5*math.pi
        elif start == 'up':
            self.pos = np.array([l+0.25*w,2*l+w-self.length/2])
            self.yaw = -0.5*math.pi
        elif start == 'left':
            self.pos = np.array([self.length/2,l+0.25*w])
            self.yaw = 0
        elif start == 'right':
            self.pos = np.array([2*l+w-self.length/2,l+0.75*w])
            self.yaw = math.pi
        self.poly = trans(self.pos, self.yaw, self.length, self.width)

class Obstacle():
    def __init__(self, name, pos):
        self.name = name
        self.uri = URIRef("http://example.com/" + self.name)
        self.pos = pos
        self.yaw = 0.3
        self.length = 4
        self.width = 4
        self.poly = trans(self.pos, self.yaw, self.length, self.width)
    
    def plot(self):
        #self.poly = trans(self.pos, self.yaw, self.length, self.width)
        plt.fill(*self.poly.exterior.xy, edgecolor='black', hatch='//', fill=False)