import math
import numpy as np
from turtle import pos
from basic_functions import trans, coordinate_transform
import matplotlib.pyplot as plt
from global_variables import H, dt

class Control():
    def __init__(self):
        pass

    def execute_skill(self, world):
        skill = world.skill
        if skill == 'move_in_lane':
            self.move_in_lane(world)
        elif skill == 'turn':
            self.turn(world)
        else:
            self.stop()

    def move_in_lane(self, world):
        prev_pos = world.robot.pos[0]
        self.velocity = world.robot.velocity
        self.omega = 0

    def turn(self, world):
        self.velocity = world.robot.velocity
        self.omega = 0.5

    def stop(self):
        pass


    def actuate(self, simulator):
        AV = simulator.robots[0]
        simulator.move_robot(AV, self.velocity, self.omega)
        # prev_yaw = AV.yaw
        # prev_pos = AV.pos

        # self.yaw = prev_yaw + self.omega*dt
        # self.pos[0] = x_prev + self.velocity*math.cos(self.yaw)*dt
        # self.pos[1] = y_prev + self.velocity*math.sin(self.yaw)*dt
        # simulator.robots[0].pos = self.pos




                
                

