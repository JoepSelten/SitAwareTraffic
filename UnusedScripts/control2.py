import math
import numpy as np
from turtle import pos
from basic_functions import trans, coordinate_transform
import matplotlib.pyplot as plt
from global_variables import H, dt

class Control():
    def __init__(self):
        pass

    def turn_and_drive(self, phi_des):
        pass

    def move(self, world, simulator):
        goal = world.get_relative_subgoal() # or should i use absolute control
        print(f'goal: {goal}')
        self.predict(world, goal)
        #self.check_constraints()
        self.actuate(simulator, world)

    def predict(self, world, goal):
        goal_angle = math.atan2(goal.coords[0][1]-goal.coords[1][1], goal.coords[0][0]-goal.coords[1][0])

        self.x_pred = np.zeros(H)
        self.y_pred = np.zeros(H)
        self.yaw_pred = np.zeros(H)
        # self.x_pred[0] = robot.pos[0]
        # self.y_pred[0] = robot.pos[1]
        # self.yaw_pred[0] = robot.yaw
        self.x_pred[0] = 0
        self.y_pred[0] = 0
        self.yaw_pred[0] = 0.5*math.pi

        
        #if robot.resource == 'velocity control':    # on unicycle model

        #print(goal_angle % math.pi)
        if goal_angle % math.pi < 0.03:
            self.omega = 0
        else:
            self.omega = world.robot.omega_max
            #self.omega = 0

        for i in range(1,H):
            self.yaw_pred[i] = self.yaw_pred[i-1] + self.omega*dt
            #self.yaw_pred[i] = self.yaw_pred[i-1]
            self.x_pred[i] = self.x_pred[i-1] + world.robot.velocity*math.cos(self.yaw_pred[i])*dt
            self.y_pred[i] = self.y_pred[i-1] + world.robot.velocity*math.sin(self.yaw_pred[i])*dt
                        

        plt.plot(self.x_pred, self.y_pred, color='black', linestyle='dashed')
                
    def actuate(self, simulator, world):
        simulator.move_robot(world.robot, self.omega)
        world.update_pos()     # dit is eigenlijk de odometry



                
                

