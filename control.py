import math
import numpy as np
from turtle import pos
from basic_functions import trans
import matplotlib.pyplot as plt

class Control():
    def __init__(self, dt, H):
        self.dt = dt
        self.H = H

    def move(self, robot, monitor, world):
        self.predict(robot, monitor, world)
        #self.check_constraints()
        self.actuate(robot)

    def predict(self, robot, monitor, world):
        self.x_pred = np.zeros(self.H)
        self.y_pred = np.zeros(self.H)
        self.yaw_pred = np.zeros(self.H)
        self.x_pred[0] = robot.pos[0]
        self.y_pred[0] = robot.pos[1]
        self.yaw_pred[0] = robot.yaw

        if robot.resource == 'velocity control':    # on unicycle model
            if monitor.subtask == 'Drive forward':
                for i in range(1,self.H):
                    self.yaw_pred[i] = self.yaw_pred[i-1]
                    self.x_pred[i] = self.x_pred[i-1] + robot.velocity*math.cos(self.yaw_pred[i])*self.dt
                    self.y_pred[i] = self.y_pred[i-1] + robot.velocity*math.sin(self.yaw_pred[i])*self.dt
                    future_box = trans(np.array([self.x_pred[i], self.y_pred[i]]), self.yaw_pred[i-1], robot.length, robot.width)
                    monitor.check_area(future_box, world)         # check invariants
                    if monitor.subtask != 'Drive forward':
                        break

            if monitor.subtask == 'Correct to the left':
                pass

            if monitor.subtask == 'Correct to the right':
                pass

            
            plt.plot(self.x_pred, self.y_pred, 'b--')
                
    def actuate(self, robot):
        robot.yaw = self.yaw_pred[1]
        robot.pos[0] = self.x_pred[1]
        robot.pos[1] = self.y_pred[1]
        




                
                

