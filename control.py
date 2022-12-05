import math
import numpy as np
from turtle import pos
from basic_functions import trans, coordinate_transform
import matplotlib.pyplot as plt
from global_variables import H, dt

class Control():
    def __init__(self):
        pass

    def move(self, robot, monitor):
        self.predict(robot, monitor)
        #self.check_constraints()
        self.actuate(robot)

    def predict(self, robot, monitor):
        self.x_pred = np.zeros(H)
        self.y_pred = np.zeros(H)
        self.yaw_pred = np.zeros(H)
        self.x_pred[0] = robot.pos[0]
        self.y_pred[0] = robot.pos[1]
        self.yaw_pred[0] = robot.yaw

        #if robot.resource == 'velocity control':    # on unicycle model
        if monitor.subtask == 'Drive forward':
            for i in range(1,H):
                self.yaw_pred[i] = self.yaw_pred[i-1]+0.001
                self.x_pred[i] = self.x_pred[i-1] + robot.velocity*math.cos(self.yaw_pred[i])*dt
                self.y_pred[i] = self.y_pred[i-1] + robot.velocity*math.sin(self.yaw_pred[i])*dt
                robot_pred = trans(np.array([self.x_pred[i], self.y_pred[i]]), self.yaw_pred[i-1], robot.length, robot.width)
                #monitor.check_area(robot_pred, world)         # check invariants
                #if monitor.subtask != 'Drive forward':
                #    break

        if monitor.subtask == 'Correct to the left':
            pass

        if monitor.subtask == 'Correct to the right':
            pass

        pos = np.stack([self.x_pred, self.y_pred], axis=0)
        rel_pos = coordinate_transform(robot, np.transpose(pos))
        plt.plot(rel_pos[:, 0], rel_pos[:, 1], 'b--')
                
    def actuate(self, robot):
        robot.yaw = self.yaw_pred[1]
        robot.pos[0] = self.x_pred[1]
        robot.pos[1] = self.y_pred[1]
        




                
                

