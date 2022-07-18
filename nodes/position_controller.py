#!/usr/bin/env python2
from turtle import pos
import numpy as np

class PositionController:
    def  __init__(self, K):
        self.K = K

    def set_curr_position(self, position):
        self.curr_position = position

    def limit(self, max):
        for i in range(0, max.size):
            assert max[0, i] > 0
            if(self.vel[i]  > max[0, i]):
                self.vel[i] = max[0, i]
            elif(self.vel[i] < -max[0, i]):
                self.vel[i] = -max[0, i]

    def get_vel_cmd(self, desired_position):
        self.vel =  self.K * (desired_position - self.curr_position)
        max_vel = np.mat([2.0, 2.0, 2.0, 2.0])
        self.limit(max_vel)
        return self.vel


