#!/usr/bin/env python2
import numpy as np

class PositionController:
    def  __init__(self, K):
        self.K = K

    def set_curr_position(self, position):
        self.curr_position = position

    def get_vel_cmd(self, desired_position):
        return self.K * (desired_position - self.curr_position)
