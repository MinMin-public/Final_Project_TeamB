#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from xycar import Xycar

class Parallel:
    def __init__(self):
        self.counter = 0
        self.state = None

        self.teamB = Xycar()
        self.lidar_avg=None

    def parallel_start(self):
        if self.teamB.lidar:
            self.lidar_avg=np.mean(self.teamB.lidar[-126:-48])

        if self.lidar_avg>0.9:
            return True

        else:
            return False

    def parallel_right(self):
        if self.teamB.lidar:
            self.lidar_avg=np.mean(self.teamB.lidar[-126:-48])

        if self.lidar_avg>0.4:
            return True

        else:
            return False

    def parallel_left(self):
        self.counter+=1

        if self.counter<120:
            return True

        else:
            return False


    def parallel_back(self):
        if self.teamB.ultra[3]>0.4:
            return True

        else:
            return False