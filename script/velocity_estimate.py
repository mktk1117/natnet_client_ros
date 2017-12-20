#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
velocity_estimate.py

Copyright (c) 2017 Takahiro Miki

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
'''

import numpy as np


class VelocityEstimate:
    def __init__(self, n):
        ''' Velocity Estimation'''
        # self.time_range = time_range
        self.time_list = np.zeros(n)
        self.x_list = np.zeros(n)
        self.y_list = np.zeros(n)
        self.z_list = np.zeros(n)
        self.prev_v = np.zeros(3)

    def queue_array(self, src, x):
        dst = np.roll(src, -1)
        dst[-1] = x
        return dst

    def get_velocity(self, t, x):
        self.time_list = self.queue_array(self.time_list, t)
        self.x_list = self.queue_array(self.x_list, x[0])
        self.y_list = self.queue_array(self.y_list, x[1])
        self.z_list = self.queue_array(self.z_list, x[2])
        vx = self.estimate_velocity(self.time_list, self.x_list, 0)
        vy = self.estimate_velocity(self.time_list, self.y_list, 1)
        vz = self.estimate_velocity(self.time_list, self.z_list, 2)
        return np.array([vx, vy, vz])

    def estimate_velocity(self, t_list, pose_list, i):
        time_list = t_list - t_list[0]
        n = len(time_list)
        st = np.sum(time_list)
        st2 = np.sum(time_list**2)
        st3 = np.sum(time_list**3)
        st4 = np.sum(time_list**4)
        sx = np.sum(pose_list)
        sxt = np.sum(pose_list * time_list)
        sxt2 = np.sum(pose_list * time_list**2)
        A = np.array([[st2, st, n],
                      [st3, st2, st],
                      [st4, st3, st2]])
        b = np.array([sx, sxt, sxt2])
        try:
            v = np.linalg.solve(A, b)
            a = v[0] * 2
            v0 = v[1]
            vel = v0 + a * time_list[-1]
            if np.fabs(self.prev_v[i] - vel) < 1.0:
                self.prev_v[i] = vel
            return self.prev_v[i]
        except:
            return self.prev_v[i]

#
# if __name__ == '__main__':
#     main(sys.argv)
