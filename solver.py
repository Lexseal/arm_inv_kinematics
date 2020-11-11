from scipy import optimize
from math import sin, cos, pi
from random import random, shuffle
import sys
import numpy as np
import time

class Inv_kin():
    def __init__(self):
        self.L_a = 0.409575 # length of arm
        self.L_e = 0.4699 # length of elbow
        self.theta_s_min = -0.78
        self.theta_s_max = 0.78 # shoulder pivot
        self.theta_a_min = 0.4
        self.theta_a_max = 2 # arm updown
        self.theta_e_min = -0.78
        self.theta_e_max = 2.35 # elbow updown
        self.state = [(self.theta_s_min+self.theta_s_max)/2,
                     (self.theta_a_min+self.theta_a_max)/2, 
                     (self.theta_e_min+self.theta_e_max)/2]

    def build_func(self, des_pos, abs_val=True):
        des_x, des_y, des_z = des_pos
        def cartesian(angle):
            """ takes in shoulder, arm, and elbow rotation and spits out end cartesian """
            theta_s, theta_a, theta_e = angle
            theta = pi/2 + theta_a - theta_e
            xy_vec = self.L_a*cos(theta_a) + self.L_e*sin(theta)

            x = xy_vec*cos(theta_s)
            y = xy_vec*sin(theta_s)
            z = self.L_a*sin(theta_a)-self.L_e*cos(theta)

            if abs_val: return abs(x-des_x), abs(y-des_y), abs(z-des_z)
            return x-des_x, y-des_y, z-des_z
        return cartesian

    def calc_range(self):
        x_min, y_min, z_min = [sys.maxsize]*3
        x_max, y_max, z_max = [-sys.maxsize]*3
        point_list = []
        func = self.build_func([0, 0, 0], abs_val=False) # passing 0 equals the cartesian coord
        for i in np.linspace(self.theta_s_min, self.theta_s_max, 180):
            for j in np.linspace(self.theta_a_min, self.theta_a_max, 90):
                for k in np.linspace(self.theta_e_min, self.theta_e_max, 180):
                    pt = func([i, j, k])
                    point_list.append([[i, j, k], pt])
                    if (pt[0] < x_min): x_min = pt[0]
                    if (pt[0] > x_max): x_max = pt[0]
                    if (pt[1] < y_min): y_min = pt[1]
                    if (pt[1] > y_max): y_max = pt[1]
                    if (pt[2] < z_min): z_min = pt[2]
                    if (pt[2] > z_max): z_max = pt[2]
        print(x_min, x_max)
        print(y_min, y_max)
        print(z_min, z_max)
        return point_list

    def solve(self, des_pt, init=None):
        if init==None: init = self.state
        func = self.build_func(des_pt)
        root = optimize.root(func, init, method="hybr")
        if not root.success: return False
        self.state = root.x
        return root.x

    def benchmark(self):
        legal_points = self.calc_range()
        shuffle(legal_points)
        init = [(self.theta_s_min+self.theta_s_max)/2,  (self.theta_a_min+self.theta_a_max)/2, (self.theta_e_min+self.theta_e_max)/2] # init angles
        count = 0
        iter = 10000
        start_time = time.time()
        for i, pt in enumerate(legal_points[:iter]): # num of queries
            des_pt = pt[1]
            func = self.build_func(des_pt) # final cartesian
            root = optimize.root(func, init, method="hybr")
            if not root.success:
                count+=1
                print(i, des_pt, root.x, pt[0], "\n")
                continue
            init = root.x
        print(count/iter*100, "percent error rate", time.time()-start_time)

if __name__ == "__main__":
    inv_kin = Inv_kin()
    if len(sys.argv) != 4: inv_kin.benchmark()
    else:
        des_pt = list(map(float, sys.argv[1:4]))
        angles = inv_kin.solve(des_pt)
        print(angles)
        print(inv_kin.state)