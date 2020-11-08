from scipy import optimize
from math import sin, cos, pi
from random import random, shuffle
import sys
import numpy as np
import time

L_a = 1 # length of arm
L_e = 0.4 # length of elbow
theta_s_min = -pi/2
theta_s_max = pi/2 # shoulder pivot
theta_a_min = 0
theta_a_max = pi/2 # arm updown
theta_e_min = 0
theta_e_max = pi # elbow updown

def build_func(des_pos):
    des_x, des_y, des_z = des_pos
    def cartesian(angle):
        """ takes in shoulder, arm, and elbow rotation and spits out end cartesian """
        theta_s, theta_a, theta_e = angle
        theta = theta_e + theta_a - pi/2
        xy_vec = L_a*cos(theta_a) + L_e*sin(theta)

        x = xy_vec*cos(theta_s)
        y = xy_vec*sin(theta_s)
        z = L_a*sin(theta_a)-L_e*cos(theta)

        return x-des_x, y-des_y, z-des_z
    return cartesian

def calc_range():
    x_min, y_min, z_min = [sys.maxsize]*3
    x_max, y_max, z_max = [-sys.maxsize]*3
    point_list = []
    func = build_func([0, 0, 0]) # passing 0 equals the cartesian coord
    for i in np.linspace(theta_s_min, theta_s_max, 180):
        for j in np.linspace(theta_a_min, theta_a_max, 90):
            for k in np.linspace(theta_e_min, theta_e_max, 180):
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

if __name__ == "__main__":
    legal_points = calc_range()
    shuffle(legal_points)
    init = [(theta_s_min+theta_s_max)/2,  (theta_a_min+theta_a_max)/2, (theta_e_min+theta_e_max)/2] # init angles
    count = 0
    iter = 10000
    start_time = time.time()
    for i, pt in enumerate(legal_points[:iter]): # num of queries
        des_pt = pt[1]
        func = build_func(des_pt) # final cartesian
        root = optimize.root(func, init, method="hybr")
        if not root.success:
            count+=1
            print(i, des_pt, root.x, pt[0], "\n")
            continue
        init = root.x
    print(count/iter*100, "percent error rate", time.time()-start_time)