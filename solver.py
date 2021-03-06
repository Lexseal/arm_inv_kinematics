from scipy import optimize, spatial
from math import sin, cos, pi
from random import random, shuffle
import os.path
import sys
import numpy as np
import time

class Inv_kin():
    def __init__(self):
        self.L_a = 0.409575 # length of arm
        self.L_e = 0.4699 # length of elbow
        self.theta_s_min = -1
        self.theta_s_max = 1 # shoulder pivot
        self.theta_a_min = 0.4
        self.theta_a_max = 2 # arm updown
        self.theta_e_min = 0
        self.theta_e_max = 2.35 # elbow updown
        self.state = [(self.theta_s_min+self.theta_s_max)/2,
                     (self.theta_a_min+self.theta_a_max)/2, 
                     (self.theta_e_min+self.theta_e_max)/2] # initial state
        self.bound = optimize.Bounds([self.theta_s_min, self.theta_a_min, self.theta_e_min], 
                                     [self.theta_s_max, self.theta_a_max, self.theta_e_max])
        self.lookup_table = self.load_table()
        self.kd_tree = self.build_kd_tree()
        
    def build_func(self, des_pos, abs_val=True, norm=False):
        des_x, des_y, des_z = des_pos
        def cartesian(angle):
            """ takes in shoulder, arm, and elbow rotation and spits out end cartesian """
            theta_s, theta_a, theta_e = angle
            theta = pi/2 + theta_a - theta_e
            xy_vec = self.L_a*cos(theta_a) + self.L_e*sin(theta)

            x = xy_vec*cos(theta_s)
            y = xy_vec*sin(theta_s)
            z = self.L_a*sin(theta_a)-self.L_e*cos(theta)

            if norm: return np.linalg.norm([x-des_x, y-des_y, z-des_z])
            elif abs_val: return abs(x-des_x), abs(y-des_y), abs(z-des_z)
            else: return x-des_x, y-des_y, z-des_z
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
                    x_min = min(x_min, pt[0])
                    x_max = max(x_max, pt[0])
                    y_min = min(y_min, pt[1])
                    y_max = max(y_max, pt[1])
                    z_min = min(z_min, pt[2])
                    z_max = max(z_max, pt[2])
                    point_list.append([*pt, i, j, k])
        print(x_min, x_max)
        print(y_min, y_max)
        print(z_min, z_max)
        
        np.save("lookup_table.npy", np.array(point_list)) # save the table
        return point_list

    def load_table(self):
        if os.path.exists("lookup_table.npy"): # table already exists
            # point list is of the form [x, y, z, swivel, updown, elbow]
            return list(np.load("lookup_table.npy"))
        return self.calc_range()

    def build_kd_tree(self):
        return spatial.KDTree(np.array(self.lookup_table)[:, :3])

    def within_limit(self, solution):
        for i, angle in enumerate(solution):
            if angle < self.bound.lb[i] or angle > self.bound.ub[i]:
                return False
        return True

    def solve(self, des_pt, init=None):
        if init==None: init = self.state
        func = self.build_func(des_pt)
        root = optimize.root(func, init, method="hybr")
        if root.success and self.within_limit(root.x):
            #self.state = root.x # if succeeds
            return root.x
        print(root.x)
        return None # fails

    def minimize(self, des_pt, init=None):
        if init==None: init = self.state
        func = self.build_func(des_pt, norm=True)
        root = optimize.minimize(func, init, bounds=self.bound)
        if not root.success: return None
        self.state = root.x # if succeeds
        return root.x

    def look_up_sol(self, des_pt, init=None):
        if init==None: init = self.state
        root_idx = self.kd_tree.query_ball_point(des_pt, 0.01)
        root = None
        lowest_diff = sys.maxsize
        for i in root_idx:
            matching_state = np.array(self.lookup_table[i][3:]) # angles
            diff = np.linalg.norm(matching_state-np.array(self.state))
            if diff < lowest_diff:
                lowest_diff = diff
                root = self.lookup_table[i][3:]
        return root

    def benchmark(self, method):
        legal_points = self.lookup_table.copy()
        shuffle(legal_points)
        count = 0
        iter = 10000
        start_time = time.time()
        for i, pt in enumerate(legal_points[:iter]): # num of queries
            des_pt = pt[:3]
            root = method(des_pt)
            if root is None:
                count+=1
                print(i, des_pt) # print out the failed point
                continue
        print(count/iter*100, "% error rate", time.time()-start_time)

if __name__ == "__main__":
    inv_kin = Inv_kin()
    if len(sys.argv) != 4: inv_kin.benchmark(inv_kin.look_up_sol)
    else:
        des_pt = list(map(float, sys.argv[1:4]))
        angles = inv_kin.minimize(des_pt)
        print(angles)
        angles = inv_kin.look_up_sol(des_pt)
        print(angles)
        #print(inv_kin.state)