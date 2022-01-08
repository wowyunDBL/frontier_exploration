#! /usr/bin/env python

'''ros utils'''
from numpy.core.numeric import Inf
import rospy

'''math tool'''
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

motion_direction = np.array([[]])

def calc_potential_field(sx, sy, positive_bds):
    minx = min( min(positive_bds[:,0]), sx)
    miny = min( min(positive_bds[:,1]), sy)
    cost_map = np.zeros((1024,1024))
    calc_attractive_potential(endpoints)

def calc_vector_dist(a,b):
    a1,a2 = a
    b1,b2 = b
    return ((b1-a1)**2 + (b2-a2)**2)**0.5 
    
GOAL_DIST_THRES = 3
def calc_attractive_potential(x,y,gx,gy):
    distance = calc_vector_dist((x,y),(gx,gy))

    if distance <= GOAL_DIST_THRES:
        return distance**2
    else:
        return distance*GOAL_DIST_THRES - GOAL_DIST_THRES**2

def calc_repulsive_potential(x,y,ox,oy,rr):

def get_motion_model():
    motion_direction = np.array([[]])
    return motion_direction
def oscillations_detection(previous_ids, ix, iy):

def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)

k_step = 0.5 # 50cm
def achieve_goal_cb(msg):
    global motion_direction
    min_grid = Inf
    min_dir = [0,0]
    for i,_ in enumerate(motion_direction):
        cost = motion_direction[i]
        if cost < min_grid:
            min_grid = cost
            min_dir = motion_direction[i]

    step_x = min_dir[0]*k_step
    step_y = min_dir[1]*k_step
    next_goal = np.array([sx+step_x, sy+step_y])

def main():
    rospy.init_node("potential_field_planning_node", anonymous=True)

    positive_bds = np.array([[0,-5],[10,-5],[10,5],[0,5]])
    calc_potential_field(positive_bds)
    sx = 0.0  # start x position [m]
    sy = 10.0  # start y positon [m]
    gx = 30.0  # goal x position [m]
    gy = 30.0  # goal y position [m]
    grid_size = 0.5  # potential grid size [m]
    robot_radius = 5.0  # robot radius [m]

    ox = [15.0, 5.0, 20.0, 25.0]  # obstacle x position list [m]
    oy = [25.0, 15.0, 26.0, 25.0]  # obstacle y position list [m]


    # path generation
    _, _ = potential_field_planning(
        sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

    if show_animation:
        plt.grid(True)
        plt.axis("equal")
    if show_animation:
        plt.show()

if __name__ == '__main__':
    main()