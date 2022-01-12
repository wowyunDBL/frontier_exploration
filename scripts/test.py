#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

x_list = [1.0, 0.8, 0.6, 0.2]
y_list = [0.0, 0.0, 0.0, 0.0]

class apf_drawing():
    def __init__(self):
        self.ox = [1.0, 3.0, 5.0]  # obstacle x position list [m]
        self.oy = [2.0, 2.0, 2.0]  # obstacle y position list [m]
        self.oradi = [0.3, 0.3, 0.3]  # obstacle x position list [m]
        self.waypoints_x = 7.0  # goal x position [m]
        self.waypoints_y = 2.0

        self.GOAL_DIST_THRES = 0.1
        # self.WAYPOINTS_DIST_THRES = 0.
        self.ATTRACT_DIST_THRES = 1
        self.OBSTACLE_THRES = 1
        self.ROBOT_STEP = 0.5

        self.plot_quiver()

    def vector_dist(self,a,b):
        a1,a2 = a
        b1,b2 = b
        return ((b1-a1)**2 + (b2-a2)**2)**0.5 

    def calc_attractive_potential(self,x,y):
        distance = self.vector_dist((x,y),(self.waypoints_x, self.waypoints_y))

        if distance <= self.ATTRACT_DIST_THRES:
            return np.array([self.waypoints_x, self.waypoints_y]) - np.array([x,y])
        else:
            return self.ATTRACT_DIST_THRES/distance * ( np.array([self.waypoints_x, self.waypoints_y]) - np.array([x,y]) )

    def calc_repulsive_potential(self,x,y):
        tmp = np.array([0.0,0.0])
        Q = self.OBSTACLE_THRES
        for i in range(len(self.ox)):
            D = self.vector_dist( (x,y),(self.ox[i],self.oy[i]) ) - self.oradi[i]
            if D <= self.OBSTACLE_THRES and D>0.8:
                tmp += (-1/D+1/Q)/(D**2) * ( np.array([x,y]) - np.array([self.ox[i], self.oy[i]]) ) /2
            # else:
        return tmp

    def calc_potential_field(self, x, y):
        af = self.calc_attractive_potential(x, y)
        rf = self.calc_repulsive_potential(x, y)
        # print('af: {}; rf: {}'.format(af,rf))
        
        return af+rf

    def plot_quiver(self):
        x_list = [float(x)/5 for x in range(35)]
        y_list = [float(y)/5 for y in range(15)]
        X,Y = np.meshgrid(x_list,y_list)
        U_tmp = []
        V_tmp = []
        Z = []
        for yy in y_list:
            for xx in x_list:
                pf_force_vec = self.calc_potential_field(xx, yy)
                U_tmp.append(pf_force_vec[0])
                V_tmp.append(pf_force_vec[1])
                Z.append(self.vector_dist((0,0),pf_force_vec))
        U_tmp = np.asarray(U_tmp)
        U = np.reshape(U_tmp,(len(x_list),len(y_list)))
        V_tmp = np.asarray(V_tmp)
        V = np.reshape(V_tmp,(len(x_list),len(y_list)))
        Z = np.asarray(Z)
        Z = np.reshape(Z,(len(y_list),len(x_list)))

        print(X.shape)
        print(Y.shape)
        print(Z.shape)
        
            # plt.quiver([0,6.8], [0,2.8], [0.9,-0.9], [0.27,-0.4], color='b', units='xy', scale=5)
        # plt.quiver(X, Y, U, V, color='b', units='xy', scale=5)
        # plt.scatter(self.ox,self.oy)

        # plt.title('Vector field')
    
        # # x-lim and y-lim
        # plt.ylim(-0.5, 3)
        # plt.xlim(-.5, 7.5)

            # fig = plt.figure()
            # ax = fig.gca(projection='3d')
            # ax.plot_surface(X, Y, Z, cmap='seismic')
        fig3, ax3 = plt.subplots(subplot_kw={"projection": "3d"})
        initial_cmap = cm.get_cmap('rainbow')
        # reversed_cmap=initial_cmap.reversed()
        surf = ax3.plot_surface(X, Y, Z, cmap=initial_cmap)
        plt.title('Depth tree', fontsize='15')
        plt.xlabel('X[pixel]', fontsize='15')
        plt.ylabel('Y[pixel]', fontsize='15')
        fig3.colorbar(surf, shrink=0.5, aspect=5)
        plt.show()

        
        # Show plot with grid
        # plt.grid()
        # plt.show()

# import cv

def draw_click_circle(self, event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDBLCLK:
                cv2.circle(self.raw_pgm, (x, y), 6, 255, -1)

def reduce_noise(self):
    print("manually reduce noise---")
    cv2.namedWindow('raw_pgm')
    cv2.setMouseCallback('raw_pgm', self.draw_click_circle)
    while True:
        cv2.imshow('raw_pgm', self.raw_pgm)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("break click!")
            break
    cv2.imwrite(self.file_path+'raw_modified.png', self.raw_pgm)

if __name__ == '__main__':
    # apf_drawing_handler = apf_drawing()
    img = np.zeros( (360,360) )

