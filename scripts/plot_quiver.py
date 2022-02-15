#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

x_list = [1.0, 0.8, 0.6, 0.2]
y_list = [0.0, 0.0, 0.0, 0.0]

class apf_drawing():
    def __init__(self):
        # self.ox = [1.0, 3.0, 5.0]  # obstacle x position list [m]
        # self.oy = [2.0, 2.0, 2.0]  # obstacle y position list [m]
        self.ox = [7.0]  # obstacle x position list [m]
        self.oy = [2.0]  # obstacle y position list [m]
        self.oradi = [0.3]  # obstacle x position list [m]
        self.waypoints_x = 0.0  # goal x position [m]
        self.waypoints_y = 0.0

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
            if D <= self.OBSTACLE_THRES:# and D>0.8:
                # tmp += (-1/D+1/Q)/(D**2) * ( np.array([self.ox[i], self.oy[i]]) - np.array([x,y])) /2 * 0.5
                tmp += (-1/D+1/Q)/(D**2) * ( -np.array([self.ox[i], self.oy[i]]) + np.array([x,y])) /2 * 0.5
            # else:
            #     tmp += np.array([0.0,0.0])
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
        for j,yy in enumerate(y_list):
            for i,xx in enumerate(x_list):
                # print(xx)
                # print(X[xx][yy])
                goal_dist = np.sqrt(X[j][i]**2 + Y[j][i]**2)
                obs_dist = np.sqrt((X[j][i]-7)**2 + (Y[j][i]-2)**2)
                if goal_dist > 2 and obs_dist > 1:
                    pf_force_vec = self.calc_potential_field(xx, yy)
                    U_tmp.append(pf_force_vec[0])
                    V_tmp.append(pf_force_vec[1])
                    Z.append(self.vector_dist((0,0),pf_force_vec))
                else:
                    U_tmp.append(0)
                    V_tmp.append(0)
                    Z.append(0)
        U_tmp = np.asarray(U_tmp)
        U = np.reshape(U_tmp,(len(x_list),len(y_list)))
        V_tmp = np.asarray(V_tmp)
        V = np.reshape(V_tmp,(len(x_list),len(y_list)))
        Z = np.asarray(Z)
        Z = np.reshape(Z,(len(y_list),len(x_list)))
        
        fig1, ax1 = plt.subplots()
        plt.quiver(X, Y, U, V, color='b', units='xy', scale=5)
        plt.scatter(self.ox,self.oy, s=100)
        plt.scatter(0,0,c='r', s=100)

        plt.title('Vector field')
    
        # x-lim and y-lim
        plt.ylim(-0.5, 3)
        plt.xlim(-.5, 7.5)
        # Show plot with grid
        # plt.grid()
        plt.axis('equal')
        plt.show()

        fig3, ax3 = plt.subplots(subplot_kw={"projection": "3d"})
        initial_cmap = cm.get_cmap('rainbow')
        # reversed_cmap=initial_cmap.reversed()
        surf = ax3.plot_surface(X, Y, Z, cmap=initial_cmap)
        plt.title('Depth tree', fontsize='15')
        plt.xlabel('X[pixel]', fontsize='15')
        plt.ylabel('Y[pixel]', fontsize='15')
        fig3.colorbar(surf, shrink=0.5, aspect=5)
        plt.show()

        



if __name__ == '__main__':
    apf_drawing_handler = apf_drawing()
    # img = np.zeros( (360,360) )

