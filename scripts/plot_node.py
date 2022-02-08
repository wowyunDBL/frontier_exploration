#!/usr/bin/wowpython

from re import L, TEMPLATE
import numpy as np
import time
import random
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
from datetime import datetime

import rospy
import sys
import time
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped, Point
from nav_msgs.msg import Odometry
# from mapping_explorer.msg import Trunkset, Trunkinfo, Correspondence

# import TREEDATA
from collections import deque

import threading


class plot_nodeHandler():
    def __init__(self):
        '''plot utils'''
        if sys.version[0]=='2':
            # self.fig, self.ax1 = plt.subplots(1, 1,dpi=70,figsize=(10,10))
            # self.fig, self.ax2 = plt.subplots(1, 1,dpi=70,figsize=(10,10))
            self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2,dpi=120,figsize=(10,10))
        elif sys.version[0]=='3':
            self.fig, ((self.ax1,self.ax2)) = plt.subplots(2, 1,dpi=120,figsize=(10,10))
        
        '''set background properties'''
        '''1. self.ax.plot()[0]'''
        '''2. self.ax.text()'''
        self.ax1.set_xlim(-30,0)
        self.ax1.set_ylim(-0.5,0.5)
        self.ax2.set_xlim(-5,5)
        self.ax2.set_ylim(-.5,8)
        self.ax3.set_xlim(-30,0)
        self.ax3.set_ylim(-0.5,8)
        self.ax4.set_xlim(-30,0)
        self.ax4.set_ylim(-0.5,5)

        self.ax1.set_xlabel("Time (from now) [s]")
        self.ax1.set_ylabel("[m/s] [deg/s]")
        self.ax2.set_xlabel("[m]")
        self.ax2.set_ylabel("[m]")
        self.ax3.set_xlabel("Time (from now) [s]")
        self.ax3.set_ylabel("[m]")
        self.ax4.set_xlabel("Time (from now) [s]")
        self.ax4.set_ylabel("[m]")

        plt.ion()

        '''ros utils'''
        cmdVel_sub=rospy.Subscriber('/cmd_vel', Twist, self.cb_cmdvel,queue_size=1)
        odom_sub=rospy.Subscriber('/odom', Odometry, self.cb_odom,queue_size=1)
        traj_sub=rospy.Subscriber('/explore/traj', Point, self.cb_traj,queue_size=1)
        potential_af_sub = rospy.Subscriber('/explore/potential/af',Point, self.cb_af,queue_size=5)
        potential_rf_sub = rospy.Subscriber('/explore/potential/rf',Point, self.cb_rf,queue_size=5)
        # RTheta_sub=rospy.Subscriber('/explore/trajRTheta', Point, self.cb_RTheta,queue_size=1)

        self.lock=threading.Lock()

        '''1. declare data'''
        self.t_data=deque([], maxlen=250)
        self.t_data_odom=deque([], maxlen=2500)
        self.t_data_traj=deque([], maxlen=250)
        self.cmd_v_data=deque([], maxlen=250)
        self.cmd_omg_data=deque([], maxlen=250)
        self.odom_x_data=deque([],maxlen=2500)
        self.odom_y_data=deque([],maxlen=2500)
        self.traj_x_data=deque([],maxlen=250)
        self.traj_y_data=deque([],maxlen=250)
        self.af_data=None
        self.rf_data=None
        self.traj_x_conti_data=deque([],maxlen=2500)
        self.traj_y_conti_data=deque([],maxlen=2500)
        # self.alpha=deque([],maxlen=250)

        '''2. declare instance for handling plot'''
        self.cmd_v_line=self.ax1.plot([],[],'-o', color='b',markersize='2', label='Linear')[0] 
        self.cmd_omg_line=self.ax1.plot([],[],'-o', color='salmon',markersize='2', label='Angular')[0]
        self.traj_dot=self.ax2.plot([], [], '-o',markersize='3', color='tab:red',label='traj')[0]
        self.odom_dot=self.ax2.plot([], [], 'o',markersize='10', color='lightblue', markeredgecolor='k')[0]
        self.odom_line=self.ax2.plot([], [], '-', color='tab:blue')[0]
        self.odom_x_line=self.ax3.plot([], [], '-o', color='k', markersize='1', label='odom_x')[0]
        self.odom_y_line=self.ax4.plot([], [], '-o', color='k', markersize='1', label='odom_y')[0] 
        self.traj_x_dot=self.ax3.plot([], [], '*',markersize='5', color='tab:red', label='ref_x')[0]
        self.traj_y_dot=self.ax4.plot([], [], '*',markersize='5', color='tab:red', label='ref_y')[0]
        self.traj_x_conti_line=self.ax3.plot([], [], '-o', color='tab:red', markersize='1', label='ref_x')[0]
        self.traj_y_conti_line=self.ax4.plot([], [], '-o', color='tab:red', markersize='1', label='ref_y')[0]
        self.af_vec=self.ax2.plot([], [], '-X',markersize='5', color='tab:red', label='af')[0]
        self.rf_vec=self.ax2.plot([], [], '-X',markersize='5', color='tab:green', label='rf')[0]
        self.rf_text=self.ax2.text(3,4.5,'',fontsize=14, color='k')

        self.ax1.legend()
        self.ax2.legend()
        self.ax3.legend()
        self.ax4.legend()
        plt.pause(0.1)

        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.lock.acquire()
            t0 = time.time()
            cmd_t0=np.asarray([x-t0 for x in self.t_data])
            np_cmd_omg_data=np.asarray(self.cmd_omg_data)
            np_cmd_v_data=np.asarray(self.cmd_v_data)
            np_odom_x_data=np.asarray(self.odom_x_data)
            np_odom_y_data=np.asarray(self.odom_y_data)
            # np_odom_x_data=np.asarray(self.odom_x_data)
            # np_odom_y_data=np.asarray(self.odom_y_data)
            np_traj_x_data=np.asarray(self.traj_x_data)
            np_traj_y_data=np.asarray(self.traj_y_data)
            np_traj_x_conti_data=np.asarray(self.traj_x_conti_data)
            np_traj_y_conti_data=np.asarray(self.traj_y_conti_data)
            odom_t0=np.asarray([x-t0 for x in self.t_data_odom])
            self.lock.release()

            '''3. set data for handling plot instance'''
            self.cmd_omg_line.set_data(cmd_t0, np_cmd_omg_data)
            self.cmd_v_line.set_data(cmd_t0, np_cmd_v_data)
            self.odom_dot.set_data(-np_odom_y_data[-1], np_odom_x_data[-1])
            self.odom_line.set_data(-np_odom_y_data, np_odom_x_data)
            self.traj_dot.set_data(-np_traj_y_data,np_traj_x_data)
            self.odom_x_line.set_data(odom_t0, np_odom_x_data)
            self.odom_y_line.set_data(odom_t0, np_odom_y_data)
            # if len(self.t_data_traj):
            traj_t0=np.asarray([x-t0 for x in self.t_data_traj])
            self.traj_x_dot.set_data(traj_t0, np_traj_x_data)
            self.traj_y_dot.set_data(traj_t0, np_traj_y_data)
            self.traj_x_conti_line.set_data(odom_t0, np_traj_x_conti_data)
            self.traj_y_conti_line.set_data(odom_t0, np_traj_y_conti_data)
                # self.rho_line.set_data(RT_t0, self.rho)
                # self.alpha_line.set_data(RT_t0, self.alpha)
            if not(self.af_data is None or self.rf_data is None):
                self.af_vec.set_data([3,3-self.af_data[1]],[5,5+self.af_data[0]])
                self.rf_vec.set_data([3,3-self.rf_data[1]],[5,5+self.rf_data[0]])
                self.rf_text.set_text('rf_vec: ['+str(round(self.rf_data[0],2))+','+str(round(self.rf_data[1],2))+']')
            plt.pause(0.005)
            loop_rate.sleep()

    def cb_cmdvel(self, msg):
        self.lock.acquire()
        self.t_data.append(time.time())
        self.cmd_v_data.append(msg.linear.x)
        self.cmd_omg_data.append(msg.angular.z)
        self.lock.release()

    def cb_odom(self, msg):
        self.lock.acquire()
        self.t_data_odom.append(time.time())
        self.odom_x_data.append(msg.pose.pose.position.x)
        self.odom_y_data.append(msg.pose.pose.position.y)
        if len(self.traj_x_data):
            self.traj_x_conti_data.append(self.traj_x_data[-1])
            self.traj_y_conti_data.append(self.traj_y_data[-1])
        else:
            self.traj_x_conti_data.append(0)
            self.traj_y_conti_data.append(0)
        self.lock.release()

    def cb_traj(self,msg):
        self.lock.acquire()
        self.t_data_traj.append(time.time())
        self.traj_x_data.append(msg.x)
        self.traj_y_data.append(msg.y)
        self.lock.release()
    
    def cb_af(self,msg):
        self.af_data = [msg.x, msg.y]
    
    def cb_rf(self,msg):
        self.rf_data = [msg.x, msg.y]

    def save_fig(self):

        if sys.version[0]=='2':
            self.fig.savefig('/home/ncslaber/110-1/' + datetime.now().strftime("%d-%m-%Y %H:%M:%S.%f")+'.png',)
        elif sys.version[0]=='3':
            self.fig.savefig('/home/ncslaber/110-1/' + datetime.now().strftime("%d-%m-%Y_%H:%M:%S.%f")+'.png',)


if __name__ == '__main__':

    print("Python version: ",sys.version)
    rospy.init_node("plot_node", anonymous=True)
    pn = plot_nodeHandler()
    
    