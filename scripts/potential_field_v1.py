#! /usr/bin/env python

'''ros utils'''
import rospy
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, CameraInfo
import tf
from mapping_explorer.msg import Trunkset, Trunkinfo

'''math tool'''
from numpy.core.numeric import Inf
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance as dist

from centroidtracker_mine import CentroidTracker

from enum import Enum
import time
import json

class apf():
    def __init__(self):
        
        '''controlller parameter: angKp > linKp'''
        self.angKp = 0.7
        self.linKp = 0.6
        self.angVel_bound = 0.3 # 0.5*57=23 degree
        self.linVel_bound = 0.3
        self.GOAL_DIST_THRES = 0.2
        self.ATTRACT_DIST_THRES = 1
        self.OBSTACLE_THRES = 1.5 #0.8
        self.ROBOT_STEP = 0.3

        self.trunkTrackDist = 3 # track trunk location only when dist < self.trunkTrackDist
        self.CentroidTracker = CentroidTracker()
        
        self.MotionSM = Enum('MotionSM', 
                                'exploration moveToObs stabilizeSensing finish')
        self.current_mode = self.MotionSM.exploration.value 
        self.visited_id_dict = {}
        self.visited_id = None
        self.startSTOPtime = None

        '''zigzag path '''
        # self.ox = [1.0, 3.0, 5.0]  # obstacle x position list [m]
        # self.oy = [2.0, 2.0, 2.0]  # obstacle y position list [m]
        self.oradi = [0.25, 0.25, 0.25]  # obstacle x position list [m]
        self.ox = []
        self.oy = [] 
        self.oradi = []
        # self.waypoints_x_list = [0.0, 7.0, 7.0, 0.0]  # goal x position [m]
        # self.waypoints_y_list = [1.0, 1.0, 3.0, 3.0]  # goal y position [m]
        self.waypoints_x_list = [11.0, 35.0, 35.0, 11.0, 11.0, 35.0, 35.0, 11.0, 11.0, 35.0, 35.0, 11.0, 11.0, 35.0, 35.0, 11.0, 11.0, 35.0, 35.0, 11.0]  
        self.waypoints_y_list = [12.0, 12.0, 14.0, 14.0, 16.0, 16.0, 18.0, 18.0, 20.0, 20.0, 22.0, 22.0, 24.0, 24.0, 26.0, 26.0, 28.0, 28.0, 30.0, 30.0]
        self.wayp_index = 0
        self.waypoints_x = self.waypoints_x_list[self.wayp_index]  # goal x position [m]
        self.waypoints_y = self.waypoints_y_list[self.wayp_index]  # goal y position [m]
        self.positive_bds = np.array([[0,-5],[20,-5],[20,5],[0,5]])
        
        self.ox_tmp = []
        self.oy_tmp = []
        self.index = 0

        '''ros pub/sub'''
        self.velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.goalPub = rospy.Publisher('/explore/traj',Point,queue_size=5) #in the following variabl: traj=goal
        self.potentialPub = rospy.Publisher('/explore/potential',Point,queue_size=5)
        self.potential_af_Pub = rospy.Publisher('/explore/potential/af',Point,queue_size=5)
        self.potential_rf_Pub = rospy.Publisher('/explore/potential/rf',Point,queue_size=5)
        self.trajRThetaPub = rospy.Publisher('/explore/trajRTheta',Point,queue_size=5)
        self.statePub = rospy.Publisher('/explore/state',UInt8,queue_size=5)
        self.odomSub = rospy.Subscriber("/odom",Odometry, self.cbOdom,buff_size=2**20,queue_size=1)
        self.trunkInfo = rospy.Subscriber("/tree/trunk_info", Trunkset, self.cbTrunk,buff_size=2**20,queue_size=1)
        robotPose = rospy.wait_for_message('/odom',Odometry)
        self.init_x = robotPose.pose.pose.position.x
        self.init_y = robotPose.pose.pose.position.y
        quaternion = (robotPose.pose.pose.orientation.x, 
                        robotPose.pose.pose.orientation.y,
                        robotPose.pose.pose.orientation.z,
                        robotPose.pose.pose.orientation.w)
        _,_,self.init_theta = tf.transformations.euler_from_quaternion(quaternion)
        self.utm_trunk = None
        self.traj_x = None
        self.traj_y = None
        self.is_triggered = False
        rospy.set_param('/apf/trigger', False)
        
        
        '''start execution'''
        rospy.loginfo('generate first goal')
        self.generate_next_goal()
        rospy.loginfo('Enter while detecting trigger_param...')
        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.is_triggered = rospy.get_param('/apf/trigger')
            if self.is_triggered == True:
                if not (self.wayp_index<len(self.waypoints_x_list)):
                    self.current_mode = self.MotionSM.finish.value
                self.fnControlNode()
            else:
                self.fnStop()
            loop_rate.sleep()

        rospy.on_shutdown(self.fnShutDown)

    def fnControlNode(self):
        msg_state = UInt8()
        msg_state.data = self.current_mode
        self.statePub.publish(msg_state)
        if self.current_mode == self.MotionSM.exploration.value:
            x_diff, y_diff = self.traj_x - self.robotPoseX, self.traj_y - self.robotPoseY
            rho = np.hypot(x_diff, y_diff)
            goal_arc = np.arctan2(y_diff, x_diff)
            alpha = (goal_arc - self.robotPoseTheta + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
            

            x_diff_wp, y_diff_wp = self.waypoints_x - self.robotPoseX, self.waypoints_y - self.robotPoseY
            rho_wp = np.hypot(x_diff_wp, y_diff_wp)
            tmp_wp_vec = np.array([x_diff_wp, y_diff_wp])  
            tmp_tj_vec = np.array([x_diff, y_diff])
            dot = tmp_wp_vec.dot(tmp_tj_vec)
            ang_btw = min((dot/rho_wp/rho),1) # -1 < cos < 1
            if np.hypot(x_diff_wp, y_diff_wp) < self.GOAL_DIST_THRES:
                self.fnStop()
                print('control: achieve ONE waypoint, fnStop!')
                self.wayp_index += 1
                if self.wayp_index<len(self.waypoints_x_list):
                    self.waypoints_x = self.waypoints_x_list[self.wayp_index]  # goal x position [m]
                    self.waypoints_y = self.waypoints_y_list[self.wayp_index]  # goal y position [m]
                else:
                    return
            elif rho_wp<rho:
                print('control: rho_goal<rho, track rho_goal!')
                self.fnTrackPcontrol(rho_wp, alpha)
                msg = Point()
                msg.x = rho_wp
                msg.y = alpha
                self.trajRThetaPub.publish(msg)
            # elif rho <= self.GOAL_DIST_THRES:
            #     print('control: achieve traj sub-goal, generate next sub-goal!')
            #     self.generate_next_goal()
            # elif abs(np.arccos(ang_btw))>np.pi/2:
            #     print('control: traj behind waypoints, generate next sub-goal!')
            else:
                # print('control: track rho!',rho, alpha)
                self.fnTrackPcontrol(rho, alpha)
                msg = Point()
                msg.x = rho
                msg.y = alpha
                self.trajRThetaPub.publish(msg)
                
            '''generate next trajectory'''
            self.generate_next_goal()
        
        elif self.current_mode == self.MotionSM.moveToObs.value:
            print('state: move to obs')
            x_diff = self.visited_id_dict[self.visited_id][0] - self.robotPoseX
            y_diff = self.visited_id_dict[self.visited_id][1] - self.robotPoseY
            msgGoal = Point()
            msgGoal.x = self.visited_id_dict[self.visited_id][0]
            msgGoal.y = self.visited_id_dict[self.visited_id][1]
            self.goalPub.publish(msgGoal)
            rho = np.hypot(x_diff, y_diff)-1.1
            goal_arc = np.arctan2(y_diff, x_diff)
            alpha = (goal_arc - self.robotPoseTheta + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
            
            print('rho/alpha: ', rho, alpha)
            if rho < self.GOAL_DIST_THRES and alpha < 0.1:
                self.current_mode = self.MotionSM.stabilizeSensing.value
                self.startSTOPtime = time.time()
            else:
                self.fnTrackPcontrol(rho, alpha)
                msg = Point()
                msg.x = rho
                msg.y = alpha
                self.trajRThetaPub.publish(msg)
        
        elif self.current_mode == self.MotionSM.stabilizeSensing.value:
            print('state: stabilizeSensing')
            if time.time()-self.startSTOPtime>3:
                self.current_mode = self.MotionSM.exploration.value
                self.visited_id_dict[self.visited_id] = self.CentroidTracker.objects[self.visited_id]
                self.ox.append(self.visited_id_dict[self.visited_id][0])
                self.oy.append(self.visited_id_dict[self.visited_id][1])
                self.index += 1
                print(self.ox)
                print(self.oy)

                # print(self.CentroidTracker.objects)
                # print(self.CentroidTracker.showup)
                # json_object = json.dumps(self.CentroidTracker.objects, indent=4)
                # file_name = 'centroidTracker_objects'      
                # with open(file_name+'.json', 'a+') as outfile:
                #     outfile.write(json_object)
                # json_object = json.dumps(self.CentroidTracker.showup, indent=4)
                # file_name = 'centroidTracker_showup'      
                # with open(file_name+'.json', 'a+') as outfile:
                #     outfile.write(json_object)

            else:
                self.fnStop()
                msgGoal = Point()
                msgGoal.x = self.visited_id_dict[self.visited_id][0]
                msgGoal.y = self.visited_id_dict[self.visited_id][1]
                self.goalPub.publish(msgGoal)
        
        elif self.current_mode == self.MotionSM.finish.value:
            print('state: finish zigzag-explore all area')
            self.fnStop()

    def calc_potential_field(self, x, y):
        af = self.calc_attractive_potential(x, y)
        rf = self.calc_repulsive_potential(x, y)
        af = self.check_APFupperBound(af, 'attr-')
        rf = self.check_APFupperBound(rf, 'repul-')
        # print('af: {}; rf: {}'.format(af,rf))
        msg = Point()
        msg.x = af[0]
        msg.y = af[1]
        self.potential_af_Pub.publish(msg)
        msg = Point()
        msg.x = rf[0]
        msg.y = rf[1]
        self.potential_rf_Pub.publish(msg)
        return af+rf
    
    def calc_attractive_potential(self,x,y):
        distance = self.vector_dist((x,y),(self.waypoints_x, self.waypoints_y))

        if distance <= self.ATTRACT_DIST_THRES:
            # print('attr smaller, goal:',np.array([self.waypoints_x, self.waypoints_y]) - np.array([x,y]))
            return np.array([self.waypoints_x, self.waypoints_y]) - np.array([x,y])
        else:
            # print('attr larger')
            return self.ATTRACT_DIST_THRES/distance * ( np.array([self.waypoints_x, self.waypoints_y]) - np.array([x,y]) )

    def calc_repulsive_potential(self,x,y):
        tmp = np.array([0.0,0.0])
        Q = self.OBSTACLE_THRES
        x_diff_wp, y_diff_wp = self.waypoints_x - x, self.waypoints_y - y
        # print('-----------', self.ox)
        # print('-----------', self.oy)
        for i in range(len(self.ox)):
            D = self.vector_dist( (x,y),(self.ox[i],self.oy[i]) ) - 0.3 #self.oradi[i]
            x_diff, y_diff = self.ox[i]-x,self.oy[i]-y
            rho_wp = np.hypot(x_diff_wp, y_diff_wp)
            rho = np.hypot(x_diff, y_diff)
            tmp_way_vec = np.array([x_diff_wp, y_diff_wp])  
            tmp_obs_vec = np.array([x_diff, y_diff])
            dot = tmp_way_vec.dot(tmp_obs_vec)
            if D <= self.OBSTACLE_THRES and abs(np.arccos(dot/rho_wp/rho))<np.pi/2:
                tmp += (-1/D+1/Q)/(D**2) * (np.array([self.ox[i], self.oy[i]]) - np.array([x,y])) /2 
            
        return tmp

    def cbOdom(self,msg):
        self.robotPoseX = msg.pose.pose.position.x
        self.robotPoseY = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, 
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w)
        _,_,self.robotPoseTheta = tf.transformations.euler_from_quaternion(quaternion)

    def cbTrunk(self,msg):
        trunk_data = msg
        distance, theta, radi = self.get_closest_trunk(trunk_data)
        if (theta is not None) and (radi is not None):
            map_trunk_point = np.array([[distance*np.cos(theta)],[distance*np.sin(theta)],[1]])
            # print('>>>>>>map_trunk_point: ', map_trunk_point)
            transform_utm2map = np.array([ [np.cos(self.robotPoseTheta), -np.sin(self.robotPoseTheta),self.robotPoseX],\
                                        [np.sin(self.robotPoseTheta), np.cos(self.robotPoseTheta), self.robotPoseY], \
                                        [0,0,1]] )

            self.utm_trunk = np.dot(transform_utm2map, map_trunk_point)
            # print('trunk_info: ',self.utm_trunk)
            centroid_tmpXYR = np.array([self.utm_trunk[0,0],self.utm_trunk[1,0],radi])
            centroid_tmpXY = np.array([[self.utm_trunk[0,0],self.utm_trunk[1,0]]])
            # print('centroid_tmpXYR: ',centroid_tmpXYR)
            self.CentroidTracker.update(centroid_tmpXY)
            
            showup_dict = self.CentroidTracker.showup
            objects_dict = self.CentroidTracker.objects
            # print('a',showup_dict)
            for item in showup_dict.items():
                now_key, now_value = item
                if now_key not in self.visited_id_dict.keys():
                    if now_value > 10:
                        print('now detect tree: ', now_key, ' move to it!')
                        self.current_mode = self.MotionSM.moveToObs.value
                        self.visited_id_dict[now_key]=objects_dict[now_key]
                        self.visited_id = now_key
                        print('visited_id: ',self.visited_id)

        else:
            self.CentroidTracker.update(np.array([]))
        
 
    def generate_next_goal(self):
        
        pf_force_vec = self.calc_potential_field(self.robotPoseX, self.robotPoseY)
        msgGoal = Point()
        msgGoal.x = pf_force_vec[0]
        msgGoal.y = pf_force_vec[1]
        self.potentialPub.publish(msgGoal)
        # print('norm: ',np.linalg.norm(pf_force_vec*self.ROBOT_STEP))
        # if np.linalg.norm(pf_force_vec*self.ROBOT_STEP) < 0.2:
        #     self.traj_x, self.traj_y = pf_force_vec*self.ROBOT_STEP*5 + np.array([self.robotPoseX, self.robotPoseY])
        # else:
        self.traj_x, self.traj_y = pf_force_vec*self.ROBOT_STEP + np.array([self.robotPoseX, self.robotPoseY])
        msgGoal = Point()
        msgGoal.x = self.traj_x
        msgGoal.y = self.traj_y
        self.goalPub.publish(msgGoal)

    def fnTrackPcontrol(self, dist, angle):
        msgVel = Twist()
        if (angle > 0.5 or angle < -0.5):
            # while(abs(angle)>0.05):
            # print("adjust angle! angle: ", angle)
            angularVel = angle * self.angKp
            if np.abs(angularVel) > self.angVel_bound: 
                if angularVel > 0:
                    angularVel = self.angVel_bound
                else:
                    angularVel = -self.angVel_bound
            msgVel.angular.z = angularVel
            
            self.velPub.publish(msgVel)
            return
            # rospy.sleep(0.2)

        # elif dist > self.GOAL_DIST_THRES:# or abs(angle)>np.pi/2:
            # print("adjust vel: ")

        linearVel = dist * self.linKp
        if np.abs(linearVel) > self.linVel_bound:
            if linearVel > 0 :
                linearVel = self.linVel_bound
            else: 
                linearVel = -self.linVel_bound
        msgVel.linear.x = linearVel

        angularVel = angle * self.angKp
        if np.abs(angularVel) > self.angVel_bound: 
            if angularVel > 0:
                angularVel = self.angVel_bound
            else:
                angularVel = -self.angVel_bound
        msgVel.angular.z = angularVel
        
        self.velPub.publish(msgVel)

    def fnTrackPcontrolObs(self, dist, angle):
        msgVel = Twist()
        if (angle > 0.5 or angle < -0.5):
            # while(abs(angle)>0.05):
            # print("adjust angle! angle: ", angle)
            angularVel = angle * self.angKp
            if np.abs(angularVel) > self.angVel_bound: 
                if angularVel > 0:
                    angularVel = self.angVel_bound
                else:
                    angularVel = -self.angVel_bound
            msgVel.angular.z = angularVel
            
            self.velPub.publish(msgVel)
            return
            # rospy.sleep(0.2)

        # elif dist > self.GOAL_DIST_THRES:# or abs(angle)>np.pi/2:
            # print("adjust vel: ")

        linearVel = dist * self.linKp
        if np.abs(linearVel) > self.linVel_bound:
            if linearVel > 0 :
                linearVel = self.linVel_bound
            else: 
                linearVel = -self.linVel_bound
        msgVel.linear.x = linearVel

        angularVel = angle * self.angKp
        if np.abs(angularVel) > self.angVel_bound: 
            if angularVel > 0:
                angularVel = self.angVel_bound
            else:
                angularVel = -self.angVel_bound
        msgVel.angular.z = angularVel
        
        self.velPub.publish(msgVel)

    def get_closest_trunk(self,trunk_data):
        minDist = self.trunkTrackDist
        minTheta = None
        minRadius = None
        while(len(trunk_data.aframe)):
            inAframe = trunk_data.aframe.pop()
            distance = inAframe.d
            if distance < minDist:
                minDist = distance
                minTheta = inAframe.t
                minRadius = inAframe.r
        # print('observed distance and theta: ',round(minDist,3),round(minTheta,3))

        return minDist, minTheta, minRadius

    def fnStop(self):
        msgVel = Twist()
        self.velPub.publish(msgVel)
    
    def vector_dist(self,a,b):
        a1,a2 = a
        b1,b2 = b
        return ((b1-a1)**2 + (b2-a2)**2)**0.5 

    def check_APFupperBound(self,vector,msg):
        if abs(vector[0])>1.5 and abs(vector[1])>1.5:
            if vector[0]>0:
                vector[0] = 1.5
            else:
                vector[0] = -1.5
            print(msg+'APF x&y: exceed 1')
            return vector
        if abs(vector[0])>1:
            if vector[0]>0:
                vector[0] = 1
            else:
                vector[0] = -1
            print(msg+'APF x: exceed 1')
        if abs(vector[1])>1:
            if vector[1]>0:
                vector[1] = 1
            else:
                vector[1] = -1
            print(msg+'APF y: exceed 1')
        return vector    

    def main(self):
        print("successfully initialized check main!")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("potential_field_planning_node", anonymous=True)
    apf_node = apf()
    apf_node.main()