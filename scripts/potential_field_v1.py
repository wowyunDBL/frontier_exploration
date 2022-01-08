#! /usr/bin/env python

'''ros utils'''
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, CameraInfo
import tf

'''math tool'''
from numpy.core.numeric import Inf
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

class apf():
    def __init__(self):
        
        self.velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.goalPub = rospy.Publisher('/explore/traj',Point,queue_size=5) #in the following variabl: traj=goal
        self.potentialPub = rospy.Publisher('/explore/potential',Point,queue_size=5)
        self.potential_af_Pub = rospy.Publisher('/explore/potential/af',Point,queue_size=5)
        self.potential_rf_Pub = rospy.Publisher('/explore/potential/rf',Point,queue_size=5)
        self.trajRThetaPub = rospy.Publisher('/explore/trajRTheta',Point,queue_size=5)
        self.odomSub = rospy.Subscriber("/odom",Odometry, self.cbOdom)
        # self.subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.cbDepth)
        
        self.ox = [1.0, 3.0, 5.0]  # obstacle x position list [m]
        self.oy = [2.0, 2.0, 2.0]  # obstacle y position list [m]    
        self.waypoints_x = 7.0  # goal x position [m]
        self.waypoints_y = 2.0  # goal y position [m]
        self.positive_bds = np.array([[0,-5],[20,-5],[20,5],[0,5]])
        self.angKp = 0.1
        self.linKp = 0.3
        self.angVel_bound = 0.5 # 0.5*57=23 degree
        self.linVel_bound = 0.5
        
        robotPose = rospy.wait_for_message('/odom',Odometry)
        self.robotPoseX = robotPose.pose.pose.position.x
        self.robotPoseY = robotPose.pose.pose.position.y
        quaternion = (robotPose.pose.pose.orientation.x, 
                        robotPose.pose.pose.orientation.y,
                        robotPose.pose.pose.orientation.z,
                        robotPose.pose.pose.orientation.w)
        _,_,self.robotPoseTheta = tf.transformations.euler_from_quaternion(quaternion)
        self.goal_x = None
        self.goal_y = None
        self.is_triggered = False
        rospy.set_param('/apf/trigger', False)
        
        self.GOAL_DIST_THRES = 0.1
        # self.WAYPOINTS_DIST_THRES = 0.
        self.ATTRACT_DIST_THRES = 1
        self.OBSTACLE_THRES = 1
        self.ROBOT_STEP = 0.5

        self.generate_next_goal()
        rospy.loginfo('Enter while detecting trigger_param...')
        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.is_triggered = rospy.get_param('/apf/trigger')
            if self.is_triggered == True:
                self.fnControlNode()
            else:
                self.fnStop()
            loop_rate.sleep()

        rospy.on_shutdown(self.fnShutDown)


    def vector_dist(self,a,b):
        a1,a2 = a
        b1,b2 = b
        return ((b1-a1)**2 + (b2-a2)**2)**0.5 

    def calc_potential_field(self, x, y):
        af = self.calc_attractive_potential(x, y)
        rf = self.calc_repulsive_potential(x, y)
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
            return np.array([self.waypoints_x, self.waypoints_y]) - np.array([x,y])
        else:
            return self.ATTRACT_DIST_THRES/distance * ( np.array([self.waypoints_x, self.waypoints_y]) - np.array([x,y]) )

    def calc_repulsive_potential(self,x,y):
        tmp = np.array([0.0,0.0])
        Q = 0.5
        for i in range(len(self.ox)):
            D = self.vector_dist( (x,y),(self.ox[i],self.oy[i]) )
            if D <= self.OBSTACLE_THRES:
                tmp += (-1/D+1/Q)/(D**2) * ( np.array([x,y]) - np.array([self.ox[i], self.oy[i]]) ) /2
            # else:
        return tmp

    def cbOdom(self,msg):
        self.robotPoseX = msg.pose.pose.position.x
        self.robotPoseY = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, 
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w)
        _,_,self.robotPoseTheta = tf.transformations.euler_from_quaternion(quaternion)

    def fnControlNode(self):
        x_diff, y_diff = self.goal_x - self.robotPoseX, self.goal_y - self.robotPoseY
        rho = np.hypot(x_diff, y_diff)
        goal_arc = np.arctan2(y_diff, x_diff)
        alpha = (goal_arc - self.robotPoseTheta + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
        # print("rho: {}; alpha: {}; goal_arc: {}".format(rho,alpha,goal_arc))
        msg = Point()
        msg.x = rho
        msg.y = alpha
        self.trajRThetaPub.publish(msg)

        x_diff_goal, y_diff_goal = self.waypoints_x - self.robotPoseX, self.waypoints_y - self.robotPoseY
        rho_goal = np.hypot(x_diff_goal, y_diff_goal)
        tmp_way_vec = np.array([x_diff_goal, y_diff_goal])  
        tmp_goal_vec = np.array([x_diff, y_diff])
        dot = tmp_way_vec.dot(tmp_goal_vec)
        if np.hypot(x_diff_goal, y_diff_goal) < self.GOAL_DIST_THRES:
            self.fnStop()
            print('control: achieve waypoint, fnStop!')
        elif rho <= self.GOAL_DIST_THRES or abs(np.arccos(dot/rho_goal/rho))>np.pi/2:
            print('control: achieve traj, generate next!')
            self.generate_next_goal()
        elif rho_goal<rho:
            print('control: rho_goal<rho, track rho_goal!')
            self.fnTrackPcontrol(rho_goal, alpha)
        else:
            print('control: track rho!')
            self.fnTrackPcontrol(rho, alpha)

    def fnTrackPcontrol(self, dist, angle):
        msgVel = Twist()
        if (angle > 0.3 or angle < -0.3) and False:
            # while(abs(angle)>0.05):
            print("adjust angle! angle: ", angle)
            angularVel = angle * self.angKp
            if np.abs(angularVel) > self.angVel_bound: 
                if angularVel > 0:
                    angularVel = self.angVel_bound
                else:
                    angularVel = -self.angVel_bound
            msgVel.angular.z = angularVel
            
            self.velPub.publish(msgVel)
            rospy.sleep(0.2)
                # angle = (goal_arc - self.robotPoseTheta + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]

            # return

        elif dist > self.GOAL_DIST_THRES:# or abs(angle)>np.pi/2:
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

 
    def generate_next_goal(self):
        
        pf_force_vec = self.calc_potential_field(self.robotPoseX, self.robotPoseY)
        msgGoal = Point()
        msgGoal.x = pf_force_vec[0]
        msgGoal.y = pf_force_vec[1]
        self.potentialPub.publish(msgGoal)

        self.goal_x, self.goal_y = pf_force_vec*self.ROBOT_STEP + np.array([self.robotPoseX, self.robotPoseY])
        msgGoal = Point()
        msgGoal.x = self.goal_x
        msgGoal.y = self.goal_y
        self.goalPub.publish(msgGoal)

    def fnStop(self):
        msgVel = Twist()
        self.velPub.publish(msgVel)

    def main(self):
        print("successfully initialized check main!")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("potential_field_planning_node", anonymous=True)
    apf_node = apf()
    apf_node.main()