#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from util2 import *
from math import *
import math
from std_msgs.msg import ColorRGBA, Header,Float32MultiArray,Float32,Float64
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from microstrain_inertial_msgs.msg import FilterHeading
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3
from atm_msg.msg import GoalInfo

class boatStatus:

    def __init__(self):
        rospy.init_node('boatStatus_Node', anonymous=True)

        slamTopic = rospy.get_param('/slam_topic_name', '/odom')

        #* Subscription
        self.subGoal = rospy.Subscriber('/navi/goal', GoalInfo, self.goalCallback)              # 목적지 정보
        self.subpsi  = rospy.Subscriber('/psi_d',Float64, self.psiCallback)                     # 추종 psi
        self.subOdom = rospy.Subscriber(slamTopic, Odometry, self.odomCallback)                 # 현재 위치, 속도, 헤딩
        self.subNaviHz = rospy.Subscriber('/navi/hz', Float64, self.naviHzCallback)             # Guidance Node Hz 
        self.subGuidHz = rospy.Subscriber('/guid/hz', Float64, self.guidHzCallback)             # Guidance Node Hz 
        self.subCmdPWM = rospy.Subscriber('/ctrl/pwm', Float32MultiArray, self.CmdPWMCallback)  # CMD PWM 
        self.subRC_AUTO = rospy.Subscriber('/RC_Auto', Float64, self.RC_AUTO_Callback)          # RC_AUTO info

        while not rospy.is_shutdown():

            self.process()

    
    def goalCallback(self, msg):
        self.goalData = msg

    def psiCallback(self, msg):
        self.psiData = msg

    def odomCallback(self, msg):
        self.odometryData = msg

        #* 보트의 상태 저장
        boatOrientation = eulerFromQuaternion(self.odometryData.pose.pose.orientation.x,self.odometryData.pose.pose.orientation.y,self.odometryData.pose.pose.orientation.z,self.odometryData.pose.pose.orientation.w)
        pose = [self.odometryData.pose.pose.position.x, self.odometryData.pose.pose.position.y, boatOrientation[2]] # [x, y, psi]
        vel  = [self.odometryData.twist.twist.linear.x, self.odometryData.twist.twist.linear.y, self.odometryData.twist.twist.angular.z] # [u, v, r]
        psi = pose[2] * 180/np.pi
        self.syaw = boatOrientation[2]
        self.x_map_data = pose[0]  #my point
        self.y_map_data = pose[1] 

    def guidHzCallback(self, msg):
        self.guidHz = msg

    def CmdPWMCallback(self, msg):
        self.CmdPWM = msg

    def RC_AUTO_Callback(self, msg):
        self.RC_Auto = msg  #! 0 => RC / 1 => Auto 

    def naviHzCallback(self, msg):
        self.naviHz = msg

    def process(self):

        if (self.odometryData is None) or (self.psiData is None) or (self.goalData is None) or (self.guidHz is None) or (self.CmdPWM is None) or (self.RC_Auto is None):
            return
        

        
if __name__ == '__main__':
    try:
        boatStatus()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
