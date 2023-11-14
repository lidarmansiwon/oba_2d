#!/usr/bin/env python3
#-*- coding:utf-8 -*-
'''
File: navigation_node
Author: 김시원 (Si Won Kim)
Date: 2023-09-18
Version: v1.0
Description: 해당 노드는 SLAM의 odometry로 부터 얻은 state를 이용하여 목적지, 모드를 변경한다.
''' 

import rospy
from util2 import *
from math import *
import math
from std_msgs.msg import Header,Float64
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from microstrain_inertial_msgs.msg import FilterHeading
from geometry_msgs.msg import Quaternion, Point, Twist
from atm_msg.msg import GoalInfo




class Navigation:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=True)

        #* Parameter 불러오기
        self.getParam()
        imuTopic  = rospy.get_param('/imu_topic_name', '/imu_data')
        slamTopic = rospy.get_param('/slam_topic_name', '/odom')    

        #* Subscription
        # self.subGPS = rospy.Subscriber("/gps/fix",NavSatFix, self.Callback_gps)
        self.subImu = rospy.Subscriber(imuTopic, FilterHeading, self.Callback_imu)
        self.subOdom = rospy.Subscriber(slamTopic, Odometry, self.odomCallback)

        #* Publisher
        self.pubHz = rospy.Publisher('/navi/hz', Float64, queue_size=10)   
        self.pubGoal = rospy.Publisher('/navi/goal', GoalInfo, queue_size=5)        


        #* Callback Data
        self.nextGoal_x = 0
        self.nextGoal_y = 0
        self.odometryData = None

        #* parameter
        self.mode = 1     #기본 모드 회피 모드   
        self.goal_triger = True
        self.prevTime = None
        self.map_check = True
        self.x_base ,self.y_base = [0,0]
        self.x_map_data, self.y_map_data = [0,0]

        while not rospy.is_shutdown():
            if self.odometryData is None:
                
                pass
            else:
                self.process()
                
    def getParam(self):
        self.goals                  = rospy.get_param('/nav_info/goals')
        self.safety_width           = rospy.get_param('guid_info/safety_width')
        self.goal_threshold         = rospy.get_param('nav_info/goal_threshold')
        self.MAX_detection_range    = rospy.get_param('guid_info/MAX_detection_range')
        self.MIN_detection_range    = rospy.get_param('guid_info/MIN_detection_range')

    def odomCallback(self, msg):
        self.odometryData = msg

    def Callback_imu(self, float32):
        self.yaw = (float32.heading_rad)     

    def process(self):

        #* 데이터 확인
        if (self.odometryData is None) or (self.goals is None):
            return
        
        if self.prevTime is None:
            self.prevTime = rospy.get_rostime()
            guidHz = 0
        else:
            currentTime = rospy.get_rostime()
            dt = ( currentTime-self.prevTime).to_sec()
            self.prevTime = currentTime
            guidHz = round(1/dt,2)

        self.DecisionGoal()

        remain_distance = math.sqrt((self.x_map_data-self.nextGoal_x)**2 + (self.y_map_data-self.nextGoal_y)**2)

        self.Navi(remain_distance)

        #* header update
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        #* goal data publish
        goalMsg = GoalInfo()
        goalMsg.header = header
        goalMsg.remain_distance = remain_distance
        goalMsg.threshold = self.goal_threshold 
        goalMsg.pose.position = Point(self.nextGoal_x, self.nextGoal_y, 0) # x, y, z
        goalMsg.pose.orientation = Quaternion(0, 0, 0, 1) # x, y, z, w 
        self.pubGoal.publish(goalMsg)
        self.pubHz.publish(guidHz)

    def DecisionGoal(self):
        
        #* 보트의 상태 저장
        boatOrientation = eulerFromQuaternion(self.odometryData.pose.pose.orientation.x,self.odometryData.pose.pose.orientation.y,self.odometryData.pose.pose.orientation.z,self.odometryData.pose.pose.orientation.w)
        pose = [self.odometryData.pose.pose.position.x, self.odometryData.pose.pose.position.y, boatOrientation[2]] # [x, y, psi]
        vel  = [self.odometryData.twist.twist.linear.x, self.odometryData.twist.twist.linear.y, self.odometryData.twist.twist.angular.z] # [u, v, r]

        self.syaw = boatOrientation[2]
        if self.map_check == True:
            
            if self.x_base == 0:
                self.x_base = pose[0] #! 내 시작 위치
                self.y_base = pose[1]   
                self.map_check = False

        else:

            self.x_map_data = pose[0]  #my point
            self.y_map_data = pose[1] 
            
            if self.goal_triger == True:

                self.nextGoal_x = self.goals[0][0]
                self.nextGoal_y = self.goals[0][1]
            else:
                self.nextGoal_x = self.goals[1][0]
                self.nextGoal_y = self.goals[1][1] 

    def Navi(self, remain_distance):

        if self.goal_triger == False and remain_distance < self.goal_threshold:
            self.goal_triger = True
            print("arrive first goal")

        elif self.goal_triger == True and remain_distance < self.goal_threshold:
            self.goal_triger = False
            print("arrive second goal")
        else:
            pass

if __name__ == '__main__':

    try:
        Navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass