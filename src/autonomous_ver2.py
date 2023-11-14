#!/usr/bin/env python3
#-*- coding:utf-8 -*-

'''
File: autonomous_node
Author: 김시원 (Si Won Kim)
Date: 2023-09-11
Version: v1.0
Description: 해당 노드는 NAV.로 부터 전달받은 목적지, 모드로 부터 2D lidar data를 이용하여 보트를 안전하게 유도하여 다양한 임무를 수행한다.
''' 

import rospy
from util2 import *
from sensor_msgs.msg import LaserScan,PointField, PointCloud2
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
import math, time
from math import * 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header,Float64MultiArray,Float32,Float64
from geometry_msgs.msg import Point, Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from microstrain_inertial_msgs.msg import FilterHeading
# from os import stat_result
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
# from pyproj import Proj,Transformer,transform,CRS
from functools import partial
import numpy as np
import pandas  as pds
from functools import partial
import sys
import sensor_msgs.point_cloud2 as pc2
from atm_msg.msg import GoalInfo




class Guidance:
    def __init__(self):
        rospy.init_node('autonomous_node', anonymous=True)

        #* Parameter 불러오기
        self.getParam()

        imuTopic  = rospy.get_param('/imu_topic_name', '/imu_data')
        slamTopic = rospy.get_param('/slam_topic_name', '/odom')
        lidarTopic= rospy.get_param('lidar_topic_name', '/navi/ray_cloud')

        #* Subscription
        
        #rospy.Subscriber("/ublox_gps/fix",NavSatFix, Callback_gps)
        # self.subGPS = rospy.Subscriber("/gps/fix",NavSatFix, self.Callback_gps)
        self.subLaser = rospy.Subscriber('/ouster/scan', LaserScan, self.Callback_lidar)
        self.subImu = rospy.Subscriber(imuTopic, FilterHeading, self.Callback_imu)
        self.subCamera = rospy.Subscriber('/camera_angle', Float64, self.Callback_camera)
        self.subOdom = rospy.Subscriber(slamTopic, Odometry, self.odomCallback)
        self.subPcl = rospy.Subscriber(lidarTopic, PointCloud2, self.Callback_pcl)
        self.subNextGoal = rospy.Subscriber('/navi/next_goal', Float64MultiArray, self.nextGoalCallback)
        self.subGoal = rospy.Subscriber('/navi/goal', GoalInfo, self.goalCallback)  


        #* Publisher
        self.psi_d = rospy.Publisher("/guid/psi_d",Float64,queue_size=1)
        self.now_mode = rospy.Publisher("/guid/now_mode",Float64,queue_size=1) #!now_mode 
        self.Auto_or_Docking = rospy.Publisher("/guid/Auto_or_Docking",Float64,queue_size=1)
        self.pubHz = rospy.Publisher('/guid/hz', Float64, queue_size=10)   
        # rviz
        self.visualgoal = rospy.Publisher("/vis/goal", Marker, queue_size=1)
        self.visualgoal_thd = rospy.Publisher("/vis/goal_thd", Marker, queue_size=1)
        self.heading_pub = rospy.Publisher("/vis/heading", Marker, queue_size=1)
        self.possible_pub = rospy.Publisher("vis/possible", Marker, queue_size=1)
        self.dock_pub = rospy.Publisher("/vis/dock", Marker, queue_size=1)
        self.object_pub = rospy.Publisher("/vis/object", MarkerArray, queue_size=1)
        self.wall_pub = rospy.Publisher("/vis/wall", MarkerArray, queue_size=1)
        self.between_pub = rospy.Publisher("/vis/between_lines", MarkerArray, queue_size=1)
        self.Mid_pub = rospy.Publisher("/vis/Mid", MarkerArray, queue_size=1)
        self.range_pub = rospy.Publisher('rviz/circle_grid', MarkerArray,queue_size=1)
        self.finalPsi_pub = rospy.Publisher("/vis/finalPsi", Marker, queue_size=1)
        self.pub_docking_point = rospy.Publisher("/To_docking_point", Marker, queue_size=1)

        #* Callback Data
        self.length = None
        self.dot = None
        self.yaw = None
        self.docking_angle = None
        self.odometryData = None
        self.ray = None
        self.mode = 1 #기본 모드 회피 모드   
        self.goalData = None

        #* parameter
        self.prevTime = None
        self.map_check = True
        self.x_base ,self.y_base = [0,0]
        self.goal_triger = True

        while not rospy.is_shutdown():
            if self.ray is None:
                pass
            else:
                self.process()

        #* 10HZ Timer
        # rospy.Timer(rospy.Duration(0.05), self.process)  --> 더 속도가 느렸음
    def getParam(self):
        self.goals                  = rospy.get_param('/nav_info/goals')
        self.safety_width           = rospy.get_param('guid_info/safety_width')
        self.goal_threshold         = rospy.get_param('nav_info/goal_threshold')
        self.MAX_detection_range    = rospy.get_param('guid_info/MAX_detection_range')
        self.MIN_detection_range    = rospy.get_param('guid_info/MIN_detection_range')

    def odomCallback(self, msg):
        self.odometryData = msg

        #* 보트의 상태 저장
        boatOrientation = eulerFromQuaternion(self.odometryData.pose.pose.orientation.x,self.odometryData.pose.pose.orientation.y,self.odometryData.pose.pose.orientation.z,self.odometryData.pose.pose.orientation.w)
        pose = [self.odometryData.pose.pose.position.x, self.odometryData.pose.pose.position.y, boatOrientation[2]] # [x, y, psi]
        vel  = [self.odometryData.twist.twist.linear.x, self.odometryData.twist.twist.linear.y, self.odometryData.twist.twist.angular.z] # [u, v, r]
        # psi = pose[2] * 180/np.pi
        # print(pose,"pose")
        self.syaw = boatOrientation[2]

        self.x_map_data = pose[0]  #my point
        self.y_map_data = pose[1] 
                

    def Callback_imu(self, float32):
        self.yaw = (float32.heading_rad) 

    def Callback_lidar(self, msg):
        self.length = len(msg.ranges)
        self.dot=msg.ranges

    def Callback_pcl(self, msg):
        self.ray = msg 

    def Callback_camera(self, data):
        self.docking_angle = round(data.data,1)
    
    def nextGoalCallback(self, msg):
        self.nextGoal = msg.data

    def goalCallback(self, msg):
        self.goalData = msg
    
    def process(self):

        object = []
        object_group = []
        

        #* 데이터 확인
        # if (self.odometryData is None) or (self.ray is None) or (self.goalData is None) or (self.Laser is None) or (self.nextGoal is None):
        if (self.odometryData is None) or (self.ray is None) or (self.goalData is None):
            return

        if self.prevTime is None:
            self.prevTime = rospy.get_rostime()

            guidHz = 0
        else:
            currentTime = rospy.get_rostime()
            dt = ( currentTime-self.prevTime).to_sec()
            self.prevTime = currentTime
            guidHz = round(1/dt,2)

        pc_data = pc2.read_points(self.ray, field_names=("x", "y", "z"), skip_nans=True)

        cloud = []
        for point in pc_data:
            x, y,_= point

            point_degree = np.arctan2(y,x)*180/np.pi

            how_long_distance = math.sqrt((x-0)**2 + (y-0)**2)

            if how_long_distance > self.MAX_detection_range:
                pass
            elif how_long_distance < self.MIN_detection_range:
                pass
            else:
                
                if x < 0:
                    pass
                else:
                    cloud.append((x,y))

        first_object = [[0.1,-0.5]]
        last_object = [[0.1,0.5]]

        init = True
        for point in cloud:
            if init == True:
                init = False
                prev_point = point
            else:
                point_dist = distance(point, prev_point)

                if point_dist <= 1:
                    object.append(prev_point)

                    if point == cloud[-1]:
                        object_group.append(object)
                    else:
                        pass

                elif point_dist > 1:

                    object.append(prev_point)
                    object_group.append(object)
                    object = [point]

                prev_point = point

        
        object_group.insert(0, first_object)
        object_group.append(last_object)

        #! object_group 내에서 갈 수 있는 좌표들 선정
        correct = self.correcting(object_group)
        

        # object visualization
        self.visualObject(object_group)
        self.visualBetOb(object_group)
        self.visualHd()

        #* Goal 정보 저장

        self.slam_x_goal = self.goalData.pose.position.x
        self.slam_y_goal = self.goalData.pose.position.y

        #! using GPS  --> 목적지를 회전행렬로 변환하는 작업
        # trans_x = change_x(self.x_goal-self.x_gps_data,-(self.y_goal-self.y_gps_data),0,0, self.yaw)
        # trans_y = change_y(self.x_goal-self.x_gps_data,-(self.y_goal-self.y_gps_data),0,0, self.yaw)

        #! using Slam 
        slam_tranGoal_x = change_x(self.slam_x_goal-self.x_map_data,(self.slam_y_goal-self.y_map_data),0,0, -(self.syaw))
        slam_tranGoal_y = change_y(self.slam_x_goal-self.x_map_data,(self.slam_y_goal-self.y_map_data),0,0, -(self.syaw))

        self.visualGoal(slam_tranGoal_x,slam_tranGoal_y)
        self.visgoal_thd(self.goal_threshold, slam_tranGoal_x, slam_tranGoal_y)


        #! DP 모드
        if self.mode == 0:
            print("mode is 0")

        #! 회피 모드
        elif self.mode == 1:

            auto = autonomous()
            auto.input(cloud, correct, slam_tranGoal_x, slam_tranGoal_y, object_group)
            final_psi, final_x, final_y = auto.decide_psi()
            self.psi_d.publish(final_psi)
            # print(final_psi,final_x,final_y,"psi")
            if final_psi < 500:
                
                self.cmdPsiVisualization(final_psi)
                
            else:
                self.visualpsi(final_x,final_y)

        #! 도킹모드
        elif self.mode == 2:

            print("docking mode")

        self.pubHz.publish(guidHz)
    def correcting(self, object_group):

        self.correct = []

        ob = True
        for object in object_group:
            
            if ob == True:
                ob = False
                prev_object = object
                
            else:
                safe_distance = distance(prev_object[-1],object[0])
                
                if safe_distance >= self.safety_width: 

                    start_distance = math.sqrt((object[0][0]-0)**2 + (object[0][1]-0)**2)
                    dist_end = math.sqrt((prev_object[-1][0]-0)**2 + (prev_object[-1][1]-0)**2)
                    different = abs(start_distance - dist_end)

                    if start_distance < dist_end:    #! 첫점이 더 가까울 때 -> 각도 - 들어가는 경우 

                        if object[0][1] == 0 and object[0][0] == 0:
                            pass
                        else:
                            psi_angle = atan(object[0][1]/object[0][0])  #! atan change 
                                
                            if object[0][0] <= 0:
                                
                                if object[0][1] < 0:
                                    psi_d = (psi_angle - math.pi)
                                    return psi_d
                                elif object[0][1] > 0:
                                    psi_d = (psi_angle + math.pi)
                                    return psi_d
                            elif object[0][0] == 0:
                                psi_d = 0
                                return psi_d
                            elif object[0][0] > 0:
                                pass
                            #! ------
                            if start_distance >= 4:

                                Mid_point_x = (prev_object[-1][0]+object[0][0])/2
                                Mid_point_y = (prev_object[-1][1]+object[0][1])/2
                                correct_point = (Mid_point_x,Mid_point_y)
                                self.correct.append(correct_point)
                                

                            elif start_distance <= 1.2:

                                Mid_point_x = (prev_object[-1][0]+object[0][0])/2
                                Mid_point_y = (prev_object[-1][1]+object[0][1])/2
                                #k1 = To_close_line_start(object[0][0],object[0][1])
                                k1 = new_street_line_start(Mid_point_x,Mid_point_y)
                                r = start_distance
                                x_start1 = math.sqrt((r**2)/(1+(k1**2)))
                                x_start2 = -(math.sqrt((r**2)/(1+(k1**2))))
                                y_start1 = k1*x_start1
                                y_start2 = k1*x_start2
                                
                                s_distance1 = math.sqrt((x_start1-Mid_point_x)**2 + (y_start1-Mid_point_y)**2)
                                s_distance2 = math.sqrt((x_start2-Mid_point_x)**2 + (y_start2-Mid_point_y)**2)
                                if s_distance1 < s_distance2:
                                    correct_point = (x_start1,y_start1)
                                    self.correct.append(correct_point)
                                else:
                                    correct_point = (x_start2,y_start2)
                                    self.correct.append(correct_point)
                                    

                            elif different < 0.8:
                                Mid_point_x = (prev_object[-1][0]+object[0][0])/2
                                Mid_point_y = (prev_object[-1][1]+object[0][1])/2
                                correct_point = (Mid_point_x,Mid_point_y)
                                self.correct.append(correct_point)
                            
                            else:
                                k1 = new_street_line_start(object[0][0],object[0][1])
                                a = change_atan(object[0][0],object[0][1])
                                r = start_distance
                                x_start1 = math.sqrt((r**2)/(1+(k1**2)))
                                x_start2 = -(math.sqrt((r**2)/(1+(k1**2))))


                                y_start1 = k1*x_start1
                                y_start2 = k1*x_start2

                                s_distance1 = math.sqrt((x_start1-object[0][0])**2 + (y_start1-object[0][1])**2)
                                s_distance2 = math.sqrt((x_start2-object[0][0])**2 + (y_start2-object[0][1])**2)

                                if s_distance1 < s_distance2:

                                    correct_point = (x_start1,y_start1)
                                    self.correct.append(correct_point)
                                else:
                                    correct_point = (x_start2,y_start2)
                                    self.correct.append(correct_point)

                    else: #! 끝점이 더 가까울 때 -> 각도 + 들어가는 경우 
                        
                        if object[0][1] == 0 and object[0][0] == 0:
                            pass
                        else:
                            psi_angle = atan(prev_object[-1][1]/prev_object[-1][0])  #! atan change 

                            if prev_object[-1][0] <= 0:
                                
                                if prev_object[-1][1] < 0:
                                    psi_angle = (psi_angle - math.pi)
                                    
                                elif prev_object[-1][1] > 0:
                                    psi_angle = (psi_angle + math.pi)
                                    
                            elif prev_object[-1][0] == 0:
                                psi_angle = 0
                                
                            elif prev_object[-1][0] > 0:
                                pass
                            #! ------
                            if dist_end >= 4:
                                Mid_point_x = (prev_object[-1][0]+object[0][0])/2
                                Mid_point_y = (prev_object[-1][1]+object[0][1])/2
                                correct_point = (Mid_point_x,Mid_point_y)
                                self.correct.append(correct_point)

                            elif dist_end <= 1.2:
                                Mid_point_x = (prev_object[-1][0]+object[0][0])/2
                                Mid_point_y = (prev_object[-1][1]+object[0][1])/2
                                k2 = new_street_line_end(Mid_point_x,Mid_point_y )
                                r = dist_end
                                x_end1 = math.sqrt((r**2)/(1+(k2**2)))
                                x_end2 = -(math.sqrt((r**2)/(1+(k2**2))))
                                y_end1 = k2*x_end1
                                y_end2 = k2*x_end2
                                e_distance1 = math.sqrt((x_end1-Mid_point_x)**2 + (y_end1-Mid_point_y)**2)
                                e_distance2 = math.sqrt((x_end2-Mid_point_x)**2 + (y_end2-Mid_point_y)**2)
                                if e_distance1 < e_distance2:
                                    correct_point = (x_end1,y_end1)
                                    self.correct.append(correct_point)
                                else:
                                    correct_point = (x_end2,y_end2)
                                    self.correct.append(correct_point)
            
                            elif different < 0.8:
                                Mid_point_x = (prev_object[-1][0]+object[0][0])/2
                                Mid_point_y = (prev_object[-1][1]+object[0][1])/2
                                correct_point = (Mid_point_x,Mid_point_y)
                                self.correct.append(correct_point)
                                
                            else:
                                k2 = new_street_line_end(prev_object[-1][0],prev_object[-1][1])
                                a = change_atan(prev_object[-1][0],prev_object[-1][1])
                                r = dist_end
                                x_end1 = math.sqrt((r**2)/(1+(k2**2)))
                                x_end2 = -(math.sqrt((r**2)/(1+(k2**2))))
                                y_end1 = k2*x_end1
                                y_end2 = k2*x_end2
                                e_distance1 = math.sqrt((x_end1-prev_object[-1][0])**2 + (y_end1-prev_object[-1][1])**2)
                                e_distance2 = math.sqrt((x_end2-prev_object[-1][0])**2 + (y_end2-prev_object[-1][1])**2)

                                if e_distance1 < e_distance2:

                                    correct_point = (x_end1,y_end1)
                                    self.correct.append(correct_point)
                                else:
                                    correct_point = (x_end2,y_end2)
                                    self.correct.append(correct_point)   
                prev_object = object

        return self.correct

    def visualGoal(self, slam_tranGoal_x, slam_tranGoal_y):

        visGoal = Marker()
        visGoal.header.frame_id = "body"
        visGoal.id = 1
        visGoal.ns = "goal"
        visGoal.type = Marker.SPHERE
        visGoal.action = Marker.ADD
        visGoal.color = ColorRGBA(0, 0.8, 0, 1)
        # visGoal.points = []
        # visGoal.points.append(Point(slam_tranGoal_x, slam_tranGoal_y, 0))
        visGoal.pose.position.x = slam_tranGoal_x
        visGoal.pose.position.y = slam_tranGoal_y
        visGoal.pose.position.z = 0.0
        visGoal.pose.orientation.w = 1.0
        visGoal.scale = Vector3(0.5, 0.5, 0.1)
        self.visualgoal.publish(visGoal)

    def visualpsi(self, x, y):

        finalPsi = Marker()
        finalPsi.header.frame_id = "body"
        finalPsi.ns = "psi"
        finalPsi.id = 111
        finalPsi.type = Marker.LINE_STRIP
        finalPsi.action = Marker.ADD
        finalPsi.color = ColorRGBA(0, 1, 0, 0.8)
        finalPsi.scale = Vector3(0.1, 0.1, 0)
        finalPsi.points = []
        finalPsi.points.append(Point(0, 0, 0))
        finalPsi.points.append(Point(x, y, 0))
        finalPsi.pose.orientation.w = 1.0
        finalPsi.lifetime = rospy.Duration.from_sec(0.1)
        self.finalPsi_pub.publish(finalPsi)   

    def cmdPsiVisualization(self, final_psi):

        final_psi = final_psi*(math.pi/180)
        marker = Marker()
        marker.header.frame_id = "body"  # rviz에서 표시될 좌표 프레임
        marker.id = 112
        marker.type = marker.ARROW
        marker.scale.x = 1.5  # 화살표의 끝 부분의 크기
        marker.scale.y = 0.11  # 화살표의 중간 부분의 크기
        marker.scale.z = 0.1  # 화살표의 끝 부분의 크기
        marker.color.a = 1.0  # 투명도
        marker.color.r = 1.0  # 색상 (빨강)
        marker.color.g = 0.2
        marker.color.b = 0.1
        # 화살표의 위치
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        quaternion = quaternionFromEuler(0.0, 0.0, final_psi)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        self.finalPsi_pub.publish(marker)

    def visualHd(self):

        heading = Marker()
        heading.header.frame_id = "body"
        heading.ns = "heading"
        heading.id = 61
        heading.type = Marker.LINE_STRIP
        heading.action = Marker.ADD
        heading.color = ColorRGBA(0, 1, 0, 0.8)
        heading.scale = Vector3(0.1, 0.1, 0)
        heading.points = []
        heading.points.append(Point(0, 0, 0))
        heading.points.append(Point(2, 0, 0))
        heading.pose.orientation.w = 1.0
        heading.lifetime = rospy.Duration.from_sec(0.1)
        self.heading_pub.publish(heading)

    def visualObject(self, object_group):
        #! Line to object
        #! yellow
        make2 = 0
        markerArray2 = MarkerArray()
        
        for object in object_group:
            
            line_color = ColorRGBA()
            line_color.r = 1.0
            line_color.g = 1.0
            line_color.b = 0.0
            line_color.a = 1.0  
            end = Point()
            end.x = object[-1][0]
            end.y = object[-1][1]
            end.z = 0.0
            start = Point()
            start.x = object[0][0]
            start.y = object[0][1]
            start.z = 0.0
            visOb = Marker()
            visOb.id = make2
            visOb.header.frame_id = "body"
            visOb.type = Marker.LINE_LIST
            visOb.ns = 'object line'
            visOb.action = Marker.ADD
            visOb.scale = Vector3(0.015, 0.015, 0)
            visOb.points = []
            visOb.points.append(end) 
            visOb.points.append(start) 
            visOb.colors = []
            visOb.colors.append(line_color) 
            visOb.colors.append(line_color)
            visOb.pose.orientation.w = 1
            visOb.lifetime = rospy.Duration.from_sec(0.1)
            markerArray2.markers = []
            markerArray2.markers.append(visOb)
            make2+=1
            self.object_pub.publish(markerArray2)

    def visualBetOb(self, object_group):  #! between start and end, #! Blue 

        between = 0
        markerArray_between = MarkerArray()  

        ob = True
        for object in object_group:
            line_color = ColorRGBA()
            line_color.r = 0.0
            line_color.g = 0.0
            line_color.b = 1.0
            line_color.a = 1.0

            if ob == True:
                ob = False
                prev_object = object
                
            else:        
                pre_end_point = Point()
                pre_end_point.x = prev_object[-1][0]
                pre_end_point.y = prev_object[-1][1]
                pre_end_point.z = 0.0
                start = Point()
                start.x = object[0][0]
                start.y = object[0][1]
                start.z = 0.0
                betob = Marker()
                betob.id = between
                betob.header.frame_id = "body"
                betob.type = Marker.LINE_LIST
                betob.ns = 'Testline3'
                betob.action = Marker.ADD
                betob.scale = Vector3(0.01, 0.01, 0)
                betob.points = []
                betob.points.append(pre_end_point)
                betob.points.append(start)
                betob.colors = []
                betob.colors.append(line_color)
                betob.colors.append(line_color)
                betob.pose.orientation.w = 1
                betob.lifetime = rospy.Duration.from_sec(0.1)
                markerArray_between.markers = []
                markerArray_between.markers.append(betob)
                between+=1
                self.between_pub.publish(markerArray_between)
                prev_object = object
                
    def visgoal_thd(self, goal_threshold, slam_tranGoal_x, slam_tranGoal_y):
        visGoal = Marker()
        visGoal.header.frame_id = "body"
        visGoal.id = 1
        visGoal.ns = "goal"
        visGoal.type = Marker.SPHERE
        visGoal.action = Marker.ADD
        visGoal.color = ColorRGBA(0.8, 0, 0, 0.2)
        visGoal.pose.position.x = slam_tranGoal_x
        visGoal.pose.position.y = slam_tranGoal_y
        visGoal.pose.position.z = -0.2
        visGoal.pose.orientation.w = 1.0
        visGoal.scale = Vector3(goal_threshold, 2, 0.1)
        self.visualgoal_thd.publish(visGoal)

class autonomous:
    def __init__(self):
        self.move = True
        self.psi_d = None
        self.width = 0.4

    def input(self, cloud, correct, slam_tranGoal_x, slam_tranGoal_y, object_group):
        self.cloud = cloud
        self.correct = correct
        self.slam_tranGoal_x = slam_tranGoal_x
        self.slam_tranGoal_y = slam_tranGoal_y
        self.object_group = object_group

    def decide_psi(self):

        if self.slam_tranGoal_x == 0:
            pass
        else:
            psi_angle = atan(self.slam_tranGoal_y/self.slam_tranGoal_x)

            if self.slam_tranGoal_x < 0:
                if self.slam_tranGoal_y < 0:
                    psi_angle = (psi_angle - math.pi)
                elif self.slam_tranGoal_y > 0:
                    psi_angle = (psi_angle - math.pi)
            elif self.slam_tranGoal_x > 0:
                pass
            
            count=0
            co = 0
            close = 0
            danger = 0
            for object in self.object_group:         
            
                for point in object:
                        if (0 <= point[0] <= 5.0) and (-0.5 <= point[1] <= 0.5):    #! 위험 계수 <하>
                            count+=1
                        else:
                            pass
                        if (0 <= point[0] <= 1) and (-0.4 <= point[1] <= 0.4):      #! 위험 계수 <중>
                            close+=1
                        else:
                            pass
                        if (0 <= point[0] <= 0.3) and (-0.2 <= point[1] <= 0.2):    #! 위험 계수 <상>
                            danger+=1
                        else:
                            pass

            if danger > 0:                                                      #! 너무 가까울 때
                final_psi = 1020
                final_x = 0 
                final_y = 0     

            elif close > 0:
                
                final_point = []

                if len(self.object_group) == 0:
                    final_psi = 1000
                    final_x = 0 
                    final_y = 0     
                else:
                    for correct_point in self.correct:

                        final_distance = math.sqrt((correct_point[0]-self.slam_tranGoal_x)**2 + (correct_point[1]-self.slam_tranGoal_y)**2)
                        final_goal = [final_distance,correct_point]
                        final_point.append(final_goal)                    

                    if len(final_point) == 0:
                        
                        final_psi, final_x, final_y = self.risk_comparison(self.cloud, self.object_group)
                    else:
                        final = min(final_point)
                        psi_angle = atan(final[1][1]/final[1][0])
                        
                        if final[1][0] <= 0:
                            if final[1][1] < 0:
                                psi_angle = (psi_angle - math.pi)
                            elif final[1][1] > 0:
                                psi_angle = (psi_angle + math.pi)
                        elif final[1][0] == 0:
                            psi_angle = 0
                        elif final[1][0] > 0:
                            pass
                        if psi_angle > math.pi:
                                psi_angle = psi_angle - (2*math.pi)
                        else: 
                            pass
                        if psi_angle*(180/math.pi) > 0:
                            final_psi = psi_angle*(180/math.pi)+40  #! 너무 가까울 때 좌현으로 피할 때

                        elif psi_angle*(180/math.pi) == 0:
                            final_psi = 1000
                        else:
                            final_psi = psi_angle*(180/math.pi)-40  #! 너무 가까울 때 우현으로 피할 때
                        final_x = final[1][0]
                        final_y = final[1][1]

            elif count <= 5: #! Safe
                
                final_psi = psi_angle
                final_x = 0 
                final_y = 0    
                print(final_psi,"safe")

            else:

                final_point = []

                if len(self.object_group) == 0:
                    
                    final_psi = 1000      
                    final_x = 0 
                    final_y = 0       

                else:

                    for correct_point in self.correct:

                        final_distance = math.sqrt((correct_point[0]-self.slam_tranGoal_x)**2 + (correct_point[1]-self.slam_tranGoal_y)**2)
                        final_goal = [final_distance,correct_point]
                        final_point.append(final_goal)   

                    if len(final_point) == 0:

                        final_psi, final_x, final_y = self.risk_comparison(self.cloud, self.object_group)

                    else:

                        final = min(final_point)
                        psi_angle = atan(final[1][1]/final[1][0])
                        
                        if final[1][0] <= 0:
                            if final[1][1] < 0:
                                psi_angle = (psi_angle - math.pi)
                            elif final[1][1] > 0:
                                psi_angle = (psi_angle + math.pi)
                        elif final[1][0] == 0:
                            psi_angle = 0
                        elif final[1][0] > 0:
                            pass

                        if psi_angle > math.pi:
                            psi_angle = psi_angle - (2*math.pi)
                        else: 
                            pass
                        
                        final_x = final[1][0]
                        final_y = final[1][1]

                        final_psi = psi_angle*(180/math.pi)

        return final_psi, final_x, final_y

    def risk_comparison(self, cloud, object_group):

        left_count = 0
        right_count = 0

        for point in cloud:

            if point[1] > 0:
                left_count += 1
            else:
                right_count += 1

        if left_count > right_count: #! 좌측이 더 위험 지역 (첫 가상 장애물에서 일정 각도 뺌)

            final_x = object_group[0][0][0]
            final_y = object_group[0][0][1]
            
            psi_angle = atan(final_y/final_x)

            if final_x <= 0:

                if final_y < 0:
                    psi_angle = (psi_angle - math.pi)

                elif final_y > 0:
                    psi_angle = (psi_angle + math.pi)

            elif final_x == 0:
                psi_angle = 0

            elif final_x > 0:
                pass

            final_psi = psi_angle*(180/math.pi)-40
            
        else:                       #! 우측이 더 위험 지역 (마지막 가상 장애물에서 일정 각도 뺌)
            final_x = object_group[-1][-1][0]
            final_y = object_group[-1][-1][1]
            psi_angle = atan(final_y/final_x)

            if final_x <= 0:

                if final_y < 0:
                    psi_angle = (psi_angle - math.pi)

                elif final_y > 0:
                    psi_angle = (psi_angle + math.pi)

            elif final_x == 0:
                psi_angle = 0

            elif final_x > 0:
                pass

            final_psi = psi_angle*(180/math.pi)+40

        return final_psi, final_x, final_y

if __name__ == '__main__':

    try:
        Guidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
