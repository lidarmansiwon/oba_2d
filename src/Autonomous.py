#!/usr/bin/env python3
#-*- coding:utf-8 -*-


# from cgi import print_arguments
# from email.quoprimime import header_decode
# from logging import Filter
# from xml.dom import NO_DATA_ALLOWED_ERR
# from std_msgs.msg import Bool
# from tokenize import String
# from turtle import TurtleScreenBase, right
import rospy
from util2 import *
from sensor_msgs.msg import LaserScan,PointField, PointCloud2
from sensor_msgs import point_cloud2
import sensor_msgs.point_cloud2 as pc2
import math, time
from math import * 
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header,Float32MultiArray,Float32,Float64
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
#auto: now_mode.publish(1)
#docking: now_mode.publish(2)
#RC: now_mode.publish(3)

class autonomous:

    Point_pub = rospy.Publisher("Point", Marker, queue_size=1)

    dot=[]
    la = 0
    lo = 0
    yaw= 0
    psi = 0
    length = 0
    gps_check = True
    map_check = True
    x_base ,y_base = [0,0]
    x_gps_data,y_gps_data = [0,0]
    x_map_data,y_map_data = [0,0]
    pre_x_goal,pre_y_goal = [0,0]
    slam_x_goal,slam_y_goal = [0,0]
    x_goal,y_goal = [0,0]
    mode_change = False
    slam_goal_change = False
    goal_1 = False
    num_point = []
    docking_angle = 1000
    a = 0
    psi_angle = 0
    docking_x,docking_y = 0,0
    F_yaw = 0   #! 정면을 바라보고 있다고 가정  / 실험 위해서 -90 인데 0으로 만들어서 각도 검출하도록
    find_1 = False
    right_left = False  #! 오른쪽이 False
    pre_x_goal = +0 #? 임시 목적지  위치
    pre_y_goal = 0
    x_goal = 2
    y_goal = 0
    ray = None

    def __init__(self):
        
        rospy.init_node('lidar_node', anonymous=True)
        #rospy.Subscriber("/ublox_gps/fix",NavSatFix, Callback_gps)
        rospy.Subscriber("/gps/fix",NavSatFix, self.Callback_gps)
        rospy.Subscriber('/ouster/scan', LaserScan, self.Callback_lidar)
        rospy.Subscriber('/nav/heading', FilterHeading, self.Callback_imu)
        rospy.Subscriber('/camera_angle', Float64, self.Callback_camera)
        rospy.Subscriber("/Odometry", Odometry, self.odomCallback)
        rospy.Subscriber("/navi/ray_cloud", PointCloud2, self.Callback_pcl)
        
        # rospy.Timer(rospy.Duration(0.05), self.Autonomous)
        while not rospy.is_shutdown():
            if self.ray is None:
                pass
            else:
                self.Autonomous()

            if self.mode_change == False: # not complete
                pass
            else:
                
                Auto_or_Docking.publish(2)
                pass

    def odomCallback(self, msg):
        self.odometryData = msg

        #* 보트의 상태 저장
        boatOrientation = eulerFromQuaternion(self.odometryData.pose.pose.orientation.x,self.odometryData.pose.pose.orientation.y,self.odometryData.pose.pose.orientation.z,self.odometryData.pose.pose.orientation.w)
        pose = [self.odometryData.pose.pose.position.x, self.odometryData.pose.pose.position.y, boatOrientation[2]] # [x, y, psi]
        vel  = [self.odometryData.twist.twist.linear.x, self.odometryData.twist.twist.linear.y, self.odometryData.twist.twist.angular.z] # [u, v, r]
        # psi = pose[2] * 180/np.pi
        # print(pose,"pose")
        self.syaw = boatOrientation[2]
        if self.map_check == True:
        
            if self.x_base == 0:
                self.x_base = pose[0] #! 내 시작 위치
                self.y_base = pose[1]   
                self.map_check = False

        else:
            self.x_map_data = pose[0]  #my point
            self.y_map_data = pose[1] 

            # if slam_goal_change == False:
            #     print("first")

            # else:
            #     print("next")
            #     goal_1 = True
    
            if self.goal_1 == True:
                self.slam_x_goal = 17.8 #- x_base #위도  1682987.5393221395 - x_base
                self.slam_y_goal = -0.9 #경도  1146762.899619877 - y_base      
            else:
                self.slam_x_goal = 0.8#위도  1682987.5393221395 - x_base
                self.slam_y_goal = (-0.95) #경도  1146762.899619877 - y_base   


    def Callback_gps(self, data):
    
        self.lo = data.longitude     #경도  1146762.899619877
        self.la = data.latitude      #위도  1682987.5393221395
        self.gps_data = UTM_K_TRS(la,lo)


        if self.gps_check == True:
            
            if self.x_base == 0:
                self.x_base = self.gps_data[0] #! 내 시작 위치
                self.y_base = self.gps_data[1]   
                gps_check = False
        else:
            self.x_gps_data = self.gps_data[0] - self.x_base #my point
            self.y_gps_data = self.gps_data[1] - self.y_base
            self.x_goal = 1675173.4778633614 - self.x_base #위도  1682987.5393221395 - x_base
            self.y_goal = 1098544.051586974 - self.y_base #경도  1146762.899619877 - y_base
            
            
    # def Callback_imu(float32):
    #     global yaw
    #     yaw = (float32.heading_rad)

    def Callback_imu(self, float32):
        self.yaw = (float32.heading_rad) 


    def Callback_lidar(self, msg):
        self.length = len(msg.ranges)
        self.dot=msg.ranges

    def Callback_pcl(self, msg):
        self.ray = msg 

    def Callback_camera(self, data):

        self.docking_angle = round(data.data,1)

            # rate = rospy.Rate(1000)
            # rate.sleep()



    def Autonomous(self):
        HEADER = Header(frame_id='os_sensor')

        possible_pub = rospy.Publisher("/possible", Marker, queue_size=1)
        heading_pub = rospy.Publisher("/heading", Marker, queue_size=1)
        danger1_pub = rospy.Publisher("/danger1", Marker, queue_size=1)
        danger2_pub = rospy.Publisher("/danger2", Marker, queue_size=1)
        dock_pub = rospy.Publisher("/dock", Marker, queue_size=1)
        object_pub = rospy.Publisher("/object", MarkerArray, queue_size=1)
        wall_pub = rospy.Publisher("/wall", MarkerArray, queue_size=1)
        between_pub = rospy.Publisher("/between_lines", MarkerArray, queue_size=1)
        Mid_pub = rospy.Publisher("/Mid", MarkerArray, queue_size=1)
        range_pub = rospy.Publisher('rviz/circle_grid', MarkerArray,queue_size=1)
        co_pub = rospy.Publisher("/correct", MarkerArray, queue_size=1)
        psi_d = rospy.Publisher("/psi_d",Float64,queue_size=1)
        now_mode = rospy.Publisher("/now_mode",Float64,queue_size=1) #!now_mode 
        pub_docking_point = rospy.Publisher("/To_docking_point", Marker, queue_size=1)
        Auto_or_Docking = rospy.Publisher("/Auto_or_Docking",Float64,queue_size=1)
        points = []
        object = []
        object_group = []
        view_object = [] #--> for view laserPoints  view_object,view_object_group,view_point
        view_object_group = [] #--> for view laserPoints
        view_point = []  #--> for view laserPoints
        # start_point = []
        # end_point = []
        # mid_point = []
        # start_tan = []
        # end_tan = []
        pre_end_point = []
        correct = []
        all_points = []
        psi_angle = 0
        R_ = []

        
        ###
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
        heading_pub.publish(heading)


        pc_data = pc2.read_points(self.ray, field_names=("x", "y", "z"), skip_nans=True)


        for point in pc_data:

            x, y,_= point
            view_point.append((x,y))

            all_points.append((x,y))
            how_long_distance = math.sqrt((x-0)**2 + (y-0)**2)
            
            if how_long_distance > 6:
                pass
            elif how_long_distance < 0.25:
                pass
            else:
                
                if x < 0:
                    pass
                else:
                    points.append((x,y))
        
        #making list grouped by each object
        first_object = []
        last_object = []
        right_small_object = (0.1,-1.5)
        first_object.append(right_small_object)
        left_small_object = (0.1,1.5)
        last_object.append(left_small_object)
        
        init = True
        for point in points:
            if init == True:
                init = False
                prev_point = point
            else:

                point_dist = distance(point, prev_point)
                
                #object_group.append(first_object)
                        
                if point_dist <= 1:
                    object.append(prev_point)
                    #print(prev_point,"prev_point")
                    # if len(object) > 50:
                    #     object_group.append(object)
                    if point == points[-1]:
                        object_group.append(object)
                    else:
                        pass

                elif point_dist > 1:

                    # if len(object) < 2:  #! 버리는 경우 취소
                    #     pass
                    # else:
                    object.append(prev_point)  #? TO fix (Index Error: list index out of range)
                    object_group.append(object)
                    object = [point]

                prev_point = point

        
        object_group.insert(0, first_object)
        object_group.append(last_object)

    #############################################
        init2 = True
        for point2 in view_point:
            if init2 == True:
                init2 = False
                prev_view = point2
            else:

                view_dist = distance(point2, prev_view)
                
                #object_group.append(first_object)
                        
                if view_dist <= 2:
                    view_object.append(prev_view)
                    #print(prev_point,"prev_point")
                    # if len(object) > 50:
                    #     object_group.append(object)
                    if point2 == view_point[-1]:
                        view_object_group.append(view_object)
                    else:
                        pass

                elif view_dist > 2:

                    # if len(object) < 2:  #! 버리는 경우 취소
                    #     pass
                    # else:
                    view_object.append(prev_view)  #? TO fix (Index Error: list index out of range)
                    view_object_group.append(view_object)
                    view_object = [point2]

                prev_view = point2
    #############################

        #! Line for all points
        #! red
        wall = 0
        wall_array = MarkerArray()
        
        for view_object in view_object_group:
            
        
            line_color = ColorRGBA()
            line_color.r = 1.0
            line_color.g = 0.0
            line_color.b = 0.0
            line_color.a = 1.0  
            end = Point()
            end.x = view_object[-1][0]
            end.y = view_object[-1][1]
            end.z = 0.0
            start = Point()
            start.x = view_object[0][0]
            start.y = view_object[0][1]
            start.z = 0.0
            wall_point = Marker()
            wall_point.id = wall
            wall_point.header.frame_id = "os_sensor"
            wall_point.type = Marker.LINE_LIST
            wall_point.ns = 'object line'
            wall_point.action = Marker.ADD
            wall_point.scale = Vector3(0.015, 0.015, 0)
            wall_point.points = []
            wall_point.points.append(end) 
            wall_point.points.append(start) 
            wall_point.colors = []
            wall_point.colors.append(line_color) 
            wall_point.colors.append(line_color)
            wall_point.pose.orientation.w = 1
            wall_point.lifetime = rospy.Duration.from_sec(0.1)
            wall_array.markers = []
            wall_array.markers.append(wall_point)
            wall+=1
            wall_pub.publish(wall_array)

    ##########################$


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
            marker2 = Marker()
            marker2.id = make2
            marker2.header.frame_id = "os_sensor"
            marker2.type = Marker.LINE_LIST
            marker2.ns = 'object line'
            marker2.action = Marker.ADD
            marker2.scale = Vector3(0.015, 0.015, 0)
            marker2.points = []
            marker2.points.append(end) 
            marker2.points.append(start) 
            marker2.colors = []
            marker2.colors.append(line_color) 
            marker2.colors.append(line_color)
            marker2.pose.orientation.w = 1
            marker2.lifetime = rospy.Duration.from_sec(0.1)
            markerArray2.markers = []
            markerArray2.markers.append(marker2)
            make2+=1
            object_pub.publish(markerArray2)

        #! between start and end
        #! Blue

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
                safe_distance = distance(prev_object[-1],object[0])
                
                if safe_distance >= 1:  #! 1미터에서 1.4로 수정함
                    

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
                                correct.append(correct_point)

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
                                    correct.append(correct_point)
                                else:
                                    correct_point = (x_start2,y_start2)
                                    correct.append(correct_point)
                                    

                            elif different < 0.8:
                                
                                Mid_point_x = (prev_object[-1][0]+object[0][0])/2
                                Mid_point_y = (prev_object[-1][1]+object[0][1])/2
                                correct_point = (Mid_point_x,Mid_point_y)
                                correct.append(correct_point)
                            
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
                                    correct.append(correct_point)
                                else:
                                    correct_point = (x_start2,y_start2)
                                    correct.append(correct_point)

                            


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
                                correct.append(correct_point)

                            elif dist_end <= 1.2:
                                
                                Mid_point_x = (prev_object[-1][0]+object[0][0])/2
                                Mid_point_y = (prev_object[-1][1]+object[0][1])/2
                                #k2 = To_close_line_end_(prev_object[-1][0],prev_object[-1][1])
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
                                    correct.append(correct_point)
                                else:
                                    correct_point = (x_end2,y_end2)
                                    correct.append(correct_point)
            
                            
                            elif different < 0.8:
                                
                                Mid_point_x = (prev_object[-1][0]+object[0][0])/2
                                Mid_point_y = (prev_object[-1][1]+object[0][1])/2
                                correct_point = (Mid_point_x,Mid_point_y)
                                correct.append(correct_point)
                                
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

                                    correct.append(correct_point)
                                else:
                                    correct_point = (x_end2,y_end2)

                                    correct.append(correct_point)
                        
                
                    #! circle rviz 

                    if start_distance < dist_end:

                        id_num=0
                        circle_lines =MarkerArray()
                        circle_lines.markers=[]
                        circle_dot = []

                        circle_dot.append(circle(0.0, 0.0, start_distance,0, 360,6))      
                        for i in circle_dot:  # circle point list input
                            points_cir = [] 
                            for j in i:
                                if j[0]==0.0  :
                                    pass
                                else:
                                    
                                    line = Point()
                                    line.x=j[1]
                                    line.y=j[0]
                                    line.z=0.0
                                    points_cir.append(line)
                            circle_line = Marker()
                            circle_line.header.stamp =rospy.Time.now()
                            circle_line.header.frame_id = 'os_sensor'
                            circle_line.type=Marker.LINE_STRIP
                            circle_line.action = Marker.ADD 
                            circle_line.id = id_num
                            circle_line.color.r, circle_line.color.g, circle_line.color.b = (0.9, 1, 0.9)
                            circle_line.color.a = 0.2
                            circle_line.points=points_cir
                            circle_line.scale.x, circle_line.scale.y, circle_line.scale.z =(0.01,0.01,0.0)
                            circle_line.pose.position.x, circle_line.pose.position.y =0,0
                            circle_line.pose.orientation.w = 1
                            circle_line.lifetime =rospy.Duration.from_sec(0.1)
                            circle_lines.markers.append(circle_line) 
                            id_num+=1
                        #id_num=0
                        range_pub.publish(circle_lines)


                    else: 
                        id_num22=0
                        circle_lines =MarkerArray()
                        circle_lines.markers=[]
                        circle_dot = []

                        circle_dot.append(circle(0.0, 0.0, dist_end,0, 360,6))      
                        for i in circle_dot:  # circle point list input
                            points_cir = [] 
                            for j in i:
                                if j[0]==0.0  :
                                    pass
                                else:
                                    line = Point()
                                    line.x=j[1]
                                    line.y=j[0]
                                    line.z=0.0
                                    points_cir.append(line)
                            circle_line = Marker()
                            circle_line.header.stamp =rospy.Time.now()
                            circle_line.header.frame_id = 'os_sensor'
                            circle_line.type=Marker.LINE_STRIP
                            circle_line.action = Marker.ADD 
                            circle_line.id = id_num22
                            circle_line.color.r, circle_line.color.g, circle_line.color.b = (0.9, 1, 0.9)
                            circle_line.color.a = 0.2
                            circle_line.points=points_cir
                            circle_line.scale.x, circle_line.scale.y, circle_line.scale.z =(0.01,0.01,0.0)
                            circle_line.pose.position.x, circle_line.pose.position.y =0,0
                            circle_line.pose.orientation.w = 1
                            circle_line.lifetime =rospy.Duration.from_sec(0.1)
                            circle_lines.markers.append(circle_line) 
                            id_num22+=1
                        #id_num=0
                        range_pub.publish(circle_lines)

                pre_end_point = Point()
                pre_end_point.x = prev_object[-1][0]
                pre_end_point.y = prev_object[-1][1]
                pre_end_point.z = 0.0
                start = Point()
                start.x = object[0][0]
                start.y = object[0][1]
                start.z = 0.0
                marker3 = Marker()
                marker3.id = between
                marker3.header.frame_id = "os_sensor"
                marker3.type = Marker.LINE_LIST
                marker3.ns = 'Testline3'
                marker3.action = Marker.ADD
                marker3.scale = Vector3(0.01, 0.01, 0)
                marker3.points = []
                marker3.points.append(pre_end_point)
                marker3.points.append(start)
                marker3.colors = []
                marker3.colors.append(line_color)
                marker3.colors.append(line_color)
                marker3.pose.orientation.w = 1
                marker3.lifetime = rospy.Duration.from_sec(0.1)
                markerArray_between.markers = []
                markerArray_between.markers.append(marker3)
                between+=1
                between_pub.publish(markerArray_between)
                

                prev_object = object


        #! using GPS  --> 목적지를 회전행렬로 변환하는 작업
        trans_x = change_x(self.x_goal-self.x_gps_data,-(self.y_goal-self.y_gps_data),0,0, self.yaw)
        trans_y = change_y(self.x_goal-self.x_gps_data,-(self.y_goal-self.y_gps_data),0,0, self.yaw)
        print(self.yaw)

        #! using Slam
        
        slam_trans_x = change_x(self.x_goal-self.x_map_data,(self.y_goal-self.y_map_data),0,0, self.syaw)
        slam_trans_y = change_y(self.x_goal-self.x_map_data,(self.y_goal-self.y_map_data),0,0, self.syaw)
        print(self.x_goal,"psi")
        #print(yaw,"yaw")
        #print(trans_x,trans_y,"trans_goal")

        rviz_points = Marker()
        rviz_points.header.frame_id = "body"
        rviz_points.id = 1
        rviz_points.type = Marker.POINTS
        rviz_points.action = Marker.ADD
        rviz_points.color = ColorRGBA(1, 1, 0, 1)
        rviz_points.points = []
        rviz_points.points.append(Point(trans_x, trans_y, 0))
        rviz_points.scale = Vector3(0.3, 0.3, 0)
        self.Point_pub.publish(rviz_points)

        remain_distance = math.sqrt((self.x_map_data-self.slam_x_goal)**2 + (self.y_map_data-self.slam_y_goal)**2)
        # remain_distance = math.sqrt((2-0)**2 + (0-0)**2)
        #print(remain_distance,"remainnnn")
        print(self.x_map_data,self.y_map_data,"aaaaaaaaaaaaaa")
        print(remain_distance,"remain_dis")
        print(self.slam_x_goal,self.slam_y_goal,"slam_goal")


        if self.goal_1 == False and remain_distance < 2:
            self.mode_change = False
            # slam_goal_change = True
            
            self.goal_1 = True

            psi_d.publish(0)
            time.sleep(1)
            psi_d.publish(1020)
            time.sleep(1.5)
            psi_d.publish(2010)
            time.sleep(2)
            psi_d.publish(1020)
            time.sleep(1.2)
            psi_d.publish(2010)
            time.sleep(1)
            print("turn")
            Auto_or_Docking.publish(2)
            #print(remain_distance,"complete")
            # time.sleep(3)
        elif self.goal_1 == True and remain_distance < 1.5:
            self.mode_change = False
            # slam_goal_change = True

            self.goal_1 = False
            print("turn point_1")

            psi_d.publish(1020)
            time.sleep(1)
            psi_d.publish(2010)
            time.sleep(1)
            psi_d.publish(1020)
            time.sleep(1)
            psi_d.publish(2010)
            time.sleep(1.8)
    


        
            Auto_or_Docking.publish(2)
            
        else:
            self.mode_change = False # Not yet
            # slam_goal_change = False
            print(self.x_goal,self.y_goal,"goal")
            Auto_or_Docking.publish(1)

        

        
        
        if self.mode_change == False:  #! 장애물 회피 ----------------------------------------------------------------------------------------------------------------
        
            print("Auto")
            now_mode.publish(1)
            Auto_or_Docking.publish(1)
            if trans_x == 0:
                pass

            else:
                psi_angle = atan(trans_y/trans_x)  #! atan change 
                                
                if trans_x <= 0:
                    
                    if trans_y < 0:
                        psi_angle = (psi_angle - math.pi)
                        
                    elif trans_y > 0:
                        psi_angle = (psi_angle + math.pi)
                        
                elif trans_x == 0:
                    psi_angle = 0
                    
                elif trans_x > 0:
                    pass
                #! ------

                count=0
                co = 0
                close = 0
                danger = 0
                for object in object_group:

                    # if len(object) < 2: #! 버리는 경우 취소
                    #     pass
                    # else:

                    for point in object:
                        if (0 <= point[0] <= 5.0) and (-0.7<= point[1] <= 0.7): #! 4에서 5로 바꿈 
                            count+=1
                        else:
                            pass
                        if (0 <= point[0] <= 1) and (-0.4 <= point[1] <= 0.4):  #! 0.8에서 1로 바꿈
                            close+=1
                        else:
                            pass
                        if (0 <= point[0] <= 0.3) and (-0.2 <= point[1] <= 0.2):
                            danger+=1
                        else:
                            pass
                
                
                if danger > 0: #! 너무 가까울 때
                    
                    psi_d.publish(1020)
                    # print("danger")


                elif close > 0: #! 너무 가까울 때

                    # print("close object")
                    markerArray_co = MarkerArray()
                    marker_co = Marker()
                    marker_co.header.frame_id = "body"
                    marker_co.ns = 'correct'
                    marker_co.type = Marker.LINE_STRIP
                    marker_co.action = Marker.ADD
                    marker_co.id = co
                    marker_co.scale.x = 0.1
                    marker_co.scale.y = 0.1
                    marker_co.color.a = 1.0
                    marker_co.color.r = 1.0
                    marker_co.color.g = 1.0
                    marker_co.color.b = 0.0 
                    marker_co.pose.orientation.w = 1.0
                    me = Point()  #? Arrow start point
                    me.x = 0
                    me.y = 0
                    me.z = 0
                    final_point = []
                        
                    if len(object_group) == 0:
                        
                        psi_d.publish(1000)
                        


                    else: #! 장애물이 두개 이상일 때 

                        for correct_point in correct:

                            final_distance = math.sqrt((correct_point[0]-trans_x)**2 + (correct_point[1]-trans_y)**2)
                            final_goal = [final_distance,correct_point]
                            final_point.append(final_goal)

                        if len(final_point) == 0: #! 두개 이상인데 순간적으로 갈 수 있는 점을 찾지 못했을 때 선수 축 기준으로 len(points) 비교 해서 더 적은 쪽으로 

                            left_count = 0
                            right_count = 0

                            for point in points:

                                if point[1] > 0:
                                    left_count += 1
                                else: #? point[1] < 0:
                                    right_count += 1
                            if left_count > right_count:  #! 좌측이 더 위험 지역 (장애물 첫점에서 일정 각도 뺌)
                                
                                x_final_point = object_group[0][0][0]
                                y_final_point = object_group[0][0][1]
                                
                                psi_angle = atan(y_final_point/x_final_point)
                                
                                if x_final_point <= 0:

                                    if y_final_point < 0:
                                        psi_angle = (psi_angle - math.pi)

                                    elif y_final_point > 0:
                                        psi_angle = (psi_angle + math.pi)

                                elif x_final_point == 0:
                                    psi_angle = 0

                                elif x_final_point > 0:
                                    pass

                                #! ------
                                
                                psi_d.publish(psi_angle*(180/math.pi)-40)  #! 너무 가까워서 각을 더 줌

                                danger1_points = Marker()
                                danger1_points.header.frame_id = "body"
                                danger1_points.ns = "danger"
                                danger1_points.id = 4
                                danger1_points.type = Marker.LINE_STRIP
                                danger1_points.action = Marker.ADD
                                danger1_points.color = ColorRGBA(1, 1, 0, 1) #! 우측으로 향할 때
                                danger1_points.scale = Vector3(0.1, 0.1, 0)
                                danger1_points.points = []
                                danger1_points.points.append(Point(0,0,0))
                                danger1_points.points.append(Point(x_final_point, y_final_point, 0))
                                danger1_points.pose.orientation.w = 1.0
                                danger1_points.lifetime = rospy.Duration.from_sec(0.1)
                                danger1_pub.publish(danger1_points)

                            else:  #! 우측이 더 위험지역

                                x_final_point = object_group[-1][-1][0]
                                y_final_point = object_group[-1][-1][1]
                                
                                psi_angle = atan(y_final_point/x_final_point)
                                
                                if x_final_point <= 0:

                                    if y_final_point < 0:
                                        psi_angle = (psi_angle - math.pi)

                                    elif y_final_point > 0:
                                        psi_angle = (psi_angle + math.pi)

                                elif x_final_point == 0:
                                    psi_angle = 0

                                elif x_final_point > 0:
                                    pass

                                #! ------
                                
                                psi_d.publish(psi_angle*(180/math.pi)+40)  #! 너무 가까워서 각을 더 줌
                                
                                # danger2_points = Marker()
                                # danger2_points.header.frame_id = "body"
                                # danger2_points.ns = "danger"
                                # danger2_points.id = 5
                                # danger2_points.type = Marker.LINE_STRIP
                                # danger2_points.action = Marker.ADD
                                # danger2_points.color = ColorRGBA(1, 0, 0, 1) #? 좌측으로 향할 때 
                                # danger2_points.scale = Vector3(0.1, 0.1, 0)
                                # danger2_points.points = []
                                # danger1_points.points.append(Point(0,0,0))
                                # danger2_points.points.append(Point(x_final_point, y_final_point, 0))
                                # danger2_points.pose.orientation.w = 1.0
                                # danger2_points.lifetime = rospy.Duration.from_sec(0.1)
                                # danger2_pub.publish(danger2_points)
                            
                        else:

                            final = min(final_point)
                            co_point = Point()
                            co_point.x = final[1][0]
                            co_point.y = final[1][1]
                            co_point.z = 0.0  
                            marker_co.points = []
                            marker_co.points.append(me)
                            marker_co.points.append(co_point)
                            marker_co.lifetime = rospy.Duration.from_sec(0.1)
                            markerArray_co.markers = []
                            markerArray_co.markers.append(marker_co)
                            co += 1
                            co_pub.publish(markerArray_co)


                            psi_angle = atan(final[1][1]/final[1][0])  #! atan change 

                            if final[1][0] <= 0:
                                if final[1][1] < 0:
                                    psi_angle = (psi_angle - math.pi)
                                elif final[1][1] > 0:
                                    psi_angle = (psi_angle + math.pi)
                            elif final[1][0] == 0:
                                psi_angle = 0
                            elif final[1][0] > 0:
                                pass
                            #! ------

                            # print(psi_angle*(180/math.pi),"target in auto")

                            if psi_angle > math.pi:
                                psi_angle = psi_angle - (2*math.pi)
                            else: 
                                pass
                            
                            if psi_angle*(180/math.pi) > 0:

                                psi_d.publish(psi_angle*(180/math.pi)+40) #! 너무 가까울 때 좌현으로 피할 때
                            elif psi_angle*(180/math.pi) == 0:

                                psi_d.publish(1000)
                            else:
                                psi_d.publish(psi_angle*(180/math.pi)-40) #! 너무 가까울 때 우현으로 피할 때

                elif  count <= 5:
                    
                    # print("safe")
                    rviz_points = Marker()
                    rviz_points.header.frame_id = "body"
                    rviz_points.ns = "possible"
                    rviz_points.id = 1
                    rviz_points.type = Marker.LINE_STRIP
                    rviz_points.action = Marker.ADD
                    rviz_points.color = ColorRGBA(1, 1, 0, 1)
                    rviz_points.scale = Vector3(0.1, 0.1, 0)
                    rviz_points.points = []
                    rviz_points.points.append(Point(0, 0, 0))
                    rviz_points.points.append(Point(2.5,0, 0))
                    # rviz_points.points.append(Point(trans_x*0.5, trans_y*0.5, 0))
                    rviz_points.pose.orientation.w = 1.0
                    rviz_points.lifetime = rospy.Duration.from_sec(0.1)
                    possible_pub.publish(rviz_points)
                    
                    psi_d.publish(psi_angle*(180/math.pi))
                    #psi_d.publish(0)
                    #print(psi_angle*(180/math.pi),atan(trans_y/trans_x)*(180/math.pi),"safeeee")
                    #print((math.pi/2-yaw)*(180/math.pi),"yaww")

                else:
                    # print("meet object")
                    markerArray_co = MarkerArray()
                    marker_co = Marker()
                    marker_co.header.frame_id = "body"
                    marker_co.ns = 'correct'
                    marker_co.type = Marker.LINE_STRIP
                    marker_co.action = Marker.ADD
                    marker_co.id = co
                    marker_co.scale.x = 0.1
                    marker_co.scale.y = 0.1
                    marker_co.color.a = 1.0
                    marker_co.color.r = 1.0
                    marker_co.color.g = 1.0
                    marker_co.color.b = 0.0 
                    marker_co.pose.orientation.w = 1.0
                    me = Point()  #? Arrow start point
                    me.x = 0
                    me.y = 0
                    me.z = 0
                    final_point = []
                        
                    if len(object_group) == 0:
                        
                        psi_d.publish(1000)


                    else: #! 장애물이 두개 이상일 때 

                        for correct_point in correct:

                            final_distance = math.sqrt((correct_point[0]-trans_x)**2 + (correct_point[1]-trans_y)**2)
                            final_goal = [final_distance,correct_point]
                            final_point.append(final_goal)

                        if len(final_point) == 0: #! 두개 이상인데 순간적으로 갈 수 있는 점을 찾지 못했을 때 선수 축 기준으로 len(points) 비교 해서 더 적은 쪽으로 

                            left_count = 0
                            right_count = 0

                            for point in points:

                                if point[1] > 0:
                                    left_count += 1
                                else: #? point[1] < 0:
                                    right_count += 1
                            if left_count > right_count:  #! 좌측이 더 위험 지역 (장애물 첫점에서 일정 각도 뺌)
                                
                                x_final_point = object_group[0][0][0]
                                y_final_point = object_group[0][0][1]
                                
                                psi_angle = atan(y_final_point/x_final_point)
                                
                                if x_final_point <= 0:

                                    if y_final_point < 0:
                                        psi_angle = (psi_angle - math.pi)

                                    elif y_final_point > 0:
                                        psi_angle = (psi_angle + math.pi)

                                elif x_final_point == 0:
                                    psi_angle = 0

                                elif x_final_point > 0:
                                    pass

                                #! ------
                                
                                psi_d.publish(psi_angle*(180/math.pi)-40)

                                danger1_points = Marker()
                                danger1_points.header.frame_id = "body"
                                danger1_points.ns = "danger"
                                danger1_points.id = 4
                                danger1_points.type = Marker.LINE_STRIP
                                danger1_points.action = Marker.ADD
                                danger1_points.color = ColorRGBA(1, 0, 0, 1) #! 우측으로 향할 때
                                danger1_points.scale = Vector3(0.05, 0.05, 0)
                                danger1_points.points = []
                                danger1_points.points.append(Point(0, 0, 0))
                                danger1_points.points.append(Point(x_final_point, y_final_point, 0))
                                danger1_points.pose.orientation.w = 1.0
                                danger1_points.lifetime = rospy.Duration.from_sec(0.1)
                                danger1_pub.publish(danger1_points)

                            else:  #! 우측이 더 위험지역

                                x_final_point = object_group[-1][-1][0]
                                y_final_point = object_group[-1][-1][1]
                                
                                psi_angle = atan(y_final_point/x_final_point)
                                
                                if x_final_point <= 0:

                                    if y_final_point < 0:
                                        psi_angle = (psi_angle - math.pi)

                                    elif y_final_point > 0:
                                        psi_angle = (psi_angle + math.pi)

                                elif x_final_point == 0:
                                    psi_angle = 0

                                elif x_final_point > 0:
                                    pass

                                #! ------
                                
                                psi_d.publish(psi_angle*(180/math.pi)+40) 
                                
                                danger2_points = Marker()
                                danger2_points.header.frame_id = "body"
                                danger2_points.ns = "danger"
                                danger2_points.id = 5
                                danger2_points.type = Marker.LINE_STRIP
                                danger2_points.action = Marker.ADD
                                danger2_points.color = ColorRGBA(0, 0, 1, 1) #? 좌측으로 향할 때 
                                danger2_points.scale = Vector3(0.05, 0.05, 0)
                                danger2_points.points = []
                                danger2_points.points.append(Point(0, 0, 0))
                                danger2_points.points.append(Point(x_final_point, y_final_point, 0))
                                danger2_points.pose.orientation.w = 1.0
                                danger2_points.lifetime = rospy.Duration.from_sec(0.1)
                                danger2_pub.publish(danger2_points)
                            
                        else:

                            final = min(final_point)
                            co_point = Point()
                            co_point.x = final[1][0]
                            co_point.y = final[1][1]
                            co_point.z = 0.0  
                            marker_co.points = []
                            marker_co.points.append(me)
                            marker_co.points.append(co_point)
                            marker_co.lifetime = rospy.Duration.from_sec(0.1)
                            markerArray_co.markers = []
                            markerArray_co.markers.append(marker_co)
                            co += 1
                            co_pub.publish(markerArray_co)


                            psi_angle = atan(final[1][1]/final[1][0])  #! atan change 

                            if final[1][0] <= 0:
                                if final[1][1] < 0:
                                    psi_angle = (psi_angle - math.pi)
                                elif final[1][1] > 0:
                                    psi_angle = (psi_angle + math.pi)
                            elif final[1][0] == 0:
                                psi_angle = 0
                            elif final[1][0] > 0:
                                pass
                            #! ------

                            # print(psi_angle*(180/math.pi),"target in auto")

                            if psi_angle > math.pi:
                                psi_angle = psi_angle - (2*math.pi)
                            else: 
                                pass

                            psi_d.publish(psi_angle*(180/math.pi))
                            #print(target_angle*(180/math.pi),"target")
                            #print(psi_angle*(180/math.pi),"psi_d")
            
        else: #! Docking Mode --------------------------------------------------------------------------------------------------------------------------------
            now_mode.publish(2)
            if find_1 == False:

                if docking_angle == 1000:
                    pass
                else:
                    find_1 = True

                    if docking_angle > 0:
                        right_left = True

                    else:
                        right_left = False
                        


            if find_1 == False:

                count=0
                co = 0
                close = 0
                for object in object_group:
                    # if len(object) < 10: #! 버리는 경우 취소
                    #     pass
                    # else:
                    for point in object:
                        
                        if (-1 <= point[0] <= -0.5) and (-1 <= point[1] <= -0.2):  
                            close+=0  #! 실험을 위해서 안전하게 만드려고 1더하는거 없앰
                        if (0.5 <= point[0] <= 1.0) and (0.2 <= point[1] <= 1):
                            close+=0
                        else:
                            pass

                # print(close)

                if close > 0: #! meet object
                    # print("close object_docking mode")
                    markerArray_co = MarkerArray()
                    marker_co = Marker()
                    marker_co.header.frame_id = "body"
                    marker_co.ns = 'correct'
                    marker_co.type = Marker.ARROW
                    marker_co.action = Marker.ADD
                    marker_co.id = co
                    marker_co.scale.x = 0.1
                    marker_co.scale.y = 0.2
                    marker_co.color.a = 1.0
                    marker_co.color.r = 1.0
                    marker_co.color.g = 1.0
                    marker_co.color.b = 0.0 
                    marker_co.pose.orientation.w = 1.0
                    me = Point()  #? Arrow start point
                    me.x = 0
                    me.y = 0
                    me.z = 0
                    final_point = []

                    if len(object_group) == 0:
                        psi_d.publish(1000)
                    else: #! 장애물이 두개 이상일 때 
                        for correct_point in correct:
                            final_distance = math.sqrt((correct_point[0]-2)**2 + (correct_point[1]-0)**2) #! (1,0) 과 가장 가까운 점
                            final_goal = [final_distance,correct_point]
                            final_point.append(final_goal)
                        if len(final_point) == 0: #! 두개 이상인데 순간적으로 갈 수 있는 점을 찾지 못했을 때 선수 축 기준으로 len(points) 비교 해서 더 적은 쪽으로 
                            left_count = 0
                            right_count = 0
                            for point in points:
                                if point[1] > 0:
                                    left_count += 1
                                else: #? point[1] < 0:
                                    right_count += 1
                            if left_count > right_count:  #! 좌측이 더 위험 지역 (장애물 첫점에서 일정 각도 뺌)
                                x_final_point = object_group[0][0][0]
                                y_final_point = object_group[0][0][1]
                                psi_angle = atan(y_final_point/x_final_point)
                                if x_final_point <= 0:
                                    if y_final_point < 0:
                                        psi_angle = (psi_angle - math.pi)
                                    elif y_final_point > 0:
                                        psi_angle = (psi_angle + math.pi)
                                elif x_final_point == 0:
                                    psi_angle = 0
                                elif x_final_point > 0:
                                    pass
                                #! ------
                                psi_d.publish(psi_angle*(180/math.pi)-20)  #! 너무 가까워서 각을 더 줌
                                danger1_points = Marker()
                                danger1_points.header.frame_id = "os_sensor"
                                danger1_points.ns = "danger"
                                danger1_points.id = 4
                                danger1_points.type = Marker.POINTS
                                danger1_points.action = Marker.ADD
                                danger1_points.color = ColorRGBA(1, 0, 0, 1) #! 우측으로 향할 때
                                danger1_points.scale = Vector3(0.5, 0.5, 0)
                                danger1_points.points = []
                                danger1_points.points.append(Point(x_final_point, y_final_point, 0))
                                danger1_points.pose.orientation.w = 1.0
                                danger1_points.lifetime = rospy.Duration.from_sec(0.1)
                                danger1_pub.publish(danger1_points)
                            else:  #! 우측이 더 위험지역
                                x_final_point = object_group[-1][-1][0]
                                y_final_point = object_group[-1][-1][1]
                                psi_angle = atan(y_final_point/x_final_point)
                                if x_final_point <= 0:
                                    if y_final_point < 0:
                                        psi_angle = (psi_angle - math.pi)
                                    elif y_final_point > 0:
                                        psi_angle = (psi_angle + math.pi)
                                elif x_final_point == 0:
                                    psi_angle = 0
                                elif x_final_point > 0:
                                    pass
                                #! ------
                                psi_d.publish(psi_angle*(180/math.pi)+20)  #! 너무 가까워서 각을 더 줌
                                danger2_points = Marker()
                                danger2_points.header.frame_id = "os_sensor"
                                danger2_points.ns = "danger"
                                danger2_points.id = 5
                                danger2_points.type = Marker.POINTS
                                danger2_points.action = Marker.ADD
                                danger2_points.color = ColorRGBA(0, 0, 1, 1) #? 좌측으로 향할 때 
                                danger2_points.scale = Vector3(0.5, 0.5, 0)
                                danger2_points.points = []
                                danger2_points.points.append(Point(x_final_point, y_final_point, 0))
                                danger2_points.pose.orientation.w = 1.0
                                danger2_points.lifetime = rospy.Duration.from_sec(0.1)
                                danger2_pub.publish(danger2_points)
                        else:
                            final = min(final_point)
                            co_point = Point()
                            co_point.x = final[1][0]
                            co_point.y = final[1][1]
                            co_point.z = 0.0  
                            marker_co.points = []
                            marker_co.points.append(me)
                            marker_co.points.append(co_point)
                            marker_co.lifetime = rospy.Duration.from_sec(0.1)
                            markerArray_co.markers = []
                            markerArray_co.markers.append(marker_co)
                            co += 1
                            co_pub.publish(markerArray_co)
                            psi_angle = atan(final[1][1]/final[1][0])  #! atan change 
                            if final[1][0] <= 0:
                                if final[1][1] < 0:
                                    psi_angle = (psi_angle - math.pi)
                                elif final[1][1] > 0:
                                    psi_angle = (psi_angle + math.pi)
                            elif final[1][0] == 0:
                                psi_angle = 0
                            elif final[1][0] > 0:
                                pass
                            #! ------
                            print(psi_angle*(180/math.pi),"target in auto_docking mode")
                            if psi_angle > math.pi:
                                psi_angle = psi_angle - (2*math.pi)
                            else: 
                                pass
                            
                            if psi_angle*(180/math.pi) > 0:
                                psi_d.publish(psi_angle*(180/math.pi)) #! 너무 가까울 때 좌현으로 피할 때
                            elif psi_angle*(180/math.pi) == 0:
                                psi_d.publish(1000)
                            else:
                                psi_d.publish(psi_angle*(180/math.pi)) #! 너무 가까울 때 우현으로 피할 때

                else: #! safe -> Probe mode

                    if F_yaw - 10 < yaw < F_yaw+10 : #! 정확하게 선수가 못 향하니까 오차값 허용
                        
                        print('Find_mode_docking mode') # FIND COLOR
                        psi_d.publish(1030)
                        #print("turn left slowly")
                        # time.sleep(1)
                        #psi_d.publish(1000)
                        if docking_angle == 1000:
                            psi_d.publish(1030)
                            #print("turn left slowly ")
                            # time.sleep(1)
                            #psi_d.publish(1000)
                            time.sleep(1.2)
                            if docking_angle == 1000:
                                psi_d.publish(1030)
                                time.sleep(1.2)
                                if docking_angle == 1000:
                                    psi_d.publish(1000)
                                    time.sleep(1)
                        else: 
                            print("find color_docking mode")
                        
                        if docking_angle == 1000:
                            psi_d.publish(1060)
                            if docking_angle == 1000:
                                psi_d.publish(1060)
                                time.sleep(1.2)
                                if docking_angle == 1000:
                                    psi_d.publish(1060)
                                    time.sleep(1.2)
                                    psi_d.publish(1000)
                                    time.sleep(1)
                                    if docking_angle == 1000:
                                        psi_d.publish(1060)
                                        time.sleep(1)
                                        if docking_angle == 1000:
                                            psi_d.publish(1000)
                                            time.sleep(1)
                                    
                            else:
                                print("find color_docking mode")
                        
                        if docking_angle == 1000:
                            psi_d.publish(1030)
                            time.sleep(1.2)
                            psi_d.publish(1000)
                            time.sleep(1)
                            
                        else: 
                            print("find color_docking mode")

            else: #! find_1 => True -> Catch docking_point more than one
                
                if docking_angle == 1000: #! missing docking_point
                    
                    print('light_Find_mode_docking mode') # FIND COLOR
                    
                    psi_d.publish(1000)
                    time.sleep(1)
                    psi_d.publish(1030) #! turn left slowly

                    if docking_angle == 1000:
                        psi_d.publish(1030)
                        time.sleep(0.5)
                        if docking_angle == 1000:
                            psi_d.publish(1000)
                            time.sleep(0.5)
                    else: 
                        print("find color_docking mode")
                    
                    if docking_angle == 1000:
                        psi_d.publish(1060) #! turn right slowly
                        if docking_angle == 1000:
                            psi_d.publish(1060)
                            time.sleep(0.5)
                            if docking_angle == 1000:
                                psi_d.publish(1000)
                                time.sleep(0.5)
                        else:
                            print("find color_docking mode")
                    
                    if docking_angle == 1000:
                        psi_d.publish(1030)
                        time.sleep(0.5)
                        if docking_angle == 1000:
                            psi_d.publish(1000)
                            time.sleep(0.5)
                        
                    else: 
                        print("find color_docking mode")

                else: #! Find docking_point

                    
                    
                    co = 0
                    close = 0
                    finish_point = 0
                    for object in object_group:
                        # if len(object) < 10: #! 버리는 경우 취소
                        #     pass
                        # else:
                        for point in object:

                            # if (-1 <= point[0] <= -0.5) and (-1 <= point[1] <= -0.2):  
                            #     close+=1  #! 실험을 위해서 안전하게 만드려고 1더하는거 없앰
                            if (0.3 <= point[0] <= 2.0) and (-0.5 <= point[1] <= 0.5):
                                close+=1
                            
                            if (0.2 < point[0] < 0.3) and (-0.2 < point[1]< 0.2):
                                finish_point +=1
                            else:
                                pass
                    
                    if finish_point > 0: #! Finish
                        psi_d.publish(10000)
                        time.sleep(10)
                        sys.exit()
                    
                    elif close > 0: #! danger
                        final_point = []
                        print("danger")
                        for correct_point in correct:
                            
                            correct_angle = atan(correct_point[1]/correct_point[0])
                            if correct_point[0] <= 0:
                                
                                if correct_point[1] < 0:
                                    correct_angle = (psi_angle - math.pi)
                                    
                                elif correct_point[1] > 0:
                                    correct_angle = (psi_angle + math.pi)
                                    
                            elif correct_point[0] == 0:
                                correct_angle = 0
                                
                            elif correct_point[0] > 0:
                                pass

                        if docking_angle == 0:
                            if right_left == False: #! 처음에 본 각이 오른쪽
                                moving_angle = docking_angle - 20
                                psi_d.publish(moving_angle)
                            else: 
                                moving_angle = docking_angle + 20
                                psi_d.publish(moving_angle)

                        if docking_angle > 0:
                            moving_angle = docking_angle + 20

                            psi_d.publish(moving_angle)

                        elif docking_angle < 0:
                            moving_angle = docking_angle - 20

                            psi_d.publish(moving_angle)

                        #     final_angle = abs(docking_angle - correct_angle)
                        #     final_goal = [final_angle,correct_point]
                        #     final_point.append(final_goal)
                        
                        # if len(final_point) == 0:
                        #     pass
                        # else:

                        #     final = min(final_point)
                        
                        #     docking_point = Marker()
                        #     docking_point.header.frame_id = "laser_frame"
                        #     docking_point.id = 19
                        #     docking_point.type = Marker.POINTS
                        #     docking_point.action = Marker.ADD
                        #     docking_point.color = ColorRGBA(1, 1, 0, 1)
                        #     docking_point1 = Point()
                        #     docking_point1.x = final[1][0]
                        #     docking_point1.y = final[1][1]
                        #     docking_point1.z = 0.0  
                        #     docking_point.points = []
                        #     docking_point.scale = Vector3(0.1, 0.2, 0)
                        #     docking_point.points.append(docking_point1)
                        #     docking_point.pose.orientation.w = 1.0
                        #     docking_point.lifetime = rospy.Duration.from_sec(0.1)
                        #     pub_docking_point.publish(docking_point)

                        #     psi_angle = atan(final[1][1]/final[1][0])
                        #     psi_d.publish(psi_angle*(180/math.pi))
                        #     print(psi_angle,"moving to docking point")

                    else: #! safe

                        print(docking_angle,"docking_angle_Now safe")
                        minus_list = []
                        
                        if docking_angle >= 0:

                            if (docking_angle) >= 8:
                                psi_d.publish(docking_angle+15) #! 예스 보정

                            elif 0 <= docking_angle < 8:
                                psi_d.publish(docking_angle)

                        if docking_angle <= 0:

                            if (docking_angle) <= -8:
                                psi_d.publish(docking_angle-15) #! 예스 보정

                            elif -8 < docking_angle <= 0:
                                psi_d.publish(docking_angle)
    #!------------------------------------------------------- Find docking_point
                        for point in all_points:

                            if point[0] <= 0 or point[1] == 0:
                                pass
                            else:

                                find_angle = atan(point[1]/point[0])  #! atan change 

                                if point[0] <= 0:
                                    if point[1] < 0:
                                        find_angle = (find_angle - math.pi)
                                    elif point[1] > 0:
                                        find_angle = (find_angle + math.pi)
                                elif point[0] == 0:
                                    find_angle = 0
                                elif point[0] > 0:
                                    pass
                                #! ------

                                minus = abs((find_angle*(180/math.pi)) - (docking_angle))

                                minus_point = [minus,point]

                                if minus > 2:  #! 차이 너무 크면 필요도 없어서 오차 줄이기 위해 서로의 각도 차이는 2도 미만
                                    pass

                                else:
                                    minus_list.append(minus_point)
                        

                        if len(minus_list) == 0:
                            
                            #! 이거 범위 내에 라이다가 인식 못할 때, 강제로 포인트 만들어줌.
                            docking_x = (4*cos(docking_angle*(math.pi/180))) 
                            docking_y = (4*sin(docking_angle*(math.pi/180)))



                            dock_points = Marker()
                            dock_points.header.frame_id = "os_sensor"
                            dock_points.ns = "dock"
                            dock_points.id = 20
                            dock_points.type = Marker.POINTS
                            dock_points.action = Marker.ADD
                            dock_points.color = ColorRGBA(0, 1, 0, 1)
                            dock_points.scale = Vector3(0.3, 0.3, 0)
                            dock_points.points = []
                            dock_points.points.append(Point(docking_x, docking_y, 0))
                            dock_points.lifetime = rospy.Duration.from_sec(0.1)
                            dock_pub.publish(dock_points)

                        else: 
                            #? docking_point 는 minus_list 속에서 minus 가 제일 작은 mius_point 값
                            docking_point = min(minus_list)
                            docking_x = docking_point[1][0]
                            docking_y = docking_point[1][1]

                            print(docking_x,docking_y,"docking_point")

                            if docking_x == 0:
                                pass
                            else:
                                print(atan(docking_y/docking_x)*(180/math.pi),"docking_angle_Now safe")
                                print(docking_angle,"oringin")

                            dock_points = Marker()
                            dock_points.header.frame_id = "os_sensor"
                            dock_points.ns = "dock"
                            dock_points.id = 20
                            dock_points.type = Marker.POINTS
                            dock_points.action = Marker.ADD
                            dock_points.color = ColorRGBA(0, 1, 0, 1)
                            dock_points.scale = Vector3(0.3, 0.3, 0)
                            dock_points.points = []
                            dock_points.points.append(Point(docking_x, docking_y, 0))
                            dock_points.lifetime = rospy.Duration.from_sec(0.1)
                            dock_pub.publish(dock_points) 
                            dock_pub.publish(dock_points)



if __name__ == '__main__':

    # try:
    #     autonomous()
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass
    autonomous()
    

    
    
    