from util2 import *



def add(a, b):
    return a + b


def odomCallback(self, msg):
        self.odometryData = msg

        #* 보트의 상태 저장
        boatOrientation = eulerFromQuaternion(self.odometryData.pose.pose.orientation.x,self.odometryData.pose.pose.orientation.y,self.odometryData.pose.pose.orientation.z,self.odometryData.pose.pose.orientation.w)
        pose = [self.odometryData.pose.pose.position.x, self.odometryData.pose.pose.position.y, boatOrientation[2]] # [x, y, psi]
        vel  = [self.odometryData.twist.twist.linear.x, self.odometryData.twist.twist.linear.y, self.odometryData.twist.twist.angular.z] # [u, v, r]
        # psi = pose[2] * 180/np.pi
        # print(pose,"pose")
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