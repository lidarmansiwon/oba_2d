#!/usr/bin/env python3
#-*- coding:utf-8 -*-
'''
File: PD
Author: 김시원 (Si Won Kim)
Date: 2022-08-01
Version: v1.0
Description: 해당 노드는 최종 error 값을 전달 받아 PID제어를 통하여 boat의 PWM을 제어한다. 
''' 

#from statistics import mode
import rospy
from std_msgs.msg import String, Float64, Float32MultiArray
import random
import time
import sys
#from microstrain_inertial_msgs.msg import FilterHeading


rospy.init_node("PDcontrol_node", anonymous=True)

mode_Auto_Dock = 1
last_error = 0
port_pwm = 0
stbd_pwm = 0 
error = 0
now = False
#! ------- 
RC_Auto = 1
get_port_pwm = 1500
get_stbd_pwm = 1500
#! --------

def RC(data):
    global RC_Auto
    RC_Auto = data.data  #! 0 => RC / 1 => Auto 
    
def sub_port_pwm(data):  #! get port_pwm from ARDUINO
    global get_port_pwm
    get_port_pwm = data.data

def sub_stbd_pwm(data):  #! get stbd_pwm from ARDUINO
    global get_stbd_pwm
    get_stbd_pwm = data.data

def Auto_or_Docking(data):
    global mode_Auto_Dock, now
    mode_Auto_Dock = data.data
    if mode_Auto_Dock == 1.0:
        now = True
    else:
        now = False
        pass

    print(mode_Auto_Dock)


def Autonomous_callback(data):
    global error
    error= data.data
    return(error)


def pid_control(error): 
    global last_error,port_pwm,stbd_pwm
    
    
    if now == True:  #! 자율 운항 모드 

        Kp,Ki,Kd = 2,0,1  #! 실험 때문에 4에서 5로 올림   
        ITerm = 0
        # current_time = current_time if current_time is not None else time.time()
        # delta_time = current_time - last_time
        delta_time = 0.0471
        delta_error = error - last_error

        # if (delta_time >= sample_time):
        PTerm = Kp * error
        ITerm += error * delta_time
        DTerm = (error-last_error)/delta_time
        last_error = error

        PID = PTerm + (Ki *0) + (Kd * DTerm)
        output = int(PID)

        return output
    else:
        
        # Kp,Ki,Kd = 3,0.1,2   #! 도킹 모드 False
        Kp,Ki,Kd = 1.8,0,0  
        # error = heading - camera_angle    
        ITerm = 0
        # current_time = current_time if current_time is not None else time.time()
        # delta_time = current_time - last_time
        delta_time = 0.0471
        delta_error = error - last_error

        # if (delta_time >= sample_time):
        PTerm = Kp * error
        ITerm += error * delta_time
        DTerm = (error-last_error)/delta_time
        last_error = error

        PID = PTerm + (Ki *0) + (Kd * DTerm)
        output = int(PID)

        return output 

def control_all():
    global pwm_output,foward_pwm,mode_Auto_Dock,now,pub_port_pwm,pub_stbd_pwm,Auto_pt_pwm,Auto_st_pwm
    # print(error,"error")

    Auto_pt_pwm = rospy.Publisher('pt_pwm', Float64, queue_size=1)  #! Auto 
    Auto_st_pwm = rospy.Publisher('st_pwm', Float64, queue_size=1)  #! Auto 
    # RC_port_pwm = rospy.Publisher('pt_pwm', Float64, queue_size=1)  #! RC return 
    # RC_stbd_pwm = rospy.Publisher('st_pwm',Float64,queue_size=1)   #! RC return
    pubCmdPWM = rospy.Publisher('/ctrl/pwm', Float32MultiArray, queue_size=10)
    
    stop_pwm = 1500
    # foward_pwm = 1630   #! 1620 전부 20씩 추가함 실험 때문에 
    # foward_pwm2 = 1630
    # foward_pwm3 = 1590  
    # foward_pwm4 = 1580
    # foward_pwm5 = 1570
    # foward_pwm6 = 1560
    # foward_pwm_dock = 1550
    foward_pwm = 1540   
    foward_pwm2 = 1540
    foward_pwm3 = 1530  #! 원래 1570
    foward_pwm4 = 1530
    foward_pwm5 = 1530
    foward_pwm6 = 1530
    foward_pwm7 = 1500
    foward_pwm_dock = 1550

            
    pwm_output = pid_control(error)


    if RC_Auto == 0: #! Now mode is RC
        print("RC_mode")
        # RC_port_pwm.publish(get_port_pwm)  #! Return to ARDUINO
        # RC_stbd_pwm.publish(get_stbd_pwm)

    else: #! Now mode is Auto ------------------------------------------------------------------------
        # print('Not_RC_mode')
        # print(now,"Auto = True / Docking = False")

        if now == True :  #! mode_Auto 
            
            if  0 <= abs(error) <= 10:
                port_pwm = foward_pwm - pwm_output
                stbd_pwm = foward_pwm + pwm_output
            elif 10 <=abs(error) <= 20:
                port_pwm = foward_pwm2 - pwm_output
                stbd_pwm = foward_pwm2 + pwm_output
            elif 20 <=abs(error) <= 30:
                port_pwm = foward_pwm3 - pwm_output
                stbd_pwm = foward_pwm3 + pwm_output
            elif 30 < abs(error) <= 50:
                port_pwm = foward_pwm4 - pwm_output
                stbd_pwm = foward_pwm4 + pwm_output
            elif 50 < abs(error) < 60:
                port_pwm = foward_pwm5 - pwm_output
                stbd_pwm = foward_pwm5 + pwm_output
            elif 60 < abs(error) < 110:
                port_pwm = foward_pwm6 - pwm_output
                stbd_pwm = foward_pwm6 + pwm_output

            # elif 110 <= abs(error) <= 180:
            #     port_pwm = 1370
            #     stbd_pwm = 1370
            elif 110 <= abs(error) <= 180:
                port_pwm = foward_pwm7 - (0.7*(pwm_output))
                stbd_pwm = foward_pwm7 + (0.7*(pwm_output))


            elif abs(error) ==1000:
                port_pwm = 1500
                stbd_pwm = 1500
            elif abs(error) == 1030: #! turn left slowly
                port_pwm = 1400  #! 1350
                stbd_pwm = 1550  #! 1600
            elif abs(error) == 1060: #! turn right slowly
                port_pwm = 1550
                stbd_pwm = 1400
            elif abs(error) == 1010: #! for docking _find
                port_pwm = 1550
                stbd_pwm = 1550
            elif abs(error) == 1020: #! back 
                port_pwm = 1400
                stbd_pwm = 1400
                
            elif abs(error) == 2010:
                port_pwm = 1650
                stbd_pwm = 1350
                
            elif abs(error) == 2000:  #! 좌회전 끝 부분
                port_pwm = 1350
                stbd_pwm = 1650
                print("aaaaaaaaaaaaaaaaaa")
        
            elif abs(error) == 10000:
                port_pwm = 1500
                stbd_pwm = 1500

            else :
                port_pwm = 1500
                stbd_pwm = 1500


            # Auto_pt_pwm.publish(port_pwm)
            # Auto_st_pwm.publish(stbd_pwm)
            if port_pwm > 1700:
                port_pwm = 1700
            if stbd_pwm > 1700: 
                stbd_pwm = 1700
            if port_pwm < 1300:
                port_pwm = 1300
            if stbd_pwm < 1300:
                stbd_pwm = 1300

            thrPWM = [port_pwm, stbd_pwm,1500,1500]
            pwmMsg = Float32MultiArray()
            pwmMsg.data = thrPWM 
            pubCmdPWM.publish(pwmMsg)    

        else: #! mode_docking -----------------------------------------------------------------------
            
            

            if  0 <= abs(error) <= 10:
                port_pwm = foward_pwm_dock - pwm_output
                stbd_pwm = foward_pwm_dock + pwm_output
            elif 10 < abs(error) <= 60:
                port_pwm = foward_pwm_dock - pwm_output
                stbd_pwm = foward_pwm_dock + pwm_output
            elif 60 < abs(error) < 110:
                port_pwm = foward_pwm_dock - pwm_output
                stbd_pwm = foward_pwm_dock + pwm_output

            elif 110 <= abs(error) <= 180:
                port_pwm = 1340
                stbd_pwm = 1340
            elif abs(error) ==1000:
                port_pwm = 1500
                stbd_pwm = 1500
            elif abs(error) == 1030: #! turn left slowly
                port_pwm = 1350
                stbd_pwm = 1600
            elif abs(error) == 1060: #! turn right slowly
                port_pwm = 1600
                stbd_pwm = 1350
            elif abs(error) == 1010: #! for docking _find
                port_pwm = 1550
                stbd_pwm = 1550
            elif abs(error) == 1020: #! back 
                port_pwm = 1350
                stbd_pwm = 1350


            else :
                port_pwm = 1500
                stbd_pwm = 1500


            # Auto_pt_pwm.publish(port_pwm)
            # Auto_st_pwm.publish(stbd_pwm)   
            thrPWM = [port_pwm, stbd_pwm,1500,1500]
            pwmMsg = Float32MultiArray()
            pwmMsg.data = thrPWM 
            pubCmdPWM.publish(pwmMsg)             
            #print("PORT_PWM:{} STBD_PWM:{}, PWM_OUTPU:{}".format(port_pwm,stbd_pwm,pwm_output))  

            rate = rospy.Rate(100)
            rate.sleep()

        print("PORT_PWM:{} STBD_PWM:{}, PWM_OUTPU:{}".format(port_pwm,stbd_pwm,pwm_output)) 
    
def listener():
    rospy.Subscriber('/psi_d', Float64, Autonomous_callback)  #! get (error) from Autonomous
    rospy.Subscriber('/Auto_or_Docking', Float64, Auto_or_Docking)
    rospy.Subscriber('/RC_Auto', Float64, RC) #! get mode from ARDUINO
    # rospy.Subscriber('/port_pwm', Float32MultiArray, sub_port_pwm)  #! get (RC_pwm) from ARDUINO
    # rospy.Subscriber('/stbd_pwm', Float32MultiArray, sub_stbd_pwm)  #! get (RC_pwm) from ARDUINO

    while not rospy.is_shutdown():
        
        
        try:
            control_all()
        
                
        except KeyboardInterrupt:

            sys.exit()

            

if __name__ == '__main__':

    listener()
    