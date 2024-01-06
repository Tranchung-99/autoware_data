#!/usr/bin/env python3
# coding: UTF-8

from __future__ import print_function
from math import atan2, sqrt

import rospy
import pandas as pd
import serial
import struct
from std_msgs.msg import Int32MultiArray, Float32
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, PointCloud2

from tf import TransformBroadcaster
import tf
import serial
import numpy as np
import serial.tools.list_ports
import datetime
import csv
import time
import math
import copy
import ros_numpy
k = 0.0     # look forward gain 直進時に目標距離を速度によって変更
Lfc = 3.0   # [m] look-ahead 距離　デフォルト値
Kp = 0.1    # speed proportional gain
WB = 2.9    # [m] wheel base of vehicle

class PubPose:
    def __init__(self):
        usb_port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.ser = serial.Serial("/dev/ttyUSB0", 115200)                
        self.vel = [0.0]*3
        self.pose = [0.0]*3
        self.slow_down = 0.0   

        self.send_data = ['a']*5
        self.send_data[0] = 255
        self.raw_speed = [0, 0]

        self.auto_mode = False
        self.timer_auto = 0
        self.joy_count = 0
        self.pre_data = [0, 0]
        self.new_data = [0, 0]
        self.Lfc = 2.5   # [m] look-ahead 距離　デフォルト値
        self.Lfc_now = 2.5



        self.old_nearest_point_index = None
        self.ind = 0

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.current_v = 0.0

        self.obj = []
        self.obj_plan = []
        self.obj_plan_x = []
        self.obj_plan_y = []
        
        self.pose_initial_flag = 0  #poseを受信するまでメイン処理待機

        #障害物時　パス切り替え処理
        self.object_stop =0     #障害物が回避できない場合、停止する
        self.object_path_change = 0 #障害物時のパス変更有無
        self.object_path_change_id = []
        self.object_dis_short_id = -1

        self.object_last_id = 0
        self.old_sayuu  = [0,0,0]
        self.point_obj_clear_coutn = 0

        self.joy_auto_flag = 0
        
    def pose_now_callback(self,data):
        self.pose_initial_flag = 1
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        q = tf.transformations.euler_from_quaternion(
            (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
        self.yaw = q[2]

    #     print(self.x, self.y,self.yaw)
        end,stea = self.pure_pursuit_steer_control()
        
        if abs(math.degrees(stea))>20:  #ステアリング角度が大き場合
            target_speed = 0.3  #0.2
        elif abs(math.degrees(stea))>10:  #ステアリング角度が大き場合
            target_speed = 0.4#0.3
        else:
            target_speed = 0.6#0.4

        if self.object_stop ==1:    #障害物が避けられない場合　→　停止
            target_speed = 0

        if end == 1:
            target_speed = 0


    #     if self.object_path_change == 1:
    #         if self.object_dis_short_id == -1:
    #             self.object_dis_short_id = self.old_nearest_point_index	
    #             self.Lfc_now = copy.deepcopy(self.Lfc)
    #         elif abs(self.object_dis_short_id-self.old_nearest_point_index)>6:
    #             self.object_dis_short_id = -1
    #             self.Lfc_now = copy.deepcopy(self.Lfc)
    #     else:
    #         self.Lfc_now = copy.deepcopy(self.Lfc)
    #         self.object_dis_short_id = -1

    #     self.proportional_control(target_speed)
    #     self.vel[1] = self.current_v
    #     self.vel[2] = stea


    def proportional_control(self,target):
        #if self.joy_auto_flag == 0:
        #    self.current_v = 0
        if target < self.current_v:   #減速の場合
            a = Kp*2 * (target - self.current_v)
            self.current_v += a#[s]
            self.current_v = target
        else:
            a = Kp * (target - self.current_v)
            self.current_v += a#[s]
        


    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

    def search_target_index(self):
        ind = self.old_nearest_point_index
        distance_this_index = self.calc_distance(self.cx[ind],self.cy[ind])
            
        dx = self.x - self.cx
        dy = self.y - self.cy
        ind = np.argmin((dx**2+dy**2)**0.5)
            
        while True:
            distance_next_index = self.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
            if distance_this_index < distance_next_index:
                break
            ind = ind + 1 if (ind + 1) < len(self.cx) else ind
            distance_this_index = distance_next_index
        self.old_nearest_point_index = ind

        Lf = k * self.current_v + self.Lfc_now  # update look ahead distance

        # search look ahead target point index
        while Lf > self.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

    def pure_pursuit_steer_control(self):
        ind, Lf = self.search_target_index()

        # if self.ind >= ind:  # 消す？
        #     ind = self.ind
        
        id_dis = 6
        if self.old_nearest_point_index <len(self.cx)-id_dis:
            tx = self.cx[self.old_nearest_point_index]+(self.cx[self.old_nearest_point_index+id_dis]-self.cx[self.old_nearest_point_index])*3
            ty = self.cy[self.old_nearest_point_index]+(self.cy[self.old_nearest_point_index+id_dis]-self.cy[self.old_nearest_point_index])*3
        else:
            return 1,0

        # if ind < len(self.cx):
        #     tx = self.cx[ind]
        #     ty = self.cy[ind]
        # else:  # toward goal
        #     tx = self.cx[-1]
        #     ty = self.cy[-1]
        #     ind = len(self.cx) - 1

        alpha = math.atan2(ty - self.y, tx - self.x) - self.yaw
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
        print("delta",math.degrees(delta))
        self.ind = ind
        return 0,delta


    def closest_waypoint_index_callback(self, data):
        self.old_nearest_point_index = data.data



    def points_raw_callback(self,data):
        pc = ros_numpy.numpify(data)
        points = np.zeros((pc.shape[0], 3))
        points[:, 0] = pc['x']
        points[:, 1] = pc['y']
        points[:, 2] = pc['z']

        points_tmp = points[points[:,2]>-0.8,:]  #地面を除く
        points_range = points_tmp[points_tmp[:, 2] < 0.5, :]  #天井を除く
        # print(points.shape ,points_range.shape)
        obj = []

        path_dis = 0.4

        obj_plan = []
        obj_plan_x = []
        obj_plan_y = []
        # return 0
        r_list  =[]
        for i in range(self.old_nearest_point_index+2, self.old_nearest_point_index+16):
            if len(self.cx) <i+2:
                break

            cal_x = (self.cx_o[i]-self.x)*math.cos(-self.yaw) - (self.cy_o[i]-self.y)*math.sin(-self.yaw)
            cal_y = (self.cx_o[i]-self.x)*math.sin(-self.yaw) + (self.cy_o[i]-self.y)*math.cos(-self.yaw)
            r = ((points_range[:, 0]-cal_x)**2+(points_range[:, 1]-cal_y)**2)**0.5
            
            r_list.append(int(np.min(r)*10)/10.0)
            

            #障害物があってもいいように、回避経路を左右に作っておく
            r_path_now = ((self.cy_o[i+1]-self.cy_o[i])**2 + (self.cx_o[i+1]-self.cx_o[i])**2)**0.5
            plan_1 = [-(self.cy_o[i+1]-self.cy_o[i]),(self.cx_o[i+1]-self.cx_o[i])]/r_path_now*0.8
            plan_2 = [(self.cy_o[i+1]-self.cy_o[i]),-(self.cx_o[i+1]-self.cx_o[i])]/r_path_now*0.8
            plan_1[0] += self.cx_o[i]
            plan_1[1] += self.cy_o[i]
            plan_2[0] += self.cx_o[i]
            plan_2[1] += self.cy_o[i]
            
            # print(self.cx[i],self.cy[i],plan_1,plan_2)

            cal_x = (plan_1[0]-self.x)*math.cos(-self.yaw) - (plan_1[1]-self.y)*math.sin(-self.yaw)
            cal_y = (plan_1[0]-self.x)*math.sin(-self.yaw) + (plan_1[1]-self.y)*math.cos(-self.yaw)
            r1 = ((points_range[:, 0]-cal_x)**2+(points_range[:, 1]-cal_y)**2)**0.5

            cal_x = (plan_2[0]-self.x)*math.cos(-self.yaw) - (plan_2[1]-self.y)*math.sin(-self.yaw)
            cal_y = (plan_2[0]-self.x)*math.sin(-self.yaw) + (plan_2[1]-self.y)*math.cos(-self.yaw)
            r2 = ((points_range[:, 0]-cal_x)**2+(points_range[:, 1]-cal_y)**2)**0.5

            obj_plan_x.append([0,plan_1[0],plan_2[0]])
            obj_plan_y.append([0,plan_1[1],plan_2[1]])



            if np.sum(r < path_dis) > 0:
                obj.append(i)

                # print("r1", np.sum(r1 < path_dis),"r2",np.sum(r2 < path_dis))
                if (np.sum(r1 < path_dis) == 0) and (np.sum(r2 < path_dis) == 0):
                    obj_plan.append(3)
                elif np.sum(r1 < path_dis) == 0:#右によける必要あり
                    obj_plan.append(1)
                elif np.sum(r2 < path_dis) == 0:#左によける必要あり
                    obj_plan.append(2)
                else:           #よけられない
                    obj_plan.append(-1)
            else:
                obj.append(-1)
                obj_plan.append(0)

        if len(obj) != obj.count(-1) :#すべて-1でない→　障害物あり
            if obj[0:12].count(-1) != len(obj[0:12]): #3m以内に何らかの障害物あり
                #退避経路への移動が必要
                if -1 in obj_plan[0:2]:  #0.5m間隔のパスとして2.0m以内に避けられない障害物で停止
                    self.object_stop = 1
                    for i in self.object_path_change_id:
                        self.cx[i] = copy.deepcopy(self.cx_o[i])
                        self.cy[i] = copy.deepcopy(self.cy_o[i])
                    self.object_path_change_id = []
                    self.old_sayuu = -1
                else:
                    
                    if (1 in obj_plan[0:15]) and not(2 in obj_plan[0:15]) and not(-1 in obj_plan[0:15]) :#右回避があり、左回避がない場合
                        sayuu = 1
                    elif not(1 in obj_plan[0:15]) and (2 in obj_plan[0:15]) and not(-1 in obj_plan[0:15]):#右回避がなしで、左回避がありの場合
                        sayuu = 2
                    elif 3 in obj_plan[0:15] and not(-1 in obj_plan[0:15]):   #左右どちらに回避しても良い場合
                        sayuu = 2
                    else:
                        print("############")
                        sayuu = -1
                        self.old_sayuu = -1
                        # self.object_stop = 1
                        for i in self.object_path_change_id:
                            self.cx[i] = copy.deepcopy(self.cx_o[i])
                            self.cy[i] = copy.deepcopy(self.cy_o[i])
                        self.object_path_change_id = []
                    
                    print("sayuu", sayuu)
                    # if self.old_sayuu != sayuu and self.old_sayuu != 0 and sayuu != 0:
                    #     sayuu = self.old_sayuu

                    # print("sayuu",sayuu)
                    if sayuu == self.old_sayuu and sayuu !=-1:#左右に回避できる場合
                        self.object_stop = 0
                        self.object_path_change = 1
                        count_i =0
                        self.object_last_id = self.old_nearest_point_index+2 
                        for i in range(self.old_nearest_point_index+2, self.old_nearest_point_index+16):
                            if sayuu== 1:#右回避
                                self.cx[i] = obj_plan_x[count_i][1]
                                self.cy[i] = obj_plan_y[count_i][1]
                            else:   #左回避
                                self.cx[i] = obj_plan_x[count_i][2]
                                self.cy[i] = obj_plan_y[count_i][2]
                            if not(i in self.object_path_change_id):
                                self.object_path_change_id.append(i)
                            count_i+=1
                            if len(self.cx)<count_i+2+self.old_nearest_point_index+2:
                                break
                    self.old_sayuu = sayuu
            else:
                self.object_path_change = 0
                self.object_stop = 0
                if self.old_nearest_point_index - self.object_last_id >3:   #最後の障害物IDから2個進んでからパスを戻す
                    for i in self.object_path_change_id:
                        self.cx[i] = copy.deepcopy(self.cx_o[i])
                        self.cy[i] = copy.deepcopy(self.cy_o[i])
                    self.object_path_change_id = []
                self.old_sayuu = 0

            self.point_obj_clear_coutn = 0
        else:
            self.old_sayuu = 0
            self.object_path_change = 0
            self.object_stop = 0
            self.point_obj_clear_coutn += 1
            if (self.old_nearest_point_index - self.object_last_id >3) or self.point_obj_clear_coutn >3:
                for i in self.object_path_change_id:
                    self.cx[i] = copy.deepcopy(self.cx_o[i])
                    self.cy[i] = copy.deepcopy(self.cy_o[i])
                self.object_path_change_id = []

        # print(obj)
        self.obj = obj
        self.obj_plan = obj_plan
        self.obj_plan_x = obj_plan_x
        self.obj_plan_y = obj_plan_y


    def twist_CB(self, data):
        if self.auto_mode :
            # RANGE = 1.0
            RANGE = 3.2
            #self.vel[1] = RANGE * 1.0 * abs(data.linear.x) * self.auto_mode
            self.vel[1] = RANGE * (-1.0) * data.twist.linear.x * self.auto_mode
        
            # self.vel[1] = data.linear.y
            #self.vel[2] = RANGE * 1.0 * abs(data.angular.z) * self.auto_mode
            self.vel[2] = RANGE * (1.0) * data.twist.angular.z * self.auto_mode

    def interface_control(self, data):
        RANGE_J = 3
        self.vel[1] = RANGE_J * (1.0) * data.linear.x
        self.vel[3] = RANGE_J * (1.0) * data.angular.z
    

    def joy_CB(self, data):
        RANGE_J = 2.0

        self.vel[0] = RANGE_J * data.axes[3]
        self.vel[1] = RANGE_J * data.axes[1]
        self.vel[2] = RANGE_J * data.axes[0]

        ##if data.axes[6] or data.axes[7]:
        ##    self.vel[1] = 0.08 * data.axes[7]
        ##    self.vel[2] = 0.08 * data.axes[6]
        self.slow_down = (2 + data.axes[2])/5

        if data.buttons[1]:#o
            self.auto_mode = True
        if data.buttons[0]:#x
            self.auto_mode = False

    def write(self):
        #https://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html

        TREAD = 0.6 #車輪幅
        RANGE_K = 400#350 #255に収める係数

        V = self.vel[1] * self.slow_down
        W = self.vel[2] * self.slow_down

        VR = V + TREAD*W
        VL = V - TREAD*W

        motor_R = int(VR * RANGE_K)
        motor_L = int(VL * RANGE_K)

        if motor_R > 255:
            motor_R = 254
        elif motor_R < -255:
            motor_R = -254
        
        if motor_L > 255:
            motor_L = 254
        elif motor_L < -255:
            motor_L = -254

       # self.raw_speed[0] = motor_R
       # self.raw_speed[1] = motor_L
        
       # if self.auto_mode == 1:
       #     if motor_R == 0 and motor_L == 0:
       #         motor_R = self.pre_data[0] 
       #         motor_L = self.pre_data[1] 
       #     else:
       #         self.new_data[0] = motor_R
       #         self.new_data[1] = motor_L
                
        self.pre_data[0] = self.new_data[0] 
        self.pre_data[1] = self.new_data[1]

        if motor_R > 0:
            self.send_data[1] = 1
        else:
            self.send_data[1] = 0
        self.send_data[2] = abs(motor_R)

        if motor_L > 0:
            self.send_data[3] = 1
        else:
            self.send_data[3] = 0
        self.send_data[4] = abs(motor_L)       

        
        print(self.send_data, self.auto_mode)
        
        #self.ser.write(str.encode(self.send_data))

        
        for i in range(5):
        #self.ser.write(chr(self.send_data[i]))      
           self.ser.write(self.send_data[i].to_bytes(1, 'big'))


if __name__ == '__main__':
    try:
        rospy.init_node('pub_pose', anonymous=True)
        Hz = 20
        r = rospy.Rate(Hz)



        #csv_name = "/home/tranchung/shared_dir/data/232_0621_way1-2.csv"
        
        #print(csv_name)
        #path_data = pd.read_csv(csv_name)
   
        #print(path_data.shape)
    

        mypose = PubPose()
        rospy.Subscriber("current_pose", PoseStamped, mypose.pose_now_callback)
        rospy.Subscriber("closest_waypoint", Float32, mypose.closest_waypoint_index_callback)
	

        print("ini",mypose.x)
        
        
        target_ind, _ = mypose.search_target_index()
        rospy.Subscriber("points_raw", PointCloud2, mypose.points_raw_callback)
        rospy.Subscriber("cmd_vel", Twist, mypose.interface_control)
        rospy.Subscriber("twist_cmd", TwistStamped, mypose.twist_CB)
        rospy.Subscriber("joy", Joy, mypose.joy_CB)
        pub = rospy.Publisher("send_speed", Int32MultiArray, queue_size=10)

        try:
            while not rospy.is_shutdown():
                mypose.write()

                msg_data = Int32MultiArray(data=mypose.raw_speed)
                pub.publish(msg_data)

                r.sleep()
        except KeyboardInterrupt:
            mypose.ser.close()
    except rospy.ROSInterruptException:
        pass
