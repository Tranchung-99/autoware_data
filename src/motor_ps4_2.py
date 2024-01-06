#!/usr/bin/env python3
# coding: UTF-8

from __future__ import print_function

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Float32, Int32
from geometry_msgs.msg import TwistStamped



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
        

class serial_ps4:
    def __init__(self):
        self.send = [255, 0,0,0,0,0]
        self.max_speed_count=0
        self.data_print=[0,0]
        # serial_port = list(serial.tools.list_ports.comports())
        self.ang_offset=0
        # for i in range(len(serial_port)):
        #     print(serial_port[i])
        # print("\n select COM?")
        # print("number : ",end="")

        # timeoutを秒で設定（default:None)ボーレートはデフォルトで9600
        #self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=3)
        self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=3)
        # self.ser = serial.Serial("COM"+str(13),19200,timeout=3)  # timeoutを秒で設定（default:None)ボーレートはデフォルトで9600
        self.joy_count = 0
        self.flag_joy = 1
          
        self.auto_flag = 0 #自動制御　許可

        self.timer_joy = 0
        self.timer_auto = 0

        self.sikikenn = 0

        self.auto_flag_pub = rospy.Publisher("auto_flag", Float32, queue_size=10)
        self.jack_cmd_pub =  rospy.Publisher("jack_cmd", Int32, queue_size=10)

        


    def rec_speed(self,data):
        print("\t\t\trec_speed",data.data[0],data.data[1])
    
        #joyを使っていない場合のみ、自動制御信号を通す
        #data[0]: Right,    data[1]: Left
        if self.flag_joy == 0 and self.auto_flag == 1:
            self.sikikenn = 2
            
            self.data_print[0]=data.data[0]
            self.data_print[1]=data.data[1]
            
            self.send[0] = 255
            if data.data[1] > 0:
                self.send[3] = 1
            else:
                self.send[3] = 0            
            self.send[4] = min(abs(data.data[1]), 254)

            if data.data[0]>0:
                self.send[1] = 1
            else:
                self.send[1] = 0
            self.send[2] = min(abs(data.data[0]), 254)
            self.timer_auto = time.time()
    
        
    def interface_control(self, data):
        RANGE_I = 3.0
        self.vel[1] = RANGE_I * (1.0) * data.linear.x
        self.vel[2] = RANGE_I * (1.0) * data.angular.z

    def joy4(self, data):
        # print(str(type(data)), "Joy" in str(type(data)))
        Vx = data.axes[1]
        Vt = data.axes[2]

        if data.buttons[4]==1:
            Vx = Vx*0.3  #固定の速度
            Vt = Vt * 0.3
        elif data.buttons[2]==1:#速度検証用　特殊fanc
            Vx = 0
            Vt = 0.4
        
        # V_tmp = (Vx*Vx+Vt*Vt)**0.5
        valR = Vx + Vt 
        valL = Vx - Vt 
        
        valL = int(valL*254)
        valR = int(valR*254)
        
        self.timer_joy = time.time()

        if(self.joy_count < 5):
            self.sikikenn = 1
            self.send[0] = 255
            if valL>0:
                self.send[3] = 1
            else:
                self.send[3] = 0            
            self.send[4] = min(abs(valL),254)

            if valR>0:
                self.send[1] = 1
            else:
                self.send[1] = 0
            self.send[2] = min(abs(valR),254)
            
            self.flag_joy = 1

            self.data_print[0]=valL
            self.data_print[1]=valR

        else:
            self.flag_joy = 0

        if data.buttons[3]==1:
            self.sikikenn = 2
            self.auto_flag = 1
        if data.buttons[1]==1:
            self.sikikenn = 1

            self.auto_flag = 0

            self.data_print[0] = 0
            self.data_print[1]=0
            self.send[1]=0
            self.send[2]=0
            self.send[3]=0
            self.send[4]=0
            
        self.auto_flag_pub.publish(self.auto_flag)

        #print("a ", valL, valR, self.joy_count)
        #JOY操作があるか、ないか
        if(data.axes[1] == 0 and data.axes[2] == 0 and Vx ==0 and Vt == 0):
            self.joy_count = min(self.joy_count+1, 5)
        else:
            self.joy_count = 0


        #ジャッキ操作用 上で上がる
        if int(data.axes[10])==1:
            self.jack_cmd_pub.publish(Int32(2))
        elif int(data.axes[10])==-1:
            self.jack_cmd_pub.publish(Int32(1))

if __name__ == '__main__':
    try:

        rospy.init_node('motor_ps4', anonymous=True)
        r = rospy.Rate(30)  # 10hz

        seri = serial_ps4()  # シリアル設定　クラス

        rospy.Subscriber("joy", Joy, seri.joy4)
        rospy.Subscriber("twist_cmd", TwistStamped, seri.rec_speed)

        pub = rospy.Publisher('send_data',Float32MultiArray , queue_size=100)

        now = datetime.datetime.now()
        
        try:
            while not rospy.is_shutdown():
                if seri.flag_joy == 0 and (time.time()-seri.timer_auto) > 1.0:
                    seri.send[1] = 0
                    seri.send[2] = 0
                    seri.send[3] = 0
                    seri.send[4] = 0
                    seri.data_print[0]=0
                    seri.data_print[1]=0

                print(seri.send, seri.flag_joy,seri.data_print,seri.sikikenn, seri.auto_flag)

                #Topic: "/send_data"　送信
                tmp1 = [seri.data_print[0],seri.data_print[1]]
                tmp1.extend(seri.send)
                tmp2 = Float32MultiArray(data = tmp1)
                pub.publish(tmp2)


                seri.ser.write(seri.send[0].to_bytes(1, 'big'))
                seri.ser.write(seri.send[1].to_bytes(1, 'big'))
                seri.ser.write(seri.send[2].to_bytes(1, 'big'))
                seri.ser.write(seri.send[3].to_bytes(1, 'big'))
                seri.ser.write(seri.send[4].to_bytes(1, 'big'))
                seri.ser.write(seri.send[5].to_bytes(1, 'big'))
                # rospy.spin()
                r.sleep()
                # time.sleep(0.1)
        except KeyboardInterrupt:
            seri.close()

    except rospy.ROSInterruptException:
        pass
