#!/usr/bin/env python3
# coding: UTF-8

from __future__ import print_function
from math import atan2, sqrt

import rospy
import serial
import struct
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

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

    def twist_CB(self, data):
        if self.auto_mode:
            # RANGE = 1.0
            RANGE = 3.2
            #self.vel[1] = RANGE * 1.0 * abs(data.linear.x) * self.auto_mode
            self.vel[1] = RANGE * (-1.0) * data.twist.linear.x * self.auto_mode
            # self.vel[1] = data.linear.y
            #self.vel[2] = RANGE * 1.0 * abs(data.angular.z) * self.auto_mode
            self.vel[2] = RANGE * (1.0) * data.twist.angular.z * self.auto_mode

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

        self.raw_speed[0] = motor_R
        self.raw_speed[1] = motor_L

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

        mypose = PubPose()
        #rospy.Subscriber("cmd_vel", TwistStamped, mypose.twist_CB)
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
