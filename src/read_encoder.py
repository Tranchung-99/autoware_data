#!/usr/bin/env python3
# coding: UTF-8

from __future__ import print_function

import serial
import rospy
import roslib
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32

if __name__ == '__main__':
    try:
        rospy.init_node('encoder', anonymous=True)
        r = rospy.Rate(50)  
        encoder_data = rospy.Publisher(
            "encoder_data", Float32MultiArray, queue_size=10)
        encoder_speed = rospy.Publisher(
            "encoder_speed", Float32MultiArray, queue_size=10)
        encoder_R = rospy.Publisher(
            "encoder_R", Float32, queue_size=10)
        encoder_L = rospy.Publisher(
            "encoder_L", Float32, queue_size=10)

        usb_port = rospy.get_param("~port", "/dev/ttyUSB1")
        ser = serial.Serial(usb_port, 115200, timeout=3)

        encoder_rec_data = [0, 0]
        encoder_rec_data_before = [0, 0]
        encoder_cal_speed = [0, 0]
        rec_count = 0
        rec_data = [0]*11
        rec_flag = 0


        count_print= 0

        while not rospy.is_shutdown():
            buffer = ser.readline()
            # print(buffer)
            # print(len(buffer))

            if len(buffer)==9:
                ibuffer = np.frombuffer(buffer, count=2, dtype=np.float32)
                
                data = np.copy(ibuffer)
                tmp = np.copy(data)

                data[0] = -tmp[1]
                data[1] = -tmp[0]

                # buffer = np.frombuffer(buffer, dtype=np.uint8)
                # data = np.array([0, 0], dtype=np.float32)

                # data[0] = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + (buffer[3])
                # data[1] = (buffer[4] << 24) + (buffer[5] << 16) + (buffer[6] << 8) + (buffer[7])

                reduction_ratio = 1.0/20.0
                wheel_radius = 0.22/2 #[m]
                data = data*reduction_ratio*wheel_radius

                send_msg = Float32MultiArray()
                send_msg.data = data
                encoder_speed.publish(send_msg)

                send_r = Float32()
                send_r.data = data[1]
                encoder_R.publish(send_r)

                send_l = Float32()
                send_l.data = data[0]
                encoder_L.publish(send_l)

                if count_print > 30:
                    print(data)
                    count_print = 0
                    # print(buffer)

            else:
                # print("Not connected")
                None

            count_print += 1

            # r.sleep()


    except rospy.ROSInterruptException:
        ser.close()
