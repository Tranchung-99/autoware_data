#!/usr/bin/env python3
# coding: UTF-8

from __future__ import print_function

import tf
import serial
import rospy
import roslib
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

class NDT_encoder:
    def __init__(self):
        rospy.init_node('ndt_encoder', anonymous=True)
        self.r = rospy.Rate(30)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.last_time = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0


    def encoder_CB(self, data):
        lr = data.data
        speed_R = lr[1]*0.6
        speed_L = lr[0]*0.6
        vx = (speed_R + speed_L)/2.0
        vy = 0.0
        vth = (speed_R - speed_L)/(2.0 * 0.53/2.0)

        time = rospy.Time.now()
        current_time = time.to_sec()

        dt = current_time - self.last_time
        delta_x = (vx * np.cos(self.th) - vy * np.sin(self.th)) * dt
        delta_y = (vx * np.sin(self.th) + vy * np.cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        #odom_quat = Quaternion()
        #odom_quat = tf.createQuaternionMsgFromYaw(self.th)
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # odom_trans = TransformStamped()
        # odom_trans.header.stamp = current_time
        # odom_trans.header.frame_id = "odom"
        # odom_trans.child_frame_id = "base_link"

        # odom_trans.transform.translation.x = self.x
        # odom_trans.transform.translation.y = self.y
        # odom_trans.transform.translation.z = 0.0
        # odom_trans.transform.rotation = odom_quat
        # print(odom_trans)

        self.odom_broadcaster.sendTransform((self.x, self.y, 0),
                     odom_quat,
                     rospy.Time.now(),
                     "odom",
                     "base_link")

        ##self.odom_broadcaster.sendTransform(odom_trans)

        odom = Odometry()
        odom.header.stamp = time
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)
        print(odom)

        self.last_time = current_time

    def main(self):
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        rospy.Subscriber("encoder_speed", Float32MultiArray, self.encoder_CB)

        while not rospy.is_shutdown():
            self.r.sleep()

if __name__ == '__main__':
  
    myencoder = NDT_encoder()
    try:
    	
        myencoder.main()

    except rospy.ROSInterruptException:
        print("fin.")
