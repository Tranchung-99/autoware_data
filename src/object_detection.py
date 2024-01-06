#!/usr/bin/env python3
# coding: UTF-8

from __future__ import print_function

import rospy
import numpy as np
import ros_numpy
from std_msgs.msg import Header, Int16MultiArray
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
import struct
import open3d as o3d

FIELDS = [
    # 点の座標(x, y, z)
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

class ObjectDetection:
    def __init__(self):
        rospy.init_node('object_detection', anonymous=True)
        Hz = 20
        self.r = rospy.Rate(Hz)

        self.pub_marker_front = rospy.Publisher("object_front", Marker, queue_size=10)
        self.pub_marker_right = rospy.Publisher("object_right", Marker, queue_size=10)
        self.pub_marker_left = rospy.Publisher("object_left", Marker, queue_size=10)
        self.pub_object_detection = rospy.Publisher("object_detection", Int16MultiArray, queue_size=10)
        #self.pub_test_points = rospy.Publisher("test_points", PointCloud2, queue_size=10)
        rospy.Subscriber("points_raw", PointCloud2, self.pointcloud_CB)

        self.wide_view_angle = rospy.get_param("~wide_view", 240.0)
        self.wide_view_angle = self.wide_view_angle/2.0 * 2.0*np.pi/360.0
        self.narrow_view_angle = rospy.get_param("~narrow_view", 60.0)
        self.narrow_view_angle = self.narrow_view_angle/2.0 * 2.0*np.pi/360.0
        self.detection_range_front = rospy.get_param("~front_range", 1.5)
        self.detection_range_side = rospy.get_param("~side_range", 1.3)
        # self.detection_range_front = rospy.get_param("~front_range", 2.5)
        # self.detection_range_side = rospy.get_param("~side_range", 2.5)       
        self.min_detect_points = rospy.get_param("~min_points", 100)


    def pointcloud_CB(self, msg):
        np_points = ros_numpy.numpify(msg)
        self.points=np.zeros((np_points.shape[0],5))
        self.points[:,0]=np_points['x']
        self.points[:,1]=np_points['y']
        self.points[:,2]=np_points['z']
        # self.points[:,3]=0xffffff
        # self.points[:,4]=np.arange(self.points.shape[0])

        self.points = self.points[self.points[:, 0] > 0]
        self.points = self.points[self.points[:, 2] > -0.7]

        angle = np.arctan2(self.points[:, 1], self.points[:, 0])
        left_angle = np.where((angle > self.narrow_view_angle) & (angle < self.wide_view_angle), True, False)
        right_angle  = np.where((angle < -self.narrow_view_angle) & (angle > -self.wide_view_angle), True, False)
        front_angle = np.where((angle > -self.narrow_view_angle) & (angle < self.narrow_view_angle), True, False)

        right_points = self.points[right_angle]
        left_points  = self.points[left_angle]
        front_points = self.points[front_angle]

        right_r = np.hypot(right_points[:, 0], right_points[:, 1])
        left_r  = np.hypot(left_points[:, 0], left_points[:, 1])
        front_r = np.hypot(front_points[:, 0], front_points[:, 1])

        right_points = right_points[right_r < self.detection_range_side]
        left_points  = left_points[left_r < self.detection_range_side]
        front_points = front_points[front_r < self.detection_range_front]

        detecttion = [0, 0, 0]#left front right
        if left_points.size > self.min_detect_points:
            detecttion[0] = 1
            marker = self.make_marker_msg(1.0, 1.0, 0.0, 0.0, 1.0)
            self.pub_marker_left.publish(marker)
        else:
            marker = self.make_marker_msg(1.0, 1.0, 1.0, 1.0, 1.0)
            self.pub_marker_left.publish(marker)

        if front_points.size > self.min_detect_points:
            detecttion[1] = 1
            marker = self.make_marker_msg(1.5, 0.0, 0.0, 0.0, 1.0)
            self.pub_marker_front.publish(marker)
        else:
            marker = self.make_marker_msg(1.5, 0.0, 1.0, 1.0, 1.0)
            self.pub_marker_front.publish(marker)

        if right_points.size > self.min_detect_points:
            detecttion[2] = 1
            marker = self.make_marker_msg(1.0, -1.0, 0.0, 0.0, 1.0)
            self.pub_marker_right.publish(marker)
        else:
            marker = self.make_marker_msg(1.0, -1.0, 1.0, 1.0, 1.0)
            self.pub_marker_right.publish(marker)
        print(detecttion)

        msg = Int16MultiArray()
        msg.data = detecttion
        self.pub_object_detection.publish(msg)

        #self.pub_points(right_points)


    def make_marker_msg(self, x, y, r, g, b):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = 2#sphere
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 0.8
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        return marker


    # def pub_points(self, filtered_points):
    #     filtered_points = filtered_points[:, 0:3]
    #     print(filtered_points.shape)
    #     if filtered_points.size != 0:
    #         #直接msg型の中身を書き込んでいく
    #         data_send = PointCloud2()
    #         t = rospy.Time.now()
    #         data_send.header.stamp.secs=int(t.to_sec())
    #         data_send.header.stamp.nsecs=t.to_nsec()-int(t.to_sec())*10**9
    #         data_send.header.frame_id = "velodyne"
    #         data_send.height = 1
    #         data_send.width = filtered_points.shape[0]
    #         data_send.fields = FIELDS
    #         data_send.is_bigendian = False
    #         data_send.point_step = 12#FIELDSのbyte数
    #         data_send.row_step = 12*filtered_points.shape[0]#総byte数

    #         tmp = filtered_points.reshape(filtered_points.shape[0]*filtered_points.shape[1])
    #         data_send.data = struct.pack('fff'*filtered_points.shape[0], *tmp)#*tmpでunpackして引数を並べる
    #         data_send.is_dense = True

    #         self.pub_test_points.publish(data_send)

    #         end_time = rospy.Time.now()
    #         end_time = end_time.to_sec()
    #         # print(end_time - start_time)
    #     else:
    #         print("No points")


    def main(self):
        try:
            while not rospy.is_shutdown():

                self.r.sleep()
        except KeyboardInterrupt:
            None


if __name__ == '__main__':
    myinfo = ObjectDetection()
    try:
        myinfo.main()
    except rospy.ROSInterruptException:
        pass
