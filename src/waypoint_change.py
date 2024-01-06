#!/usr/bin/env python3
# coding: UTF-8

from __future__ import print_function

import rospy
import roslib
from std_msgs.msg import Header, Int16MultiArray, Int64
from geometry_msgs.msg import TwistStamped, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf

import struct
from PIL import Image
import numpy as np
from scipy.spatial.transform import Rotation
import os


class Waypoint:
    def __init__(self):
        rospy.init_node('waypoint_sender', anonymous=True)
        self.r = rospy.Rate(10)

        self.pub_marker = rospy.Publisher("waypoints_mark", MarkerArray, queue_size=10)
        rospy.Subscriber("look_waypoint", Int64, self.id_CB)
        rospy.Subscriber("object_detection", Int16MultiArray, self.object_CB)
        

        self.waypoints = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float)

        self.waypoints_msg = MarkerArray()

        self.id = 0
        self.now_id = 0
        self.object_detection = np.zeros(3, dtype=np.int16)

        self.is_change_root = rospy.get_param("~change_root", True)
        self.max_r = rospy.get_param("~max_r", 20)
    
   # def waypoint_CB(self, msg):


    def id_CB(self, msg):
        self.now_id = msg.data


    def object_CB(self, msg):
        tmp = msg.data
        self.object_detection = np.array(tmp, dtype=np.int16)


    def add_marker(self, point):
        #point = [x, y, theta]
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "create_way"
        marker.id = self.id
        marker.type = 0
        marker.action = Marker.ADD

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, point[2])
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.scale.x = point[3]
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker.text = str(point[3])

        self.waypoints_msg.markers.append(marker)

        self.id += 1


    def add_position(self, now, theta, range):
        return now[0] + range*np.cos(theta), now[1] + range*np.sin(theta)


    def change_root(self):
        print(self.object_detection)
        r = np.hypot(self.waypoints_first[:, 0] - self.waypoints[:, 0], self.waypoints_first[:, 1] - self.waypoints[:, 1])
        
        if np.sum(self.object_detection) == 3:
            print("STOP!!!!!!!!!!!!!!!!")
            for i in range(-1, 3, 1):
                 self.waypoints_msg.markers[self.now_id + i].scale.x = 0.0

        else:
            print("Just straight !!")
            for i in range(-1, 3, 1):
                self.waypoints_msg.markers[self.now_id+i].pose.position.x = self.waypoints_first[self.now_id+i, 0]
                self.waypoints_msg.markers[self.now_id+i].pose.position.y = self.waypoints_first[self.now_id+i, 1]
                self.waypoints_msg.markers[self.now_id+i].scale.x = 0.5*self.waypoints_first[self.now_id+i, 3]
                self.waypoints[self.now_id+i] = self.waypoints_first[self.now_id+i]
        #else:
        #    print("STOP!!!!!!!!!!!!!!!!")
        #    for i in range(-1, 3, 1):
        #        self.waypoints_msg.markers[self.now_id + i].scale.x = 0.0                        
        
        # if np.sum(self.object_detection) == 3:
        #     print("STOP!!!!!!!!!!!!!!!!")
        #     for i in range(-1, 3, 1):
        #         self.waypoints_msg.markers[self.now_id + i].scale.x = 0.0
        # elif np.sum(self.object_detection) == 2:
        #     if self.object_detection[1] == 1:
        #         if self.object_detection[0] == 1:
        #             print("Trun RIGHT large")
        #             for i in range(-1, 3, 1):
        #                 x, y = self.add_position(self.waypoints[self.now_id+i, 0:2], self.waypoints[self.now_id+i, 2]-np.pi/1.9, 0.2)#左 -90度
        #                 # if r[self.now_id+i] < self.max_r:
        #                 if np.hypot(self.waypoints_first[self.now_id+i, 0]-x, self.waypoints_first[self.now_id+i, 1]-y) <= self.max_r:
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.x = x
        #                     self.waypoints_msg.markers[self.now_id+i].scale.x = 0.3*self.waypoints[self.now_id+i, 3]
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.y = y
        #                     self.waypoints[self.now_id+i, 0] = x
        #                     self.waypoints[self.now_id+i, 1] = y
        #         elif self.object_detection[2] == 1:
        #             print("Trun LEFT large")
        #             for i in range(-1, 3, 1):
        #                 x, y = self.add_position(self.waypoints[self.now_id+i, 0:2], self.waypoints[self.now_id+i, 2]+np.pi/1.8, 0.2)#右 +90度
        #                 # if r[self.now_id+i] < self.max_r:
        #                 if np.hypot(self.waypoints_first[self.now_id+i, 0]-x, self.waypoints_first[self.now_id+i, 1]-y) <= self.max_r:
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.x = x
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.y = y
        #                     self.waypoints_msg.markers[self.now_id+i].scale.x = 0.3*self.waypoints[self.now_id+i, 3]
        #                     self.waypoints[self.now_id+i, 0] = x
        #                     self.waypoints[self.now_id+i, 1] = y
        #     else :
        #         print("Just Straight !!")
        #         for i in range(-1, 3, 1):
        #             self.waypoints_msg.markers[self.now_id+i].pose.position.x = self.waypoints_first[self.now_id+i, 0]
        #             self.waypoints_msg.markers[self.now_id+i].pose.position.y = self.waypoints_first[self.now_id+i, 1]
        #             self.waypoints_msg.markers[self.now_id+i].scale.x = 0.5*self.waypoints_first[self.now_id+i, 3]
        #             self.waypoints[self.now_id+i] = self.waypoints_first[self.now_id+i]                
                

                
        # elif np.sum(self.object_detection) == 1:
        #     if self.object_detection[0] == 1:
        #         print("Trun RIGHT short")
        #         for i in range(-1, 3, 1):
        #                 x, y = self.add_position(self.waypoints[self.now_id+i, 0:2], self.waypoints[self.now_id+i, 2]-np.pi/2.0, 0.1)#左 -90度
        #                 # if r[self.now_id+i] < self.max_r:
        #                 if np.hypot(self.waypoints_first[self.now_id+i, 0]-x, self.waypoints_first[self.now_id+i, 1]-y) <= self.max_r:
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.x = x
        #                     self.waypoints_msg.markers[self.now_id+i].scale.x = 0.3*self.waypoints[self.now_id+i, 3]
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.y = y
        #                     self.waypoints[self.now_id+i, 0] = x
        #                     self.waypoints[self.now_id+i, 1] = y
        #     elif self.object_detection[1] == 1:
        #         print("Trun RIGHT shortshort")
        #         for i in range(-1, 3, 1):
        #                 x, y = self.add_position(self.waypoints[self.now_id+i, 0:2], self.waypoints[self.now_id+i, 2]-np.pi/2.0, 0.1)#左 -90度
        #                 # if r[self.now_id+i] < self.max_r:
        #                 if np.hypot(self.waypoints_first[self.now_id+i, 0]-x, self.waypoints_first[self.now_id+i, 1]-y) <= self.max_r:
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.x = x
        #                     self.waypoints_msg.markers[self.now_id+i].scale.x = 0.3*self.waypoints[self.now_id+i, 3]
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.y = y
        #                     self.waypoints[self.now_id+i, 0] = x
        #                     self.waypoints[self.now_id+i, 1] = y
        #     elif self.object_detection[2] == 1:
        #             print("Trun LEFT short")
        #             for i in range(-1, 3, 1):
        #                 x, y = self.add_position(self.waypoints[self.now_id+i, 0:2], self.waypoints[self.now_id+i, 2]+np.pi/2.0, 0.02)#右 +90度
        #                 # if r[self.now_id+i] < self.max_r:
        #                 if np.hypot(self.waypoints_first[self.now_id+i, 0]-x, self.waypoints_first[self.now_id+i, 1]-y) <= self.max_r:
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.x = x
        #                     self.waypoints_msg.markers[self.now_id+i].pose.position.y = y
        #                     self.waypoints_msg.markers[self.now_id+i].scale.x = 0.3*self.waypoints[self.now_id+i, 3]
        #                     self.waypoints[self.now_id+i, 0] = x
        #                     self.waypoints[self.now_id+i, 1] = y

        # elif np.sum(self.object_detection) == 0:
        #     print("Just straight !!")
        #     for i in range(-1, 3, 1):
        #         self.waypoints_msg.markers[self.now_id+i].pose.position.x = self.waypoints_first[self.now_id+i, 0]
        #         self.waypoints_msg.markers[self.now_id+i].pose.position.y = self.waypoints_first[self.now_id+i, 1]
        #         self.waypoints_msg.markers[self.now_id+i].scale.x = 0.5*self.waypoints_first[self.now_id+i, 3]
        #         self.waypoints[self.now_id+i] = self.waypoints_first[self.now_id+i]


    def main(self):
        try:
            # package_dir = roslib.packages.get_pkg_dir("soturon_layout")
            # file = rospy.get_param("~way_point_file", "kojo_konoji_ex3_ch.csv")
            # file_path = os.path.join(package_dir, "map/waypoint/" + file)
            #file_path = "/home/tranchung/shared_dir/data/spetified_route5.csv"
            file_path = "/home/tranchung/shared_dir/data/spetified_route1219-1.csv"
            #file_path = "/home/tranchung/shared_dir/data/1003_waypoint1.csv"
            #file_path = "/home/tranchung/shared_dir/data/232_0621_way1-2.csv"
            #print(csv_name)
            #path_data = pd.read_csv(csv_name)
            #print(path_data.shape)
            #print(file_path)
            # x, y, z, theta, v, None -> x, y, theta, v
            self.waypoints = np.loadtxt(file_path, skiprows=1, delimiter=',', dtype=float)
            self.waypoints = np.delete(self.waypoints, 2, axis=1)
            self.waypoints = np.delete(self.waypoints, 4, axis=1)
            self.waypoints_first = np.copy(self.waypoints)
            print(self.waypoints)

            for i in self.waypoints:
                self.add_marker(i)

            while not rospy.is_shutdown():
                if self.is_change_root == True:
                    self.change_root()
                self.pub_marker.publish(self.waypoints_msg)

                self.r.sleep()
        except KeyboardInterrupt:
            None


if __name__ == '__main__':
    mypoint = Waypoint()
    try:
        mypoint.main()
    except rospy.ROSInterruptException: pass
