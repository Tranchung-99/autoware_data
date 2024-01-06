#!/usr/bin/env python3
# coding: UTF-8

from __future__ import print_function

import rospy
import roslib
from std_msgs.msg import Header, Int64
from geometry_msgs.msg import TwistStamped, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf

import struct
from PIL import Image
import numpy as np
from scipy.spatial.transform import Rotation


class PurePursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        self.r = rospy.Rate(20)

        self.pub_twist = rospy.Publisher("twist_cmd_obj", TwistStamped, queue_size=10)
        self.pub_marker = rospy.Publisher("next_target_mark1", Marker, queue_size=10)
        self.pub_marker_way = rospy.Publisher("next_waypoint_mark1", Marker, queue_size=10)
        self.pub_marker_pose = rospy.Publisher("current_pose_mark1", Marker, queue_size=2)
        self.pub_id = rospy.Publisher("look_waypoint", Int64, queue_size=10)
        rospy.Subscriber("waypoints_mark", MarkerArray, self.waypoints_CB)
        #rospy.Subscriber("estimate_pose", PoseStamped, self.pose_CB)
        rospy.Subscriber("current_pose", PoseStamped, self.pose_CB)

        self.lookahead = rospy.get_param("~lookahead", 1)
        print(self.lookahead)
        self.offset = rospy.get_param("~offset", 1)
        print(self.offset)
        self.waypoints = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float)
        self.pose_now = np.array([0.0, 0.0, 0.0], dtype=np.float)
        self.speed = np.array([0.0, 0.0], np.float) # 並進, 回転
        self.waypoints_size = 0


    def waypoints_CB(self, msg):
        self.waypoints = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float)
        point = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float)
        for i in msg.markers:
            point[0] = i.pose.position.x
            point[1] = i.pose.position.y
            q = [i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w]
            euler = tf.transformations.euler_from_quaternion(q)
            point[2] = euler[2]
            point[3] = i.scale.x#並進速度
            if i == 0:
                self.waypoints = point
            else:
                self.waypoints = np.vstack([self.waypoints, point])
        self.waypoints_size = self.waypoints.shape[0]
        print("Callback waypointns")
        


    def pose_CB(self, msg):
        self.pose_now[0] = msg.pose.position.x
        self.pose_now[1] = msg.pose.position.y
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(q)
        self.pose_now[2] = euler[2]
        print("pose ==========================")
        print(self.pose_now)


    def motion_ctl(self, waypoints, pose_now, id):
        look_angle = -pose_now[2] + np.arctan2(waypoints[id, 1]-pose_now[1], waypoints[id, 0]-pose_now[0])
        base_angle = waypoints[id - self.offset, 2] - pose_now[2]
        print("base_angle", base_angle)

        angle = look_angle# + base_angle
        angle2 = look_angle + base_angle
        print("angle_raw : ", angle)
        if angle >  np.pi:
            angle -= 2.0*np.pi
        elif angle < -np.pi:
            angle += 2.0*np.pi

        self.speed[0] = waypoints[id-self.offset+1, 3]*np.abs((np.pi - np.abs(angle)))/np.pi
        L = np.hypot(waypoints[id, 1]-pose_now[1], waypoints[id, 0]-pose_now[0])
        self.speed[1] = 2*self.speed[0]*np.sin(angle2)/L# + (np.max(waypoints[:, 3]) - 2*self.speed[0])*np.sin(angle2)/L
        print("angle : ", angle)
        print("Vx : ", self.speed[0])
        print("Vw : ", self.speed[1])

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = 2#sphere
        marker.action = Marker.ADD
        marker.pose.position.x = waypoints[id, 0]
        marker.pose.position.y = waypoints[id, 1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.pub_marker.publish(marker)

        marker.pose.position.x = waypoints[id-self.offset, 0]
        marker.pose.position.y = waypoints[id-self.offset, 1]
        marker.pose.position
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.pub_marker_way.publish(marker)

        marker.pose.position.x = pose_now[0]
        marker.pose.position.y = pose_now[1]
        marker.pose.position
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.pub_marker_pose.publish(marker)
        
      




    def main(self):
        try:
            while not rospy.is_shutdown():
                if self.waypoints.size < 4*2:
                    # print("break")
                    continue
                # 更新途中になることがあるので、完全に更新されたサイズと確認
                if self.waypoints.shape[0] != self.waypoints_size:
                    # print("break")
                    continue

                waypoints = self.waypoints
                pose_now = self.pose_now
                distance = np.hypot(waypoints[:, 0]-pose_now[0], waypoints[:, 1]-pose_now[1])
                min_id = np.argmin(distance)
                min_dis = np.min(distance)

                if min_dis < self.lookahead:
                    min_id += 1

                id = min_id + self.offset
                # print("look_id :", id)

                msg_tmp = Int64()
                msg_tmp.data = id
                self.pub_id.publish(msg_tmp)

                self.motion_ctl(waypoints, pose_now, id)

                send_twist = TwistStamped()
                send_twist.twist.linear.x = self.speed[0]
                send_twist.twist.angular.z = self.speed[1]

                self.pub_twist.publish(send_twist)


                self.r.sleep()
        except KeyboardInterrupt:
            None


if __name__ == '__main__':
    mypoint = PurePursuit()
    try:
        mypoint.main()
    except rospy.ROSInterruptException: pass
