#!/usr/bin/env python3
# coding: UTF-8

import rospy
from Autoware_msgs.msg import LaneArray
from visualization_msgs.msg import Marker, MarkerArray

def lane_array_to_marker_array(lane_array):
    marker_array = MarkerArray()
    
    for lane in lane_array.lanes:
        # Extract information from the LaneArray and create a Marker
        marker = Marker()
        marker.header = lane_array.header
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Set the line width
        marker.color.r = 1.0
        marker.color.a = 1.0
        
        # Assuming the lane has points in lane.waypoints (modify this according to Autoware's message structure)
        for waypoint in lane.waypoints:
            point = waypoint.pose.pose.position
            marker.points.append(point)
        
        marker_array.markers.append(marker)
    
    return marker_array

# Your callback function for the LaneArray message
def lane_array_callback(msg):
    marker_array = lane_array_to_marker_array(msg)
    # Publish the MarkerArray
    marker_publisher.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('lane_array_to_marker_array')
    lane_array_topic = '/based/lane_waypoints_raw'
    marker_publisher = rospy.Publisher('/waypoints_mark', MarkerArray, queue_size=10)
    rospy.Subscriber(lane_array_topic, LaneArray, lane_array_callback)
    rospy.spin()