#!/usr/bin/env python

import sys
import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

def new_marker(pos, id):
    obstacle_marker = Marker()
    obstacle_marker.ns = "obstacle_vis"
    obstacle_marker.header.frame_id = "/world"
    obstacle_marker.header.stamp = rospy.Time.now()
    obstacle_marker.action = Marker.ADD
    obstacle_marker.id = id
    obstacle_marker.type = Marker.SPHERE
    obstacle_marker.scale.x = 0.5
    obstacle_marker.scale.y = 0.5
    obstacle_marker.scale.z = 0.5
    obstacle_marker.color.a = 1
    obstacle_marker.color.r = 1.0
    obstacle_marker.color.g = 1.0
    obstacle_marker.color.b = 0.0
    obstacle_marker.pose.position.x = pos[0]
    obstacle_marker.pose.position.y = pos[1]
    obstacle_marker.pose.position.z = pos[2]
    obstacle_marker.pose.orientation.x = 0.0;
    obstacle_marker.pose.orientation.y = 0.0;
    obstacle_marker.pose.orientation.z = 0.0;
    obstacle_marker.pose.orientation.w = 1.0;
    return obstacle_marker

def talker():
    pub = rospy.Publisher('obstacle_markers', MarkerArray, queue_size=10)
    rospy.init_node('vis_diy_world', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    obstacle_markers = MarkerArray()
    id_cnt = 0
    obstacle_markers.markers.append(new_marker([0, 0, 0], id_cnt))
    id_cnt += 1
    obstacle_markers.markers.append(new_marker([3, 0, 0], id_cnt))
    id_cnt += 1
    obstacle_markers.markers.append(new_marker([6, 0, 0], id_cnt))

    while not rospy.is_shutdown():
        pub.publish(obstacle_markers)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
