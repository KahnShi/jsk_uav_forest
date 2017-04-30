#!/usr/bin/env python

import sys
import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

target_poses = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

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

def target1Callback(msg):
    target_poses[0][0] = msg.x
    target_poses[0][1] = msg.y
    target_poses[0][2] = msg.z

def target2Callback(msg):
    target_poses[1][0] = msg.x
    target_poses[1][1] = msg.y
    target_poses[1][2] = msg.z
def target3Callback(msg):
    target_poses[2][0] = msg.x
    target_poses[2][1] = msg.y
    target_poses[2][2] = msg.z

def talker():
    pub = rospy.Publisher('obstacle_markers', MarkerArray, queue_size=10)
    rospy.init_node('vis_diy_world', anonymous=True)
    target_number = rospy.get_param("~target_number", 1)
    rate = rospy.Rate(30) # 1hz

    sub_target1 = rospy.Subscriber('target_tree_pose_1', Point, target1Callback)
    sub_target2 = rospy.Subscriber('target_tree_pose_2', Point, target2Callback)
    sub_target3 = rospy.Subscriber('target_tree_pose_3', Point, target3Callback)

    while not rospy.is_shutdown():
        obstacle_markers = MarkerArray()
        for i in range(0, 3):
            id_cnt = i
            obstacle_markers.markers.append(new_marker(target_poses[i], id_cnt))
        pub.publish(obstacle_markers)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
