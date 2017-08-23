#!/usr/bin/env python  
import rospy
import math
import tf
from arm_move import srv_action_client
from geometry_msgs.msg import (
    Pose,
)
import ipdb

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    destination_frame = '/button_co'
    original_frame = '/base'
    listener.waitForTransform(destination_frame, original_frame, rospy.Time(0), rospy.Duration(60.0))
    (trans,rot) = listener.lookupTransform(destination_frame, original_frame, rospy.Time(0))

    rospy.loginfo("got button pose")

    limb = 'right'
    traj = srv_action_client.Trajectory(limb)
    traj.clear('right')
    push_button_pose = Pose() 
    push_button_pose.position.x = trans[0]
    push_button_pose.position.y = trans[1]
    push_button_pose.position.z = trans[2]
    push_button_pose.orientation.x = rot[0]
    push_button_pose.orientation.y = rot[1]
    push_button_pose.orientation.z = rot[2]
    push_button_pose.orientation.w = rot[3]
    traj.add_pose_point(push_button_pose, 4.0)
    traj.start()

