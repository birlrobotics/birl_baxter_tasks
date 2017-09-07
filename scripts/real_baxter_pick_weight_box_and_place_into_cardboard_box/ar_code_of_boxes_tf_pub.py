#!/usr/bin/env python  
import rospy
from math import pi
import tf
import ipdb

if __name__ == '__main__':
    rospy.init_node('ar_code_of_boxes_tf_pub')

    weight_box_frame = 'ar_marker_1'
    weight_box_pick_frame = 'pick_pose_frame'
    cardboard_box_frame = 'ar_marker_0'
    cardboard_box_place_frame = 'place_pose_frame'
    cardboard_depth = 0.15
    cardboard_center_offset = 0.1

    rate = rospy.Rate(1)

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    transformer_ros = tf.TransformerROS()

    while not rospy.is_shutdown():
        rospy.loginfo('hihi')

        listener.waitForTransform('/base', weight_box_frame, rospy.Time(0), rospy.Duration(100.0))
        (trans,rot) = listener.lookupTransform('/base', weight_box_frame, rospy.Time(0))

        mat = transformer_ros.fromTranslationRotation(trans,rot)
        x_axis_vec = mat[:3, 0]
        y_axis_vec = mat[:3, 1]
        z_axis_vec = mat[:3, 2]

        mat[:3, 0] = -x_axis_vec 
        mat[:3, 2] = -z_axis_vec
        if y_axis_vec[1] < 0:
            mat[:3, 0] = -x_axis_vec
            mat[:3, 1] = -y_axis_vec

        br.sendTransform(
            trans,
            tf.transformations.quaternion_from_matrix(mat),
            rospy.Time.now(),
            weight_box_pick_frame,
            '/base',
        )

        listener.waitForTransform('/base', cardboard_box_frame, rospy.Time(0), rospy.Duration(100.0))
        (trans,rot) = listener.lookupTransform('/base', cardboard_box_frame, rospy.Time(0))

        mat = transformer_ros.fromTranslationRotation(trans,rot)
        x_axis_vec = mat[:3, 0].copy()
        y_axis_vec = mat[:3, 1].copy()
        z_axis_vec = mat[:3, 2].copy()

        mat[:3, 0] = -y_axis_vec 
        mat[:3, 1] = z_axis_vec
        mat[:3, 2] = -x_axis_vec

        trans += cardboard_center_offset*mat[:3, 1]+cardboard_depth*mat[:3, 2]

        br.sendTransform(
            trans,
            tf.transformations.quaternion_from_matrix(mat),
            rospy.Time.now(),
            cardboard_box_place_frame,
            '/base',
        )
        rate.sleep()
