#!/usr/bin/env python  
import rospy
from math import pi
import tf




if __name__ == '__main__':
    rospy.init_node('push_button_tf_broadcaster')
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform((0,-0.15,0),
                     tf.transformations.quaternion_from_euler(pi, 0, -pi/2),
                     rospy.Time.now(),
                     "button_co",
                     "ar_marker_1")
        rate.sleep()


