#!/usr/bin/env python

import rospy
from std_msgs.msg import (
    Header
)

if __name__ == "__main__":
    pub = rospy.Publisher("/anomaly_timestamp", Header, queue_size=1)
    rospy.init_node("manual_anomaly_singal", anonymous=True)
    while not rospy.is_shutdown():
        now_header = Header()
        rospy.loginfo('click enter to send one anomaly timestamp')
        raw_input()
        now_header.stamp = rospy.Time.now()
        pub.publish(now_header)
