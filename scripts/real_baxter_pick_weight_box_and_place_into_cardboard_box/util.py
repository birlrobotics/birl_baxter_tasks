import rospy
from visualization_msgs.msg import (
    Marker
)

def send_traj_point_marker(marker_pub, pose, id, rgba_tuple):
    marker = Marker()
    marker.header.frame_id = "/base"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "traj_point" 
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.r = rgba_tuple[0]
    marker.color.g = rgba_tuple[1]
    marker.color.b = rgba_tuple[2]
    marker.color.a = rgba_tuple[3]
    marker.lifetime = rospy.Duration()
    marker_pub.publish(marker)       
