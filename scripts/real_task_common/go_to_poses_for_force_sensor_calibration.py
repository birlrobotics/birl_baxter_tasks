import tf
import rospy
from math import pi
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)
from arm_move import srv_action_client

rospy.init_node('push_button_task')

# for X and Y
pose = Pose(
    position = Point(  
        x = 1.21296722331,
        y = -0.358296212153,
        z = 0.632572044162,
    ),
    orientation = Quaternion(
        x = 0,
        y = 0,
        z = 0,
        w = 0,
    ),
)
right_traj = srv_action_client.Trajectory('right')
right_traj.clear('right')
right_traj.stop()
x, y, z, w = tf.transformations.quaternion_from_euler(0, pi/2, 0)
pose.orientation.x = x
pose.orientation.y = y
pose.orientation.z = z
pose.orientation.w = w
right_traj.add_pose_point(pose, 4)
right_traj.start()
right_traj.wait(5)
print 'input a char to continue'
raw_input()


right_traj = srv_action_client.Trajectory('right')
right_traj.clear('right')
right_traj.stop()
x, y, z, w = tf.transformations.quaternion_from_euler(-pi/2, 0, -pi/2)
pose.orientation.x = x
pose.orientation.y = y
pose.orientation.z = z
pose.orientation.w = w
right_traj.add_pose_point(pose, 4)
right_traj.start()
right_traj.wait(5)
print 'input a char to continue'
raw_input()

#  for  Z
pose =  Pose(
    position =  Point(
        x = 0.409358816873,
        y = -0.878879603878,
        z = 0.148172329014,
    ),
    orientation = Quaternion(
        x = 0,
        y = 0,
        z = 0,
        w = 0,
    ),
)
right_traj = srv_action_client.Trajectory('right')
right_traj.clear('right')
right_traj.stop()
x, y, z, w = tf.transformations.quaternion_from_euler(0, pi, 0)
pose.orientation.x = x
pose.orientation.y = y
pose.orientation.z = z
pose.orientation.w = w
right_traj.add_pose_point(pose, 4)
right_traj.start()
right_traj.wait(5)
print 'input a char to continue'
raw_input()
