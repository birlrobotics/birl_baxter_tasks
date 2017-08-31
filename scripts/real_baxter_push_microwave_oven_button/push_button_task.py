#!/usr/bin/env python  
import rospy
import math
import tf
from arm_move import srv_action_client
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Vector3,
)
from visualization_msgs.msg import (
    Marker
)
import ipdb
import baxter_interface
from baxter_interface import CHECK_VERSION
import copy
from optparse import OptionParser
import util

import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 

def move_to_start_pose():
    name = ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
    position = [0.0, -0.06289321230330196, -0.022626216621309852, 0.7501166052759674, -1.39975746894544, -0.5568350260024052, -0.28033498898605935, 0.5813787186085718, 0.31408256631953846, -0.139975746894544, 1.389786593824185, 0.44293695250191323, -0.11159710231866385, -0.07976700097004151, -1.321524448763284, -0.14150972768242942, -12.565987119160338]

    left_limb = baxter_interface.Limb('left')
    right_limb = baxter_interface.Limb('right')
    
    d = dict(zip(name, position))

    left_traj = srv_action_client.Trajectory('left')
    left_traj.clear('left')
    left_traj.stop()
    left_traj.add_point([d[k] for k in left_limb.joint_names()], 4)
    left_traj.start()

    right_traj = srv_action_client.Trajectory('right')
    right_traj.clear('right')
    right_traj.stop()
    right_traj.add_point([d[k] for k in right_limb.joint_names()], 4)
    right_traj.start()

    left_traj.wait(5)
    right_traj.wait(5)


def build_parser():
    parser = OptionParser()
    parser.add_option(
        "--testmode",
        action="store_true", 
        dest="testmode",
        default = False,
        help="True if you want testmode.")
    return parser



if __name__ == '__main__':
    parser = build_parser()
    (options, args) = parser.parse_args()

    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('push_button_task')

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rospy.loginfo("Enabling robot... ")
    rs.enable()

    rospy.loginfo("move_to_start_pose... ")
    move_to_start_pose()
    rospy.loginfo("Done... ")

    if options.testmode:
        trans = (0.920350715896, -0.605202117105, 0.0247626151505)
        rot = (-0.0066838303625, 0.697091188679, -0.204543722836, 0.687154325116)
    else:
        listener = tf.TransformListener()
        from_frame = '/base'
        to_frame = '/button_co'
        listener.waitForTransform(from_frame, to_frame, rospy.Time(0), rospy.Duration(10.0))
        (trans,rot) = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))

    rospy.loginfo("got button pose")

    button_pose = Pose(
        position=Point(
            x = trans[0],
            y = trans[1],
            z = trans[2],
        ),
        orientation=Quaternion(
            x = rot[0],
            y = rot[1],
            z = rot[2],
            w = rot[3],
        ),
    ) 

    print button_pose

    step_amount = 5
    step_length = 0.01

    transformer_ros = tf.TransformerROS()
    mat = transformer_ros.fromTranslationRotation(trans,rot)
    x_axis_vec = mat[:3, 0]
    y_axis_vec = mat[:3, 1]
    z_axis_vec = mat[:3, 2]

    list_of_poses = []
    for count in range(0, step_amount+1):
        step_apart = step_amount-count
        now_pose = copy.deepcopy(button_pose)
        now_translation = trans-step_apart*step_length*z_axis_vec
        now_pose.position.x = now_translation[0]
        now_pose.position.y = now_translation[1]
        now_pose.position.z = now_translation[2]

        # visualize the pose in rviz using marker
        alpha = float(count)/step_amount*0.5+0.5 
        rgba_tuple = (0, 0.5, 0.5, alpha)
        util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=count, rgba_tuple=rgba_tuple)

        rospy.loginfo("add one pose")
        list_of_poses.append(now_pose)

    # Moveit!
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("right_arm")

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory
    )

    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ End effector link: %s" % group.get_end_effector_link() 
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    print "============ Generating plan 1"
    pose_target = list_of_poses[0]
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    print "============ Visulaize plan 1"
    display_trajectory_publisher.publish(display_trajectory);
       
    print "============ Move plan 1"
    group.go(wait=True)
    print "============ Done"
    group.stop()
    rospy.spin()
