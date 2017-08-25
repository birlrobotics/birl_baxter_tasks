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

def move_to_start_pose():
    name = ['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
    position = [0.0, -0.06289321230330196, -0.022626216621309852, 0.7501166052759674, -1.39975746894544, -0.5568350260024052, -0.28033498898605935, 0.5813787186085718, 0.31408256631953846, -0.139975746894544, 1.389786593824185, 0.44293695250191323, -0.11159710231866385, -0.07976700097004151, -1.321524448763284, -0.14150972768242942, -12.565987119160338]

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    lj = left.joint_names()
    rj = right.joint_names()
    d = dict(zip(name, position))
    left.move_to_joint_positions({k:d[k] for k in lj})
    right.move_to_joint_positions({k:d[k] for k in rj})

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


    rospy.init_node('push_button_task')
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    def clean_shutdown():
        rospy.loginfo("Exiting...")
        if not init_state:
            rospy.loginfo("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Enabling robot... ")
    rs.enable()

    move_to_start_pose()

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

    limb = 'right'
    traj = srv_action_client.Trajectory(limb)
    traj.clear('right')
    for count in range(0, step_amount+1):
        step_apart = step_amount-count
        now_pose = copy.deepcopy(button_pose)
        now_translation = trans-step_apart*step_length*z_axis_vec
        now_pose.position.x = now_translation[0]
        now_pose.position.y = now_translation[1]
        now_pose.position.z = now_translation[2]
        traj.add_pose_point(now_pose, 5.0+count*5.0)

        alpha = float(count)/step_amount*0.5+0.5 
        rgba_tuple = (0, 0.5, 0.5, alpha)
        util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=count, rgba_tuple=rgba_tuple)

        rospy.loginfo("add one traj pose")
        
    #traj.start()
    #traj.wait(100)
    #print traj.result()


