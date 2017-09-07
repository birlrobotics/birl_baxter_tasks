
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
    position = [0.0, -0.045252433242619704, 0.38809713933500967, 1.226034144717417, -0.9690923627466101, -0.9460826509283289, -1.092577816171386, 1.554689528521867, 2.037509981508801, -0.9901845985800346, 1.5063691337034764, 0.3321068405771921, -1.1470341341413182, 0.4463884092746554, 1.3023496889147164, -0.03298058693953639, -12.565987119160338]

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
    
    rospy.loginfo('sleep 5 secs to let camera stablize.')
    rospy.sleep(5)

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

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory
    )

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)

    rospy.loginfo("move_to_start_pose... ")
    move_to_start_pose()
    rospy.loginfo("Done... ")

    listener = tf.TransformListener()
    from_frame = '/base'
    to_frame = 'pick_pose_frame'
    listener.waitForTransform(from_frame, to_frame, rospy.Time(0), rospy.Duration(10.0))
    (trans,rot) = listener.lookupTransform(from_frame, to_frame, rospy.Time(0))

    rospy.loginfo("got pick pose")

    transformer_ros = tf.TransformerROS()
    mat = transformer_ros.fromTranslationRotation(trans,rot)
    x_axis_vec = mat[:3, 0]
    y_axis_vec = mat[:3, 1]
    z_axis_vec = mat[:3, 2]

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
    rospy.loginfo('raw button pose %s'%button_pose)

    button_pose_trans = trans-0.2*z_axis_vec+0.10*y_axis_vec
    button_pose.position.x = button_pose_trans[0] 
    button_pose.position.y = button_pose_trans[1] 
    button_pose.position.z = button_pose_trans[2] 

    step_amount = 30 
    step_length = 0.01
    list_of_poses = []
    for count in range(0, step_amount+1):
        step_apart = step_amount-count
        now_pose = copy.deepcopy(button_pose)
        now_translation = button_pose_trans-step_apart*step_length*z_axis_vec
        now_pose.position.x = now_translation[0]
        now_pose.position.y = now_translation[1]
        now_pose.position.z = now_translation[2]

        # visualize the pose in rviz using marker
        alpha = float(count)/step_amount*0.5+0.5 
        rgba_tuple = (0, 0.5, 0.5, alpha)
        util.send_traj_point_marker(marker_pub=marker_pub, pose=now_pose, id=count, rgba_tuple=rgba_tuple)

        rospy.loginfo("add one pose")
        list_of_poses.append(now_pose)

    rospy.loginfo("============ Generating plan")
    group.set_max_acceleration_scaling_factor(0.01)
    group.set_max_velocity_scaling_factor(0.01)
    (plan, fraction) = group.compute_cartesian_path(
                             list_of_poses,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    rospy.loginfo("============ Visulaize plan")
    display_trajectory_publisher.publish(display_trajectory);


    rospy.loginfo("============ Enter \"ok\" to execute plan")
    s = raw_input()
    if s == 'ok':
        rospy.loginfo("============ Gonna execute plan")
        group.execute(plan)
    rospy.loginfo("============ Done")
    rospy.spin()
