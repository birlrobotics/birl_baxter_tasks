#!/usr/bin/env python
"""
pick and place service smach server

prereqursite:

!!Please rosrun baxter_interface Joint_trajectory_server first!
"""

import baxter_interface
from birl_baxter_tasks.srv import *

from birl_baxter_tasks.msg import (
    Hmm_Log
)

import sys
import rospy
import copy

from arm_move import srv_action_client

import smach
import smach_ros

import std_msgs.msg

import os

import hardcoded_data

import cv2
import cv_bridge

from sensor_msgs.msg import (
    Image,
)


event_flag = 1
execution_history = []


def hmm_state_switch_client(state):
    rospy.wait_for_service('hmm_state_switch')
    try:

        hmm_state_switch_proxy = rospy.ServiceProxy('hmm_state_switch',
                                                    State_Switch)
        req = State_SwitchRequest()
        req.state = state
        resp = hmm_state_switch_proxy(req)
        if resp.finish.data:
            print "Hmm State switch to %d succesfully" %state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)


## @brief wait for trajectory goal to be finished, perform preemptive anomaly detection in the meantime. 
## @param trajectory instance 
## @return True if anomaly is detected.
def wait_for_motion_and_detect_anomaly(traj_obj):
    # loop while the motion is not finished
    while not traj.wait(0.00001):
        # anomaly is detected
        if event_flag == 0:
            traj_obj.stop()
            rospy.loginfo("anomaly detected")
            return True

    return False

## @brief record exec history
## @param current_state_name string
## @param current_userdata userdata passed into current state 
## @param depend_on_prev_states True if current state's success depends on previous states 
## @return None
def write_exec_hist(state_instance, current_state_name, current_userdata, depend_on_prev_states):
    import copy
    global execution_history

    saved_userdata = {}
    for k in state_instance._input_keys:
        saved_userdata[k] = copy.deepcopy(current_userdata[k])

    execution_history.append(
        {
            "state_name": current_state_name,
            "saved_userdata": saved_userdata,
            "depend_on_prev_states": depend_on_prev_states
        }
    )

def execute_decorator(original_execute):
    def f(self, userdata): 
        if not hasattr(self, 'state_no'):
            state_no = 0 
        else:
            state_no = self.state_no

        if not hasattr(self, 'depend_on_prev_state'):
            depend_on_prev_state = False 
        else:
            depend_on_prev_state = True 
        write_exec_hist(self, type(self).__name__, userdata, depend_on_prev_state )

        global mode_no_state_trainsition_report
        if not mode_no_state_trainsition_report:
            hmm_state_switch_client(state_no)
        ret = original_execute(self, userdata)
        if not mode_no_state_trainsition_report:
            hmm_state_switch_client(0)


        global event_flag
        if event_flag == -1:
            event_flag = 1
            rospy.loginfo("UnBlock anomlay detection")
            send_image(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'green.jpg'))

        return ret
    return f

class Go_to_Pick_Hover_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        
    @execute_decorator   
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        hover_pick_object_pose = hardcoded_data.hover_pick_object_pose
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_pose_point(hover_pick_object_pose, 4.0)
        traj.start()
        traj.wait(5)

        return 'Successful'
    
class Go_to_Pick_Position(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Successful', 'NeedRecovery'])
        self.state_no = 2
        self.depend_on_prev_state = True
        
    @execute_decorator   
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        traj.gripper_open()
        
        # make gripper dive vertically to approach the object
        traj.clear('right')
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)

        pick_object_pose = hardcoded_data.pick_object_pose
        
        tmp_position = copy.deepcopy(pick_object_pose)
        tmp_position.position.z = pick_object_pose.position.z + hardcoded_data.hover_distance*3/4
        traj.add_pose_point(tmp_position, 1.0)
        
        tmp_position.position.z = pick_object_pose.position.z + hardcoded_data.hover_distance*2/4
        traj.add_pose_point(tmp_position, 2.0)
        
        tmp_position.position.z = pick_object_pose.position.z + hardcoded_data.hover_distance*1/4
        traj.add_pose_point(tmp_position, 3.0)
    
        traj.add_pose_point(pick_object_pose, 4.0)

        traj.start()
        if wait_for_motion_and_detect_anomaly(traj):
            return 'NeedRecovery'    

        # grab the object
        traj.gripper_close()

        return 'Successful'

class Go_to_Pick_Hover_Position_Again(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Successful', 'NeedRecovery'])
        self.state_no = 5
        
        
    @execute_decorator   
    def execute(self, userdata):
        global limb
        global traj
        global limb_interface
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        hover_pick_object_pose = hardcoded_data.hover_pick_object_pose
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_pose_point(hover_pick_object_pose, 4.0)
        traj.start()
        if wait_for_motion_and_detect_anomaly(traj):
            return 'NeedRecovery'    

        return 'Successful'
    
class Recovery(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes)
        
    def execute(self, userdata):
        global event_flag
        global execution_history
        global mode_no_state_trainsition_report

        rospy.loginfo("Enter Recovery State...")
        rospy.loginfo("Block anomlay detection")

        send_image(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'red.jpg'))

        history_to_reexecute = None 
        while True:
            if len(execution_history) == 0:
                rospy.loginfo("no execution_history found")
            elif execution_history[-1]['depend_on_prev_states']:
                execution_history.pop()
            else:
                history_to_reexecute = execution_history[-1]
                break

        if history_to_reexecute is None:
            return 'RecoveryFailed'

        state_name = history_to_reexecute['state_name']
        next_state = state_name
        rospy.loginfo('Gonna reenter %s'%(next_state,))

        rospy.loginfo("Block anomlay detection for the next state")
        event_flag = -1
        rospy.sleep(5)
        return 'Reenter_'+next_state

def callback_hmm(msg):
    global event_flag
    # if event_flag is not blocked by Recovery state
    if event_flag != -1:
        event_flag = 0

def callback_manual_anomaly_signal(msg):
    global event_flag
    if event_flag != -1:
        event_flag = 0
        
def shutdown():
    global limb
    global traj
    global lintimb_erface
    rospy.loginfo("Stopping the node...")
    #srv_action_client.delete_gazebo_models()
    traj.clear('right')
    traj.stop()

def calibrate_right_arm():
    rospy.loginfo('calibrating right arm...')
    limb = 'right'
    traj = srv_action_client.Trajectory(limb)
    limb_interface = baxter_interface.limb.Limb(limb)
    traj.add_pose_point(hardcoded_data.hover_pick_object_pose, 4.0)
    traj.start()
    traj.wait(5)

    rospy.sleep(5)
    from std_srvs.srv import Trigger
    trigger = rospy.ServiceProxy('/robotiq_wrench_calibration_service', Trigger)
    resp = trigger()
    rospy.sleep(5)

    rospy.loginfo('done...')
        
def main():
    global mode_no_state_trainsition_report
    global mode_no_anomaly_detection
    global sm


    rospy.init_node("pick_n_place_joint_trajectory")
    rospy.on_shutdown(shutdown)
    if not mode_no_anomaly_detection:
        if mode_use_manual_anomaly_signal:
            rospy.Subscriber("/manual_anomaly_signal", std_msgs.msg.String, callback_manual_anomaly_signal)
        else:
            rospy.Subscriber("/anomaly_detection_signal", std_msgs.msg.Header, callback_hmm)
 


    sm = smach.StateMachine(outcomes=['TaskFailed', 'TaskSuccessful'])

    global traj
    global limb_interface
    global limb
    
    limb = 'right'
    traj = srv_action_client.Trajectory(limb)
    limb_interface = baxter_interface.limb.Limb(limb)
   

    rospy.loginfo('Building state machine...')
    with sm:

        smach.StateMachine.add(
            Go_to_Pick_Hover_Position.__name__,
            Go_to_Pick_Hover_Position(),
            transitions={
                'Successful': Go_to_Pick_Position.__name__,
            }
        )

        smach.StateMachine.add(
			Go_to_Pick_Position.__name__,
			Go_to_Pick_Position(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Successful': Go_to_Pick_Hover_Position_Again.__name__,
            }
        )

        smach.StateMachine.add(
			Go_to_Pick_Hover_Position_Again.__name__,
			Go_to_Pick_Hover_Position_Again(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Successful':'TaskSuccessful'
            }
        )

        # build Recovery states automatically
        recovery_outcomes = ['RecoveryFailed']
        recovery_state_transitions = {
            'RecoveryFailed':'TaskFailed'
        }
        for added_state in sm._states:
            recovery_outcomes.append('Reenter_'+added_state)
            recovery_state_transitions['Reenter_'+added_state] = added_state

        smach.StateMachine.add(
			'Recovery',
			Recovery(outcomes=recovery_outcomes),
            transitions=recovery_state_transitions
        )
    
                           
    rospy.loginfo('Done...')

    send_image(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'green.jpg'))

    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')
    sis.start()

    calibrate_right_arm()
    if not mode_no_state_trainsition_report:
        hmm_state_switch_client(0)
    outcome = sm.execute()
    if not mode_no_state_trainsition_report:
        hmm_state_switch_client(0)

    rospy.spin()
    

if __name__ == '__main__':
    mode_no_state_trainsition_report = False
    mode_no_anomaly_detection = False 
    mode_use_manual_anomaly_signal = False 
    sys.exit(main())







