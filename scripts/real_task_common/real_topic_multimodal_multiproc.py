#!/usr/bin/env python
import argparse
import struct
import sys
import copy
import ipdb

import rospy
from std_msgs.msg import (
    Empty,
    Header
)

import copy

from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from birl_baxter_tasks.msg import Tag_MultiModal
from birl_baxter_tasks.srv import (
    State_Switch,
    State_SwitchResponse
)
from control_msgs.msg import FollowJointTrajectoryActionFeedback
import multiprocessing
from multiprocessing import Queue
    
hmm_state = None 
def state_switch_handle(req):
    global hmm_state
    hmm_state = req.state
    rospy.loginfo("state is changed to %d" %req.state)
    resp = State_SwitchResponse()
    resp.finish.data = True
    return resp
    
class TopicReceiverProc(multiprocessing.Process):
    def __init__(self, topic_name, topic_type, com_queue):
        multiprocessing.Process.__init__(self)     

        self.topic_name = topic_name
        self.topic_type = topic_type
        self.com_queue = com_queue

    def callback(self, data):
        # put in only one data. 
        # if it's not consumed yet, 
        # we drop the latest data and wait
        if self.com_queue.empty():
            self.com_queue.put(data)       
        else:
            pass

    def run(self):
        # set up Subscribers
        rospy.init_node("node_listens_%s"%self.topic_name.replace('/', '_'), anonymous=True)
        rospy.Subscriber(self.topic_name, self.topic_type, self.callback)
        rospy.loginfo('%s subscribed'%self.topic_name)

        while not rospy.is_shutdown():
            rospy.spin()

def check_msg_headers_match(list_of_msg):
    sorted_list = sorted(list_of_msg, key=lambda x:x.header.stamp)
    range_of_stamp = (sorted_list[-1].header.stamp-sorted_list[0].header.stamp).to_sec()
    print range_of_stamp 
    if range_of_stamp < 0.05:
        return True
    else:
        return False

def main():
    global hmm_state
    hmm_state = 0

    publishing_rate = 100
    



    list_of_topics_to_merge = [
        ["/robot/limb/right/endpoint_state", EndpointState],
        ["/robot/joint_states", JointState],
        ["/robotiq_force_torque_wrench", WrenchStamped],
        ["/robot/limb/right/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback],
    ]
    endpoint_state_idx = 0 
    joint_state_idx = 1
    force_sensor_idx = 2
    traj_feedbk_idx = 3

    list_of_proc = []
    list_of_com_queue = []
    for i in list_of_topics_to_merge:
        com_queue = Queue()
        proc = TopicReceiverProc(i[0], i[1], com_queue)
        proc.start()
        list_of_proc.append(proc)
        list_of_com_queue.append(com_queue)

    rospy.init_node("topic_multimodal_multiproc", anonymous=True)
    state_switch = rospy.Service('hmm_state_switch', State_Switch, state_switch_handle)
    pub = rospy.Publisher("/tag_multimodal",Tag_MultiModal, queue_size=10)

    r = rospy.Rate(publishing_rate)

    traj_feedbk = FollowJointTrajectoryActionFeedback()
    tag_multimodal = Tag_MultiModal()
    while not rospy.is_shutdown():
        tag_multimodal.tag = hmm_state

        now_header = Header()
        now_header.stamp = rospy.Time.now()
        tag_multimodal.header = now_header

        if not list_of_com_queue[endpoint_state_idx].empty():
            tag_multimodal.endpoint_state = list_of_com_queue[endpoint_state_idx].get()
            

        if not list_of_com_queue[joint_state_idx].empty():
            tag_multimodal.joint_state = list_of_com_queue[joint_state_idx].get()

        if not list_of_com_queue[force_sensor_idx].empty():
            tag_multimodal.wrench_stamped = list_of_com_queue[force_sensor_idx].get()

        if not list_of_com_queue[traj_feedbk_idx].empty():
            traj_feedbk = list_of_com_queue[traj_feedbk_idx].get()

        if check_msg_headers_match([tag_multimodal.endpoint_state, tag_multimodal.joint_state, traj_feedbk]):
            pub.publish(tag_multimodal)

        r.sleep()

    for i in list_of_proc:
        i.terminate()

if __name__ == '__main__':
    sys.exit(main())
