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
    
class StateSwitchProc(multiprocessing.Process):
    def __init__(self, com_queue):
        multiprocessing.Process.__init__(self)     
        self.com_queue = com_queue

    def callback(self, req):
        timestamp = rospy.Time.now()
        self.com_queue.put([req.state, timestamp])       
        resp = State_SwitchResponse()
        resp.finish.data = True
        return resp

    def run(self):
        # set up Subscribers
        rospy.init_node("StateSwitchProc_node", anonymous=True)
        rospy.Service('hmm_state_switch', State_Switch, self.callback)
        while not rospy.is_shutdown():
            rospy.spin()


    
class TopicReceiverProc(multiprocessing.Process):
    def __init__(self, topic_name, topic_type, com_queue):
        multiprocessing.Process.__init__(self)     

        self.topic_name = topic_name
        self.topic_type = topic_type
        self.com_queue = com_queue

    def callback(self, data):
        self.com_queue.put(data)       

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
    if range_of_stamp < 0.05:
        rospy.loginfo('range_of_stamp %s, which is good.'%range_of_stamp)
        return True
    else:
        rospy.loginfo('range_of_stamp %s, which is bad.'%range_of_stamp)
        return False

def main():

    publishing_rate = 100
    



    list_of_topics_to_merge = [
        ["/robot/limb/right/endpoint_state", EndpointState],
        ["/robot/joint_states", JointState],
        ["/robotiq_force_torque_wrench", WrenchStamped],
    ]
    endpoint_state_idx = 0 
    joint_state_idx = 1
    force_sensor_idx = 2

    list_of_proc = []
    list_of_com_queue = []
    for i in list_of_topics_to_merge:
        com_queue = Queue()
        proc = TopicReceiverProc(i[0], i[1], com_queue)
        proc.start()
        list_of_proc.append(proc)
        list_of_com_queue.append(com_queue)

    state_com_queue = Queue()
    state_proc = StateSwitchProc(state_com_queue)
    state_proc.start()

    rospy.init_node("topic_multimodal_multiproc", anonymous=True)
    pub = rospy.Publisher("/tag_multimodal",Tag_MultiModal, queue_size=10)

    r = rospy.Rate(publishing_rate)

    tag_multimodal = Tag_MultiModal()

    stamp_max_diff = 0.1
    base_idx = 0
    queue_amount = len(list_of_com_queue)

    hmm_state = 0
    list_of_new_hmm_state = [] 
    list_of_new_hmm_state_ts = [] 
    while not rospy.is_shutdown():
        data_to_send = [None]*queue_amount
        while True:
            if data_to_send[base_idx] is None:
                base_data = list_of_com_queue[base_idx].get(True)
            else:
                base_data = data_to_send[base_idx]
            data_to_send[base_idx] = base_data

            ok_to_send = True
            base_stamp = base_data.header.stamp
            now_idx = (base_idx+1)%queue_amount
            while now_idx != base_idx:
                if data_to_send[now_idx] is None:
                    now_data = list_of_com_queue[now_idx].get(True)
                else:
                    now_data = data_to_send[now_idx]
                now_stamp = now_data.header.stamp

                stamp_diff = (now_stamp-base_stamp).to_sec()

                while stamp_diff < 0:
                    now_data = list_of_com_queue[now_idx].get(True)
                    now_stamp = now_data.header.stamp
                    stamp_diff = (now_stamp-base_stamp).to_sec()

                if stamp_diff < stamp_max_diff:
                    data_to_send[now_idx] = now_data 
                else:
                    rospy.loginfo("stamp_diff %s, which is bad"%stamp_diff)
                    ok_to_send = False
                    rospy.loginfo("base_idx from %s to %s"%(base_idx, now_idx))
                    base_idx = now_idx
                    break

                now_idx = (now_idx+1)%queue_amount
        
            if ok_to_send:
                break
            else:
                rospy.loginfo("not ok to send, gonna try new base")
                continue

        if state_com_queue.empty():
            pass
        else:
            new_hmm_state, new_timestamp = state_com_queue.get()
            rospy.loginfo("new hmm state received %s"%new_hmm_state)
            list_of_new_hmm_state.append(new_hmm_state)
            list_of_new_hmm_state_ts.append(new_timestamp) 

        if len(list_of_new_hmm_state) != 0:
            new_hmm_state = list_of_new_hmm_state[0]
            new_timestamp = list_of_new_hmm_state_ts[0]
            if base_data.header.stamp > new_timestamp:
                rospy.loginfo("hmm_state from %s"%hmm_state)
                hmm_state = new_hmm_state
                rospy.loginfo("hmm_state to %s"%hmm_state)
                del list_of_new_hmm_state[0]
                del list_of_new_hmm_state_ts[0]

        tag_multimodal.tag = hmm_state
        tag_multimodal.header = base_data.header

        tag_multimodal.endpoint_state = data_to_send[endpoint_state_idx]
        tag_multimodal.joint_state = data_to_send[joint_state_idx]
        tag_multimodal.wrench_stamped = data_to_send[force_sensor_idx]

        pub.publish(tag_multimodal)

        r.sleep()

    for i in list_of_proc:
        i.terminate()

if __name__ == '__main__':
    sys.exit(main())
