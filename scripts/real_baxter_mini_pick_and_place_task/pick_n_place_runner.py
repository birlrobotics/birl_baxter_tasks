#!/usr/bin/env python
import baxter_interface
from birl_baxter_tasks.srv import *

import rospy

import smach_ros


def shutdown():
    pass

if __name__ == '__main__':
    rospy.init_node("pick_n_place_joint_trajectory")
    rospy.on_shutdown(shutdown)

    from task_states import assembly_user_defined_sm
    sm = assembly_user_defined_sm()
    from introspection_framework.interface import modify_user_defined_sm, start_instrospection

    sm = modify_user_defined_sm(sm)

    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')
    sis.start()

    start_instrospection()

    outcome = sm.execute()

    rospy.spin()







