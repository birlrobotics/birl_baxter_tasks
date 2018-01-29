#!/usr/bin/env python
import baxter_interface

import rospy

import smach_ros


def shutdown():
    pass

if __name__ == '__main__':
    rospy.init_node("pick_n_place_joint_trajectory")
    rospy.on_shutdown(shutdown)

    from task_states import assembly_user_defined_sm
    sm = assembly_user_defined_sm()
    from smach_based_introspection_framework.interface import modify_user_defined_sm, start_instrospection

    sm = modify_user_defined_sm(sm)

    sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')
    sis.start()

    start_instrospection(
        no_anomaly_detection=False,
        use_manual_anomaly_signal=False,
    )

    outcome = sm.execute()

    rospy.spin()
