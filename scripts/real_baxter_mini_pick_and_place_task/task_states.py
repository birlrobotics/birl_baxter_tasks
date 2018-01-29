import smach
import hardcoded_data
import baxter_interface
import rospy
from smach_based_introspection_framework.motion_handler import (
    BreakOnAnomalyTrajectoryClient,
)
import copy

class CalibrateForceSensor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])

    def execute(self, userdata):
        limb = 'right'
        traj = BreakOnAnomalyTrajectoryClient(limb)
        limb_interface = baxter_interface.limb.Limb(limb)

        from geometry_msgs.msg import (
            Pose,
            Quaternion,
        )

        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)

        calibration_pose = Pose()
        calibration_pose.position.x = 0.76301988477
        calibration_pose.position.y = -0.290728116404
        calibration_pose.position.z = -0.0195624201388+0.15
        calibration_pose.orientation = Quaternion(
            x= -0.0259799924463,
            y= 0.999465665097,
            z= 0.00445775211005,
            w= 0.0193275122869,
        )

        traj.add_pose_point(calibration_pose, 4.0)
        traj.start()
        traj.wait(5)
        rospy.sleep(5)
        from std_srvs.srv import Trigger
        try:
            rospy.wait_for_service('/robotiq_wrench_calibration_service', timeout=3)
            trigger = rospy.ServiceProxy('/robotiq_wrench_calibration_service', Trigger)
            resp = trigger()
            rospy.sleep(5)
        except Exception as exc:
            rospy.logerr("calling force sensor calibration failed: %s"%exc)
        return 'Successful'

class GotoPickHoverPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        
    def execute(self, userdata):
        limb = 'right'
        traj = BreakOnAnomalyTrajectoryClient(limb)
        limb_interface = baxter_interface.limb.Limb(limb)
        
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        hover_pick_object_pose = hardcoded_data.hover_pick_object_pose
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_pose_point(hover_pick_object_pose, 4.0)
        traj.start()
        traj.wait(5)

        return 'Successful'


class GoToPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Successful', 'NeedRecovery'])
        self.state_no = 2
        self.depend_on_prev_state = True
        
    def execute(self, userdata):
        limb = 'right'
        traj = BreakOnAnomalyTrajectoryClient(limb)
        limb_interface = baxter_interface.limb.Limb(limb)
        
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

        goal_achieved = traj.wait(5)
        if goal_achieved:
            traj.gripper_close()
            return 'Successful'
        else:
            traj.stop()
            return 'NeedRecovery'    

class GoToPickHoverPositionAgain(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Successful'])
        
    def execute(self, userdata):
        limb = 'right'
        traj = BreakOnAnomalyTrajectoryClient(limb)
        limb_interface = baxter_interface.limb.Limb(limb)
        
        traj.gripper_close()

        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        hover_pick_object_pose = hardcoded_data.hover_pick_object_pose
        traj.clear('right')
        traj.add_point(current_angles, 0.0)
        traj.add_pose_point(hover_pick_object_pose, 4.0)
        traj.start()

        goal_achieved = traj.wait(5)
        if goal_achieved:
            return 'Successful'
        else:
            traj.stop()
            return 'Successful'

def assembly_user_defined_sm():
    sm = smach.StateMachine(outcomes=['TaskFailed', 'TaskSuccessful'])
    with sm:
        smach.StateMachine.add(
            CalibrateForceSensor.__name__,
            CalibrateForceSensor(),
            transitions={
                'Successful': GotoPickHoverPosition.__name__,
            }
        )

        smach.StateMachine.add(
            GotoPickHoverPosition.__name__,
            GotoPickHoverPosition(),
            transitions={
                'Successful': GoToPickPosition.__name__,
            }
        )

        smach.StateMachine.add(
			GoToPickPosition.__name__,
			GoToPickPosition(),
            transitions={
                'NeedRecovery': 'Recovery',
                'Successful': GoToPickHoverPositionAgain.__name__,
            }
        )

        smach.StateMachine.add(
			GoToPickHoverPositionAgain.__name__,
			GoToPickHoverPositionAgain(),
            transitions={
                'Successful':'TaskSuccessful'
            }
        )

    return sm
