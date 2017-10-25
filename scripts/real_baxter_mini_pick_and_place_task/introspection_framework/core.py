from constant import (
    ANOMALY_DETECTED,
    ANOMALY_DETECTION_BLOCKED, 
    ANOMALY_NOT_DETECTED,
)
import smach
import os
import rospy

mode_no_state_trainsition_report = False 
event_flag = 1
execution_history = []

def get_event_flag():
    global event_flag
    return event_flag

def set_event_flag(value):
    global event_flag
    event_flag = value

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

def send_image(path):
    import cv2
    import cv_bridge
    from sensor_msgs.msg import (
        Image,
    )
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'image', path))
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.


def hmm_state_switch_client(state):
    global mode_no_state_trainsition_report
    if mode_no_state_trainsition_report:
        print 'mode_no_state_trainsition_report'
        return
    rospy.wait_for_service('hmm_state_switch')
    try:

        from birl_baxter_tasks.srv import State_Switch, State_SwitchRequest
        hmm_state_switch_proxy = rospy.ServiceProxy('hmm_state_switch',
                                                    State_Switch)
        req = State_SwitchRequest()
        req.state = state
        resp = hmm_state_switch_proxy(req)
        if resp.finish.data:
            print "Hmm State switch to %d succesfully" %state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

class AnomalyDiagnosis(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes)
    def execute(self, userdata):
        hmm_state_switch_client(-1)
        send_image('red.jpg')
        rospy.sleep(5)
        return 'GoToRollBackRecovery'


class HumanTeachingRecovery(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes)
    def execute(self, userdata):
        hmm_state_switch_client(-3)
        rospy.sleep(5)
        return 'RecoveryDone'
    
class RollBackRecovery(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes)
        
    def execute(self, userdata):
        global execution_history
        hmm_state_switch_client(-2)

        rospy.loginfo("Enter RollBackRecovery State...")
        rospy.loginfo("Block anomlay detection")

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
        set_event_flag(ANOMALY_DETECTION_BLOCKED)
        rospy.sleep(5)
        return 'Reenter_'+next_state
