#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Baxter RSDK Joint Trajectory Example: file playback
"""

import argparse
import operator
import sys
import threading

from bisect import bisect
from copy import copy
from os import path

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class Trajectory(object):
    def __init__(self,limb="right"):
        #create our action server clients
        self._right_client = actionlib.SimpleActionClient( 
            'robot/limb/right/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )

        #verify joint trajectory action servers are available
        
        r_server_up = self._right_client.wait_for_server(rospy.Duration(10.0))
        if not r_server_up:
            msg = ("Action server not available."
                   " Verify action server availability.")
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)
        #create our goal request
        self._limb_name = limb
        self._gripper = baxter_interface.Gripper(limb)
        self._r_goal = FollowJointTrajectoryGoal()

        #limb interface - current angles needed for start move
        
        self._r_arm = baxter_interface.Limb('right')

        #gripper interface - for gripper command playback
        

        #flag to signify the arm trajectories have begun executing
        self._arm_trajectory_started = False
        #reentrant lock to prevent same-thread lockout
        self._lock = threading.RLock()

        # Verify Grippers Have No Errors and are Calibrated

        #gripper goal trajectories


        # Timing offset to prevent gripper playback before trajectory has started
        self._slow_move_offset = 0.0
        self._trajectory_start_offset = rospy.Duration(0.0)
        self._trajectory_actual_offset = rospy.Duration(0.0)

        #param namespace
        self._param_ns = '/rsdk_joint_trajectory_action_server/'

        #gripper control rate
        self._gripper_rate = 20.0  # Hz
    
    def gripper_open(self):
        self._gripper.open()

    def gripper_close(self):
        self._gripper.close()
    
    def _execute_gripper_commands(self):
        start_time = rospy.get_time() - self._trajectory_actual_offset.to_sec()
        r_cmd = self._r_grip.trajectory.points
        pnt_times = [pnt.time_from_start.to_sec() for pnt in r_cmd]
        end_time = pnt_times[-1]
        rate = rospy.Rate(self._gripper_rate)
        now_from_start = rospy.get_time() - start_time
        while(now_from_start < end_time + (1.0 / self._gripper_rate) and
              not rospy.is_shutdown()):
            idx = bisect(pnt_times, now_from_start) - 1

            rate.sleep()
            now_from_start = rospy.get_time() - start_time

    def _clean_line(self, line, joint_names):
        """
        Cleans a single line of recorded joint positions

        @param line: the line described in a list to process
        @param joint_names: joint name keys

        @return command: returns dictionary {joint: value} of valid commands
        @return line: returns list of current line values stripped of commas
        """
        def try_float(x):
            try:
                return float(x)
            except ValueError:
                return None
        #convert the line of strings to a float or None
        line = [try_float(x) for x in line.rstrip().split(',')]
        #zip the values with the joint names
        combined = zip(joint_names[1:], line[1:])
        #take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        #convert it to a dictionary with only valid commands
        command = dict(cleaned)
        return (command, line,)

    def result(self):
        return self._client.get_result()
        
    def stop(self):
        self._right_client.cancel_goal()
        
    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)
        
    def _add_point(self, positions, side, time):
        """
        Appends trajectory with new point

        @param positions: joint positions
        @param side: limb to command point
        @param time: time from start for point in seconds
        """
        #creates a point in trajectory with time_from_start and positions
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        if side == 'right':
            self._r_goal.trajectory.points.append(point)
        elif side == 'right_gripper':
            self._r_grip.trajectory.points.append(point)

    def parse_file(self, filename):
        """
        Parses input file into FollowJointTrajectoryGoal format

        @param filename: input filename
        """
        #open recorded file
        with open(filename, 'r') as f:
            lines = f.readlines()
        #read joint names specified in file
        joint_names = lines[0].rstrip().split(',')
        #parse joint names for the left and right limbs
        for name in joint_names:
            if 'right' == name[:-3]:
                self._r_goal.trajectory.joint_names.append(name)

        def find_start_offset(pos):
            #create empty lists
            cur = []
            cmd = []
            dflt_vel = []
            vel_param = self._param_ns + "%s_default_velocity"
            #for all joints find our current and first commanded position
            #reading default velocities from the parameter server if specified
            for name in joint_names:
                if 'right' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._r_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
            diffs = map(operator.sub, cmd, cur)
            diffs = map(operator.abs, diffs)
            #determine the largest time offset necessary across all joints
            offset = max(map(operator.div, diffs, dflt_vel))
            return offset

        for idx, values in enumerate(lines[1:]):
            #clean each line of file # lines[1:] record all the data except the first line
            cmd, values = self._clean_line(values, joint_names)
            #find allowable time offset for move to start position
            if idx == 0:
                # Set the initial position to be the current pose.
                # This ensures we move slowly to the starting point of the
                # trajectory from the current pose - The user may have moved
                # arm since recording                
                cur_cmd = [self._r_arm.joint_angle(jnt) for jnt in self._r_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'right', 0.0)
                start_offset = find_start_offset(cmd)
                # Gripper playback won't start until the starting movement's
                # duration has passed, and the actual trajectory playback begins
                self._slow_move_offset = start_offset
                self._trajectory_start_offset = rospy.Duration(start_offset + values[0])
            #add a point for this set of commands with recorded time
            cur_cmd = [cmd[jnt] for jnt in self._r_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'right', values[0] + start_offset)
            
            

    def _feedback(self, data):
        # Test to see if the actual playback time has exceeded
        # the move-to-start-pose timing offset
        if (not self._get_trajectory_flag() and
              data.actual.time_from_start >= self._trajectory_start_offset):
            self._set_trajectory_flag(value=True)
            self._trajectory_actual_offset = data.actual.time_from_start

    def _set_trajectory_flag(self, value=False):
        with self._lock:
            # Assign a value to the flag
            self._arm_trajectory_started = value

    def _get_trajectory_flag(self):
        temp_flag = False
        with self._lock:
            # Copy to external variable
            temp_flag = self._arm_trajectory_started
        return temp_flag

    def start(self):
        """
        Sends FollowJointTrajectoryAction request
        """
        
        self._right_client.send_goal(self._r_goal, feedback_cb=self._feedback)
#        rospy.info()
        # Syncronize playback by waiting for the trajectories to start
        while not rospy.is_shutdown() and not self._get_trajectory_flag():
            rospy.sleep(0.05)
        



    def wait(self, timeout=15.0):
        return self._right_client.wait_for_result(timeout=rospy.Duration(timeout))

def main():
    """RSDK Joint Trajectory Example: File Playback

    Plays back joint positions honoring timestamps recorded
    via the joint_recorder example.

    Run the joint_recorder.py example first to create a recording
    file for use with this example. Then make sure to start the
    joint_trajectory_action_server before running this example.

    This example will use the joint trajectory action server
    with velocity control to follow the positions and times of
    the recorded motion, accurately replicating movement speed
    necessary to hit each trajectory point on time.
    """
    epilog = """
Related examples:
  joint_recorder.py; joint_position_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        '-f', '--file', metavar='PATH', required=True,
        help='path to input file'
    )
    parser.add_argument(
        '-l', '--loops', type=int, default=1,
        help='number of playback loops. 0=infinite.'
    )
    # remove ROS args and filename (sys.arv[0]) for argparse
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_file_playback")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    traj = Trajectory()
    traj.parse_file(path.expanduser(args.file))
    #for safe interrupt handling
    rospy.on_shutdown(traj.stop)
    result = True
    loop_cnt = 1
    loopstr = str(args.loops)
    if args.loops == 0:
        args.loops = float('inf')
        loopstr = "forever"
    while (result == True and loop_cnt <= args.loops
           and not rospy.is_shutdown()):
        print("Playback loop %d of %s" % (loop_cnt, loopstr,))
        traj.start()
        result = traj.wait()
        loop_cnt = loop_cnt + 1
    print("Exiting - File Playback Complete")

if __name__ == "__main__":
    main()
