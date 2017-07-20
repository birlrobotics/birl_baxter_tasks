#!/usr/bin/env python
import os
base_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'DMP_record')


recorded_go_to_start_position_path = os.path.join(base_path, 's1.txt')
generalized_go_to_start_position_path = os.path.join(base_path, 's1_dmp.txt')

recorded_go_to_gripper_position_path = os.path.join(base_path, 's2.txt')
generalized_go_to_gripper_position_path = os.path.join(base_path, 's2_dmp.txt')

recorded_go_back_path = os.path.join(base_path, 's3.txt')
generalized_go_back_path = os.path.join(base_path, 's3_dmp.txt')

recorded_go_forward_path = os.path.join(base_path, "s4.txt")
generalized_go_forward_path = os.path.join(base_path, "s4_dmp.txt")

recorded_go_back_to_start_position_path = os.path.join(base_path, "s5.txt")
generalized_go_back_to_start_position_path = os.path.join(base_path, "s5_dmp.txt")

