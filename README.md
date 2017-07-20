# birl_baxter_tasks Repository
---

# Purpose Of This Repository
This repository contains codes that are used to control Rethink Robotics Baxter in several real or simulated tasks.

The tasks are:
- simulated
  - place box
  - place snap
  - pick and place box
    - codes of these 3 tasks can be found in ./scripts/sim_baxter_pick_and_place_task/
- real
  - pick and place box
    - codes of this task can be found in ./scripts/real_baxter_pick_and_place_task
  - open drawer (use DMP to generate motions)
    - codes of this task can be found in ./scripts/real_baxter_open_drawer_task

# How To Run These Tasks
## To Run Simulated Tasks
1. install dependencies
   - [baxter_sdk](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) 
   - [baxter_simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)

1. catkin_make this package

1. select a task-specific launch file in ./launch, then use roslaunch to launch the task. For example, use the following command to launch the place snap task:  

    ```bash
    roslaunch birl_sim_examples place_snap.launch
    ```
    and the Gazebo simulator will show up, you will see something like this:

    ![place snap task](https://github.com/birlrobotics/birl_baxter_common/blob/master/media/full.png)

## To Run Real Tasks
1. hardware dependencies
   - Rethink Robotics Baxter
   - An optional force sensor installed on Baxter's right hand so that we can measure gripper force directly. We use a force sensor from robotiq.

1. install dependencies
   - [baxter_sdk](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)
   - The force sensor driver. In our case, it is [robotiq](https://github.com/ros-industrial/robotiq.git).

1. catkin_make this package

1. connect Baxter

1. bring up Baxter's joint trajectory action server:
    ```bash
    rosrun baxter_interface joint_trajectory_action_server.py
    ```

1. bring up force sensor publisher (in our case, robotiq):
    ```bash
    rosrun robotiq_force_torque_sensor rq_sensor 
    ```
1. bring up this node:
    ```bash
    rosrun birl_baxter_tasks real_topic_multimodal.py
    ```
    This node will collect task data that will be needed in later analysis
    
1. run the task
    - If you want to run the pick and place task, use:
        ```bash
        rosrun birl_baxter_tasks pick_n_place_with_5_states_using_joint_trajectory_and_smach_with_recovery.py
        ```
    - If you want to run the open drawer task, use:
        ```bash
        rosrun birl_baxter_tasks open_drawer_smach_test.py
        ```    





