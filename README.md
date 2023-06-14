# UR3e-LabRob
Customized packages for UR3e robot control with Hand-e robotiq gripper. We made modifications in the launch files to adapt the scene to the workspace.
Various routines are explored to create dynamics trayectories and HRI activities using MoveIt libraries. This package is based on ROS Melodic using Python programming.

## Demostration
**Dynamic avoidance**

![Dynamic_Avoidance](https://github.com/Serrmarl/UR3e-LabRob/assets/119684013/93efac24-2515-4a94-b3fb-3b956452de7a)

**Tracking**

![Seguimiento_Close](https://github.com/Serrmarl/UR3e-LabRob/assets/119684013/5564088d-62c3-4967-8498-09c28aa24a2c)

**HRI**

![HRI_Close](https://github.com/Serrmarl/UR3e-LabRob/assets/119684013/70cee71d-3f27-4221-9ef2-ffde3e65f9d3)

## Usage
### Simulation
To test the routines created in simulation, we will start the gazebo environment with our modified workspace:
```
roslaunch ur3_gazebo ur_gripper_hande_banda.launch
```
Then, in other command window, load the Moveit configuration:
```
roslaunch ur_hande_moveit_config start_moveit.launch
```
Finally, execute any routine from the "routine" folder writing in another command window:
```
rosrun ur_control [ROUTINE_NAME]
```
>The same files were used to test them in hardware, to execute in simulation we will have to comment the lines referring to the gripper.
>The following line is an example of what should be commented on.
https://github.com/Serrmarl/UR3e-LabRob/blob/7e4ee93054120a467800abe9b35a02d0227e9bb3/src/routines/moveit_HRI1.py#L292
### Usage with real hardware
To run the routines on the real robot we will have to start the drivers
```
roslaunch ur_robot_driver ur3e_handler.launch robot_ip:=[ROBOT_IP] kinematics_config_:="[CONFIG_FILE_PATH]
```
Then, in other command window, load the Moveit configuration:
```
roslaunch ur_hande_moveit_config start_moveit.launch
```
Finally, execute any routine from the "routine" folder writing in another command window:
```
rosrun ur_control [ROUTINE_NAME]
```
