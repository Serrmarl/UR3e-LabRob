# UR3e-LabRob
Customized packages for UR3e robot control with Hand-e robotiq gripper. Various routines are explored to create HRI activities.

## Demostration
**Video**
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
Then, in other command window, execute any routine from the "routine" folder
```
rosrun ur_control moveit_[ROUTINE_NAME]
```
### Usage with real hardware
To run the routines on the real robot we will have to start the drivers
```
roslaunch ur_robot_driver ur3e_handler.launch robot_ip:=[ROBOT_IP_ASSIGNED] kinematics_config_:="[CONFIG_FILE_PATH]
```
Then, in other command window, load the Moveit configuration:
```
roslaunch ur_hande_moveit_config start_moveit.launch
```
Then, in other command window, execute any routine from the "routine" folder
```
rosrun ur_control moveit_[ROUTINE_NAME]
```
