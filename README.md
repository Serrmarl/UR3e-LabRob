# UR3e-LabRob
Customized packages for UR3e robot control with Hand-e robotiq gripper. Various routines are explored to create dynamics trayectories and HRI activities.

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
Then, in other command window, execute any routine from the "routine" folder
```
rosrun ur_control [ROUTINE_NAME]
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
rosrun ur_control [ROUTINE_NAME]
```
