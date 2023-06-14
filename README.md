# UR3e-LabRob
Customized packages for UR3e robot control with Hand-e robotiq gripper. Various routines are explored to create HRI activities.

## Simulation
To test the routines created in simulation, we will start the gazebo environment:

roslaunch ur3_gazebo ur_gripper_hande_banda.launch ur_robot:=ur3e

Then load the Moveit configuration:

roslaunch ur_hande_moveit_config start_moveit.launch

Then, execute any routine from the "routine" folder

rosrun ur_control moveit_[Routine_Name]

## UR3e
To run the routines on the real robot we will have to start the drivers

roslaunch ur_robot_driver ur3e_handler

Then load the Moveit configuration:

roslaunch ur_hande_moveit_config start_moveit.launch

Then, execute any routine from the "routine" folder

rosrun ur_control moveit_[Routine_Name]
