#!/usr/bin/env python2

#
# This is one of the proposed solutions for follow a dynamic target (Circle)
#

import argparse
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from robotiq_control.controller import Robotiq
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import signal


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to a planning group (group of joints).
        # This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = move_group.get_planning_frame()
        self.eef_link = move_group.get_end_effector_link()
        self.group_names = robot.get_group_names()

	# Setting Max Velocity and acceleration.
	self.move_group.set_max_acceleration_scaling_factor(0.1)
        self.move_group.set_max_velocity_scaling_factor(0.3)

    def display_basic_info(self):
        # We can get the name of the reference frame for this robot:
        print "============ Planning frame: %s" % self.planning_frame


        # We can get a list of all the groups in the robot:
        print "============ Available Planning Groups:", self.group_names

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""
	print self.move_group.get_current_pose()

    def go_to_joint_state(self, a, b, c, d, e, f):
        # Copy class variables to local variables to make the web tutorials more clear.
        move_group = self.move_group

        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = np.deg2rad(a)
        joint_goal[1] = np.deg2rad(b)
        joint_goal[2] = np.deg2rad(c)
        joint_goal[3] = np.deg2rad(d)
        joint_goal[4] = np.deg2rad(e)
        joint_goal[5] = np.deg2rad(f)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.set_joint_value_target(joint_goal)

        plan = move_group.plan()
        move_group.go(wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        print("Current pose", move_group.get_current_pose().pose)
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, a, b, c):
        # Copy class variables to local variables to make the web tutorials more clear.
        move_group = self.move_group

        # We can plan a motion for this group to a desired pose for the
        # end-effector:
        pose_goal = move_group.get_current_pose().pose
        pose_goal.position.x = a
        pose_goal.position.y = b
	pose_goal.position.z = c

        pose_goal.orientation.w = 0

        move_group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
	print(current_pose)
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        move_group = self.move_group

        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through.
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        ##
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        move_group = self.move_group

        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        move_group.execute(plan, wait=True)

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

    def go_to_home(self):
        # This is a copy of go_to_pose_goal
	# But using a knowed pose
        move_group = self.move_group

        home = move_group.get_current_joint_values()
        home[0] = np.deg2rad(22)
        home[1] = np.deg2rad(-75)
        home[2] = np.deg2rad(-81)
        home[3] = np.deg2rad(-110)
        home[4] = np.deg2rad(90)
        home[5] = np.deg2rad(0)

        move_group.set_joint_value_target(home)

        plan = move_group.plan()
        move_group.go(wait=True)

        move_group.stop()

def main():
    #"""Moveit Control Example

    # Defining the object.
    tutorial = MoveGroupPythonIntefaceTutorial()
    rospy.sleep(1)

    # Moving the robot to home posicion
    tutorial.go_to_home()
    rospy.sleep(2)

    # Define the radius of the circle
    r = 0.05

    # Defining the main loop
    while(1):
        # Display the position information
        tutorial.display_basic_info()
	# Define the variables using parametric ecuations and finally move the robot to the position describe by the variables.
        i = rospy.Time.now().to_sec()
        x = -0.362 + (r * math.cos(i))
        y = 0.106 + (r * math.sin(i))
        tutorial.go_to_pose_goal(x, y,  0.3043)


if __name__ == '__main__':
    main()

