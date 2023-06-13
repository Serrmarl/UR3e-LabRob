#!/usr/bin/env python

"""
UR Joint Position Example: keyboard
"""
import argparse

import rospy
import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion

from ur_control.arm import Arm
from ur_control import transformations
from robotiq_msgs.msg import CModelCommand

import getch

import numpy as np
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)


def map_keyboard():

    # Instantiate coammand to control the Hande
    command = CModelCommand();

    def print_robot_state():
        print("Joint angles:", np.round(arm.joint_angles(), 4).tolist())
        print("EE Pose:", np.round(arm.end_effector(), 5).tolist())
        if arm.gripper:
            print("Gripper position:", np.round(arm.gripper.get_position(), 4))

    def set_j(joint_name, sign):
        global delta_q
        current_position = arm.joint_angles()
        current_position[joint_name] += delta_q * sign
        arm.set_joint_positions_flex(current_position, t=0.25)

    def update_d(delta, increment):
        if delta == 'q':
            global delta_q
            delta_q += np.deg2rad(increment)
            print(("delta_q", np.rad2deg(delta_q)))
        if delta == 'x':
            global delta_x
            delta_x += increment
            print(("delta_x", delta_x))

    def set_pose_ik(dim, sign):
        global delta_x
        global delta_q

        x = arm.end_effector()
        delta = np.zeros(6)

        n = 500
        dt = 0.25/float(n)

        if dim <= 2:  # position
            delta[dim] += delta_x * sign / 0.25
        else:  # rotation
            delta[dim] += delta_q * sign / 0.25

        for _ in range(n):
            x = transformations.pose_from_angular_velocity(x, delta, dt=dt, ee_rotation=relative_ee)

        arm.set_target_pose_flex(pose=x, t=0.25)

    def open_gripper():
        command.rPR = 0
	pub.publish(command)

    def close_gripper():
        command.rPR = 255
	pub.publish(command)

    def speed_gripper(delta):
        if delta == 25:
            command.rSP += 25
            if command.rSP > 255:
                command.rSP = 255
                pub.publish(command)
        if delta == -25:
            command.rSP -= 25
            if command.rSP < 0:
                command.rSP = 0
                pub.publish(command)

    def force_gripper(delta):
        if delta == 25:
            command.rFR += 25
            if command.rFR > 255:
                command.rFR = 255
                pub.publish(command)
        if delta == -25:
            command.rFr -= 25
            if command.rFR < 0:
                command.rFR = 0
                pub.publish(command)

    global delta_q
    global delta_x
    delta_q = np.deg2rad(1.0)
    delta_x = 0.005
    target = np.zeros(7)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    target = arm.end_effector()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            past = rospy.Time.now() - rospy.Duration(0)
            trans = tfBuffer.lookup_transform_full(
                target_frame = 'world',
                target_time = rospy.Time.now(),
                source_frame = 'object',
                source_time = past,
                fixed_frame = 'world',
                timeout = rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        target[:3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z + 0.15]
	object_tf = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
	object_msg = Quaternion(object_tf[0], object_tf[1], object_tf[2], object_tf[3])
	target[3:] = [object_msg.x, object_msg.y, object_msg.z, object_msg.w]
	arm.set_target_pose_flex(target, t=0.25)

        rate.sleep()


def main():
    """Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        formatter_class=arg_fmt, description=main.__doc__, epilog=epilog)
    parser.add_argument(
        '--relative', action='store_true', help='Motion Relative to ee')
    parser.add_argument(
        '--namespace', type=str, help='Namespace of arm (useful when having multiple arms)', default=None)
    parser.add_argument(
        '--gripper', action='store_true', help='enable gripper commands')
    parser.add_argument(
        '--robot', type=str, help='Version of Universal Robot arm. Default="ur3e"', default='ur3e')

    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("joint_position_keyboard", log_level=rospy.INFO)
    global pub
    pub = rospy.Publisher('/gripper/command', CModelCommand, queue_size=3)	

    global relative_ee
    relative_ee = args.relative

    ns = ''
    joints_prefix = None
    robot_urdf = "ur3e"
    rospackage = None
    tcp_link = None
    use_gripper = args.gripper

    global arm
    arm = Arm(ft_sensor=False,
              gripper=use_gripper, namespace=ns,
              joint_names_prefix=joints_prefix,
              robot_urdf=robot_urdf, robot_urdf_package=rospackage,
              ee_link=tcp_link)

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()
