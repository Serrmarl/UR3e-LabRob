#!/usr/bin/env python2

#
# This program is a node publisher
# This node publish the position of an object to move in the space.
#

import rospy
import tf2_ros
import geometry_msgs.msg
import math

if __name__ == '__main__':
    # Define node name.
    rospy.init_node('dynamic_tf_broadcaster')
    # Define the objects using ros libraries.
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # Define the reference frame and the name of the frame created.
    t.header.frame_id = "world"
    t.child_frame_id = "object"

    # Define the main loop and rate used to make a little stop time
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

	# Define the x (time) in seconds using the system timer.
        x = rospy.Time.now().to_sec()

	# Define the identifier of the operation.
        t.header.stamp = rospy.Time.now()

	# Define the variation of the position
        t.transform.translation.x = 0.25 * math.sin(x)
        t.transform.translation.y = 0.30
        t.transform.translation.z = 0.22 + (0.05 * math.sin(4 * x))
        t.transform.rotation.x = 1
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 0

	# Comunicate the position of the frame to the system.
        br.sendTransform(t)
        rate.sleep()

