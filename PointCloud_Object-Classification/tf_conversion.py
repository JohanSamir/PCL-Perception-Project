#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

#This code finds the transformation from Velodyne to
#zed_left_camera_frame using ROS libraries

if __name__ == '__main__':

	rospy.init_node('velodyne_tf_listener')

	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/velodyne', '/zed_left_camera_frame', rospy.Time(0))
			print('trans: ', trans, 'rot: ', rot)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		rate.sleep()