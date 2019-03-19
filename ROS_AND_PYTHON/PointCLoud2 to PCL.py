#!/usr/bin/env python
# coding=utf-8

import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import sys
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import PointCloud2 as pc2
import numpy as np
import laser_geometry.laser_geometry as lg
from sensor_msgs import point_cloud2 as pc2c


class synchronizer:
	def __init__(self):
		self.laserProj = LaserProjection()
		self.bridge = CvBridge()
		self.image_sub = message_filters.Subscriber('/left/image_rect_color', Image)
		self.pc_sub = message_filters.Subscriber('velodyne_points', PointCloud2)
		#self.pointcloudpu = rospy.Publisher("/pointcloud_esta_SI",PointCloud2, queue_size = 1)

		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.pc_sub], 10, 0.1, allow_headerless=False)
		self.ts.registerCallback(self.callback)
		
	def callback(self, image, pointcloud):
		cloud_out = self.laserProj.projectLaser(pointcloud)
		
		cont = image.header.seq
		print('aaaaaaaaaaaaaaaaaa')
		print 'aaaaaaaaaaaaaaaaaa'
		nombre_img = "img"+str(cont)+".png"
		print pointcloud.data
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imwrite("/home/johan/Desktop/Jackal/Images/"+nombre_img,cv_image)

		#pc2_msg = lp.projectLaser(pointcloud)
		#Xg = pc2c.read_points(pc2_msg, skip_nans=True, field_names=("x", "y", "z"))
        #cloud_points = np.empty((pc2_msg.width, 3))
        Xg = pc2c.read_points(cloud_out, skip_nans=True, field_names=("x", "y", "z"))
        cloud_points = np.empty((cloud_out.width, 3))