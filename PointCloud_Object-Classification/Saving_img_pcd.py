#!/usr/bin/env python
# coding=utf-8
#This code save Imgs and PointCloud from rosbag. These
# were saving using ROS and the robotic platform Jackal. 


import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import sys
from cv_bridge import CvBridge, CvBridgeError
import cv2
#from sensor_msgs.msg import PointCloud2 as pc2
import numpy as np
import laser_geometry.laser_geometry as lg
#from sensor_msgs import point_cloud2 as pc2c
import sensor_msgs.point_cloud2 as pc2
import open3d as opn3




class synchronizer:
	def __init__(self):
		#self.laserProj = LaserProjection()
		self.bridge = CvBridge()
		#self.pcd = PointCloud()
		#self.image_sub = message_filters.Subscriber('/left/image_rect_color', Image)
		self.image_sub = message_filters.Subscriber('/left/image_raw_color', Image)
		self.pc_sub = message_filters.Subscriber('velodyne_points', PointCloud2)
		#self.pointcloudpu = rospy.Publisher("/pointcloud_esta_SI",PointCloud2, queue_size = 1)

		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.pc_sub], 10, 0.1, allow_headerless=False)
		self.ts.registerCallback(self.callback)
		
	def callback(self, image, pointcloud):
		#cloud_out = self.laserProj.projectLaser(pointcloud)
		
		cont = image.header.seq
		nombre_img = "img"+str(cont)+".png"
		nombre_point = "points"+str(cont)+".pcd"
		#print pointcloud.data
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imwrite("/home/johan/Desktop/Jackal/Files/Classification/Classification9/Images/"+nombre_img,cv_image)

		lidarPC2 = pc2.read_points(pointcloud)
		lidar = np.array(list(lidarPC2))
		lidarPointsXYZ = lidar[:,0:3]
		print lidarPointsXYZ.shape
		#np.save("/home/johan/Desktop/Jackal/Files/Classification/Classification1/Points/"+nombre_point,lidarPointsXYZ)
		#http://www.open3d.org/docs/tutorial/Basic/file_io.html
		pcd = opn3.PointCloud()
		pcd.points = opn3.Vector3dVector(lidarPointsXYZ)
		opn3.write_point_cloud("/home/johan/Desktop/Jackal/Files/Classification/Classification9/Points/"+nombre_point, pcd)

def main(args):
  rospy.init_node('sync_node', anonymous=True)
  rospy.loginfo("sync_node on")
  sc = synchronizer()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
	
    main(sys.argv)