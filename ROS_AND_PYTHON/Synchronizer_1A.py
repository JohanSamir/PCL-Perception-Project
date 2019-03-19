#!/usr/bin/env python
# coding=utf-8

import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import sys
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField


class synchronizer:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = message_filters.Subscriber('/left/image_rect_color', Image)
		self.pc_sub = message_filters.Subscriber('velodyne_points', PointCloud2)
		self.pointcloudpu = rospy.Publisher("/pointcloud_esta_SI",PointCloud2, queue_size = 1)

		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.pc_sub], 10, 0.1, allow_headerless=False)
		self.ts.registerCallback(self.callback)
		
	def callback(self, image, pointcloud):
		cont = image.header.seq
		# cont+=1
		#self.pointcloudpu.publish(pointcloud)
		nombre_img = "img"+str(cont)+".png"
		#nombre_pointcloud = "pointcloud"+str(cont)+".pcd"

		#print nombre_img,nombre_pointcloud
		print(pointcloud.data)

		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imwrite("/home/johan/Desktop/Jackal/Images/"+nombre_img,cv_image)


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