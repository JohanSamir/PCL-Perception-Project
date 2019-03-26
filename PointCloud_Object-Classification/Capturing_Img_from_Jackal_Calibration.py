#!/usr/bin/env python
# coding=utf-8

import sys
import rospy

import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/left/image_raw_color",Image, self.callback)

  def callback(self,data):
    cont = data.header.seq
    nombre_img = "img"+str(cont)+".png"

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    is_bgr = len(cv_image.shape) == 3
    cv2.imwrite("/home/johan/Desktop/Jackal/Images_Calibration/"+nombre_img,cv_image)

def main(args):
  rospy.init_node('image_example_node', anonymous=True)
  rospy.loginfo("image_example_node on")
  ic = image_converter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)