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
    self.image_sub = rospy.Subscriber("/left/image_rect_color",Image, self.callback)
    self.image_pub = rospy.Publisher("/image_output",Image, queue_size = 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image = self.image_method(cv_image)

    is_bgr = len(cv_image.shape) == 3
   
    try:
      if is_bgr:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      else:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
    except CvBridgeError as e:
      print(e)

  def image_method(self, img):   

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    return gray



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
