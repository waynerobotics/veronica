#!/usr/bin/env python

import sys
import rospy
import cv2  # To make PyLint recognize cv2
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from math import tan, atan
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from combined_thresh import combined_thresh
from perspective_transform import perspective_transform
from line_fit import line_fit, viz2, calc_curve, final_viz
from time import time
import os

image_topic = '/usb_cam1/image_raw'
count = 0
pi = np.pi
fov = np.radians(20) # blind angle in one quadrant = 90-70 => 140 total field of view. 70 is the actual field of view
xc = (640-1)/2
yc = -480.0 - xc*tan(fov) # image top left corner is (0, 0)


class image2laser:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)
    self.binary_lane_img = rospy.Publisher("binary_lane_img", Image, queue_size=10)
    self.pub=rospy.Publisher('lane_to_scan', LaserScan, queue_size=10)

    self.ls = LaserScan()
    self.ls.header.frame_id = 'map'
    self.ls.angle_increment = 0.02 # For Hokuyo LiDAR
    amin = -pi/2 + fov #20 implies 70 deg field of view in one quadrant
    amax = -float(amin)
    self.ls.angle_max = amax
    self.ls.angle_min = amin
    self.ls.angle_increment = 0.02
    self.ls.time_increment = 0.0001
    self.ls.scan_time = 0.00001
    self.ls.range_min = 0.5
    self.ls.range_max = 4.0



  def laserScan(self, img, xc, yc, inc, amin, amax):
    xm_per_pix = 0.4/250
    ym_per_pix = 0.6/100
    ranges_length = int(2*amax//inc)
    ranges = np.ones([ranges_length], dtype=float)*4.0  #Initialize all to 4 meter range
    ranges = ranges.tolist()
    for i in range(img.shape[1]-1): # width - columns - X
        for j in range(img.shape[0]): #height - rows - Y
            if img[j][i]:
                try:
                  slope = atan((yc+j)/(xc-i))
                except ZeroDivisionError:
                  slope = pi/2
                angle = (-1)**(slope>0)*pi/2 + slope  #*180/pi
                r = (((yc+j)*ym_per_pix)**2 + ((xc-i)*xm_per_pix)**2)**0.5
                rangeIndex = min(ranges_length - 1, int((angle + amax)//inc))
                # print(i,j,rangeIndex)
                if r < ranges[rangeIndex]:
                  ranges[rangeIndex] = r
    return ranges


  def callback(self,data):
    global count
    try:
      img = self.bridge.imgmsg_to_cv2(data, desired_encoding = "bgr8")
      
      if img.dtype == 'float32':
        img = np.array(img)*255
        img = np.uint8(img)
      
      img = cv2.blur(img, (5,5))  
      img, _, _, _, _ = combined_thresh(img)
      img, _, _, _ = perspective_transform(img)
      
      self.binary_lane_img.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
      self.ls.ranges = self.laserScan(img, xc, yc, self.ls.angle_increment, self.ls.angle_min, self.ls.angle_max)
      self.ls.header.stamp.secs = rospy.get_rostime().secs
      self.ls.header.stamp.nsecs = rospy.get_rostime().nsecs
      self.pub.publish(self.ls)

      # cv2.imshow("Image window", img.astype(np.float32))
      # count += 1
      # k = cv2.waitKey(1)
      # if k == 113: #q
      #     cv2.imwrite("saves/"+str(count)+"_igvcw.png", img)
      # if k == 27: #esc
      #     cv2.destroyAllWindows()
    except CvBridgeError as e:
      print(e)
    

def main():
    rospy.init_node('lane2laser', anonymous=True)
    il = image2laser()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
