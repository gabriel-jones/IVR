#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


# BGR
C_RED = np.array([0, 0, 1])
C_GREEN = np.array([0, 1, 0])
C_BLUE = np.array([1, 0, 0])
C_YELLOW = np.array([0, 1, 1])

JOINT_LENGTH_GY = 4.0
JOINT_LENGTH_YB = 3.2
JOINT_LENGTH_BR = 2.8
    
def unit_vector(v):
  return v / np.linalg.norm(v)


class JointEstimator:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    
    # initialize a publisher to send joint angles to a topic named joints_angle
    self.joint_angle2_pub = rospy.Publisher("joint_angle_2", Float64, queue_size=10)
    self.joint_angle3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
    self.joint_angle4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)
            
    # initialize subscribers to recieve messages from the camera topics and use callback functions to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
    
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    
    self.joints1 = None
    self.joints2 = None
    
  def detect_joint(self, image, color):
    mask = cv2.inRange(image, color * 100, color * 255)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if M['m00'] == 0:
      return None
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])
    
  def pixel2meters(self, image):
    joint1Pos = self.detect_joint(image, C_GREEN)
    joint2Pos = self.detect_joint(image, C_YELLOW)
    dist = np.sum((joint2Pos - joint1Pos) ** 2)
    return JOINT_LENGTH_GY / np.sqrt(dist)
   
  def detect_positions(self, image, camera_num):
    g = self.detect_joint(image, C_GREEN)
    y = self.detect_joint(image, C_YELLOW)
    b = self.detect_joint(image, C_BLUE)
    r = self.detect_joint(image, C_RED)
    
    prev_joints = self.joints1 if camera_num == 1 else self.joints2
    scale = self.pixel2meters(image)
    conv = lambda pos, i: scale * pos if pos is not None else (prev_joints[i] if prev_joints is not None else None)
    
    joint1Pos = conv(g, 0)
    joint2Pos = conv(y, 1)
    joint3Pos = conv(b, 2)
    joint4Pos = conv(r, 3)
    
    return np.array([ joint1Pos, joint2Pos, joint3Pos, joint4Pos ])
    
  def calculate_angles(self):
    if self.joints1 is None or self.joints2 is None:
      return
    
    extract_pos = lambda i: np.array([ self.joints2[i][0], self.joints1[i][0], (self.joints1[i][1] + self.joints2[i][1]) / 2 ])
    j1 = extract_pos(0) # green
    j2 = extract_pos(1) # yellow
    j3 = extract_pos(2) # blue
    j4 = extract_pos(3) # red
    
    link2 = j3 - j2
    link3 = j4 - j3
    
    nl2 = np.linalg.norm(link2)
    nl3 = np.linalg.norm(link3)
    
    vx = np.array([1, 0, 0])
    vy = np.array([0, 1, 0])
    vz = np.array([0, 0, 1])
    
    y1 = np.cross(link2, vy)
    a2 = np.arccos(np.dot(y1, vx) / (nl2 * np.linalg.norm(vx)))
    
    a3 = np.arccos(np.dot(link2, vy) / (nl2 * np.linalg.norm(vy))) - (np.pi / 2)
    
    c = np.arcsin(np.linalg.norm(np.cross(link2, link3)) / (nl2 * nl3))
    a4 = np.arccos(np.dot(link2, link3) / (nl2 * nl3)) * (-1 if c < 0 else 1)
  
    try:
      self.joint_angle2_pub.publish(a2)
      self.joint_angle3_pub.publish(a3)
      self.joint_angle4_pub.publish(a4)
    except CvBridgeError as e:
      print(e)

  # Recieve data from camera 1, process it, and publish
  # Plane: YZ
  def callback1(self, data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    self.joints1 = self.detect_positions(self.cv_image1, 1)
    self.calculate_angles()

  # Receive data from camera 2, process it
  # Plane: XZ
  def callback2(self, data):
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    self.joints2 = self.detect_positions(self.cv_image2, 2)
    self.calculate_angles()

# call the class
def main(args):
  j = JointEstimator()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


