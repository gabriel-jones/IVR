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


class control:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('control', anonymous=True)

    # initialize a publisher to send joint angles to a topic named joints_angle


    self.joint_angle_1 = rospy.Subscriber("joint_angle_1", Float64, self.callback1)
    self.joint_angle_3 = rospy.Subscriber("joint_angle_3", Float64, self.callback2)
    self.joint_angle_4 = rospy.Subscriber("joint_angle_4", Float64, self.callback3)


    self.Cords = rospy.Publisher("cords", Float64MultiArray, queue_size=10)

    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  def FK(self):
     #The DH values are as follows
    #           a    |   α    |  d    |   θ
    # Link1     0        90       4.0     Taken from Angles
    # Link2    0         -90     3.2     Taken from Angles
    # Link3     0        0    2.8     Taken from Angles
    print(self.V1.data)
    print(self.V2.data)
    print(self.V3.data)

    L1 = np.array([ 0, (np.pi / 2), 4.0, self.V1.data])

    L2 = np.array([ 0, (np.pi / 2), 3.2, self.V2.data])

    L3 = np.array([ 0, 0, 2.8, self.V3.data])


    Vec1 = np.matrix([[np.cos(L1[3]), -np.sin(L1[3])*np.cos(L1[1]), np.sin(L1[3])*np.sin(L1[1]), 0*np.cos(L1[3])], [np.sin(L1[3]), np.cos(L1[3]) * np.cos(L1[1]),  -np.cos(L1[3])*np.sin(L1[1]), 0*np.sin(L1[3])], [0, np.sin(L1[1]), np.cos(L1[1]), L1[2]], [0, 0, 0, 1]])
    Vec2 = np.matrix([[np.cos(L2[3]), -np.sin(L2[3])*np.cos(L2[1]), np.sin(L2[3])*np.sin(L2[1]), 0*np.cos(L2[3])], [np.sin(L2[3]), np.cos(L2[3]) * np.cos(L2[1]),  -np.cos(L2[3])*np.sin(L2[1]), 0*np.sin(L2[3])], [0, np.sin(L2[1]), np.cos(L2[1]), L2[2]], [0, 0, 0, 1]])
    Vec3 = np.matrix([[np.cos(L3[3]), -np.sin(L3[3])*np.cos(L3[1]), np.sin(L3[3])*np.sin(L3[1]), 0*np.cos(L3[3])], [np.sin(L3[3]), np.cos(L3[3]) * np.cos(L3[1]),  -np.cos(L3[3])*np.sin(L3[1]), 0*np.sin(L3[3])], [0, np.sin(L3[1]), np.cos(L3[1]), L3[2]], [0, 0, 0, 1]])

    Test = np.matmul(Vec1, Vec2)
    Final = np.matmul(Test, Vec3)

    print(Final[:,3])
    smoke = Float64MultiArray()
    smoke.data = Final[:,3]
    return smoke


  # Recieve data from estimated joint angles in vision_2
  def callback1(self,data):

    self.V1 = Float64()
    self.V1 = data
    self.Je = Float64MultiArray
    Je = self.FK()
    self.Cords.publish(Je)

  def callback2(self,data):

    self.V2 = Float64()
    self.V2 = data
    self.Je = Float64MultiArray
    Je = self.FK()
    self.Cords.publish(Je)

  def callback3(self,data):

    self.V3 = Float64()
    self.V3 = data
    self.Je = Float64MultiArray
    Je = self.FK()
    self.Cords.publish(Je)
# call the class

def main(args):
  j = control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
