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


class Control:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('control', anonymous=True)
    
    # initialize a publisher to send joint angles to a topic named joints_angle
    self.joint_angles_sub = rospy.Subscriber("joint_angles", Float64MultiArray, self.callback)
    
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    
    self.vision_angles = None
    self.last_t = np.array([ rospy.get_time() ], dtype='float64')
    
  def jacobian(self, angles):
    ja1, ja3, ja4 =  angles
    return np.array([
      [
        3*np.cos(ja1) + 3*np.cos(ja1+ja3) + 3*np.cos(ja1+ja3+ja4),
        3*np.cos(ja1+ja3) + 3*np.cos(ja1+ja3+ja4) + 3*np.cos(ja1+ja3+ja4)
      ],
      [
        -3*np.sin(ja1) - 3*np.sin(ja1+ja3) - 3*np.sin(ja1+ja3+ja4),
        -3*np.sin(ja1+ja3) - 3*np.sin(ja1+ja3+ja4) - 3*np.sin(ja1+ja3+ja4)
      ]
    ])
    
  def control_open(self):
    t = rospy.get_time()
    dt = t - self.last_t
    self.last_t = t
    
    angles = self.vision_angles
    
    j_inv = np.linalg.pinv(self.jacobian())
    
    pos = self.detect_end_effector()
    pos_d = self.trajectory()
    
    self.error = (pos_d - pos) / dt
    return angles + (dt * np.dot(j_inv, self.error.transpose()))
  
  # Recieve data from estimated joint angles in vision_2
  def callback(self, data):
    self.vision_angles = data

# call the class
def main(args):
  c = Control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


