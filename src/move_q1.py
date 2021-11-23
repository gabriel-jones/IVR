#!/usr/bin/env python3

import rospy
import roslib
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray


def start():
	rospy.init_node('move_publisher', anonymous=True)
	rate = rospy.Rate(50)
	
	joint2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=10)
	joint3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=10)
	joint4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=10)
	target_angles_pub = rospy.Publisher('target_angles', Float64MultiArray, queue_size=10)
	
	t0 = rospy.get_time()
	while not rospy.is_shutdown():
		t = np.array([ rospy.get_time() ]) - t0
		j2 = (np.pi / 2) * np.sin(t * np.pi / 15)
		j3 = (np.pi / 2) * np.sin(t * np.pi / 20)
		j4 = (np.pi / 2) * np.sin(t * np.pi / 18)
		
		joint2_pub.publish(j2)
		joint3_pub.publish(j3)
		joint4_pub.publish(j4)
		
		a = Float64MultiArray()
		a.data = np.array([ j2, j3, j4 ])
		target_angles_pub.publish(a)
		
		rate.sleep()


if __name__ == '__main__':
	try:
		start()
	except rospy.ROSInterruptException:
		pass

