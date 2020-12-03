#!/usr/bin/env python3
# ROS Imports
import rospy

from tf.transformations import quaternion_matrix
# x,y,z,w
matrix = quaternion_matrix([0, 0, 0.383, 0.924])
print(matrix)
