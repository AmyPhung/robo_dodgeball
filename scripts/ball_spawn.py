#!/usr/bin/env python3
from __future__ import print_function, division
import rospy
from neato_node.msg import Bump
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
import cv2
import math

import os
import time
import tensorflow as tf
import numpy as np

settings = termios.tcgetattr(sys.stdin)
key = None

class ball_spawn(object):
    def __init__(self):
        rospy.init_node('ball_spawn')
        # rospy.Subscriber('/bump', Int8MultiArray, self.process_bump)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_vel = 0.0
        self.angular_vel = 0.0

        naming = "ball"
        straight = {"max_balls": 1, "spawn_rg": (-2,2, 5,5), "target": "straight", "l_vel_rg": (1,1), "a_vel_rg": (0,0), "color": None, "size_rg": (1,1)}
        modes = {"straight":straight}

    def gen_ball_loc(self):
        loc = 0
        return loc

    def gen_ball_vel(self):
        vel = 0
        return vel

    def gen_ball_mess(self):
        ball_mess = None
        return ball_mess

    def pub_ball_mess(self):
        pass

    def cleanup_balls(self):
        """
        Removes balls if they have past the robot.
        returns: True if a new ball should be spawned, False if not
        """
        pass

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    # def process_bump(self, msg):
    #     print(msg)
    #     if any((msg.data[0], msg.data[1], msg.data[2], msg.data[3])):
    #         self.desired_velocity = 0.0
    #         print("stop")

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(x=self.desired_vel), angular=Vector3(z=self.angular_vel)))
            r.sleep()
            key = self.getKey()

            self.pub.publish(Twist(linear=Vector3(x=lin_v), angular=Vector3(z=ang_v)))

if __name__ == '__main__':
    ball_spawner = ball_spawn()
    ball_spawner.run()

