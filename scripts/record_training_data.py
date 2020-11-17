#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

"""
Machine learning pipeline


Right now, can build data recording for tracking n closest objects, and setting excess objects to arbitrarily large values
if far away.

publish tf for all objects

eventually, we'll want a topic anyways - it should be fine to just use no tfs
"""


class DataRecorder():
    def __init__(self):
        rospy.init_node("data_recorder")

        # Publishers and Subscribers ------------------------------------------
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states",
            ModelStates, self.modelStateCB)
        self.twist_sub = rospy.Subscriber("/cmd_vel",
            Twist, self.twistCB)

        # ROS messages --------------------------------------------------------
        self.model_state_msg = ModelStates()
        self.twist_msg = Twist()

        # ROS Parameters ------------------------------------------------------
        # Name of the dodgeballs - used to extract model states from gazebo
        self.dodgeball_prefix = rospy.get_param('dodgeball_prefix', 'unit_sphere')
        # Number of closest dodgeballs to keep track of
        self.num_dodgeballs = rospy.get_param('num_dodgeballs', 1)

        # Loop update rate
        rate = rospy.get_param('~rate', 10) # in hz
        self.update_rate = rospy.Rate(rate)

    def modelStateCB(self, msg):
        """ Save incoming model state msg. Does not handle any computation
        to ensure the most recent messages are used """
        self.model_state_msg = msg

    def twistCB(self, msg):
        """ Save incoming twist msg. Does not handle any computation
        to ensure the most recent messages are used """
        self.twist_msg = msg

    def recordDataPoint(self):
        """ Get n closest dodgeballs, save to dataset.

        Recorded Data:
            - human velocity command (magnitude only - assumes 1D case.
            Negative values for backwards, positive for forwards)
            - magnitude of distance to neato (for each dodgeball - defaults to
            1000 if ball is not there)
            - angle of ball movement (0 means the ball is headed directly
             towards the neato, -180 or 180 means the ball is headed
             directly away from the neato) (defaults to 180 if ball is not there)
        """
        pass

    def writeDataToFile(self, filename):
        pass

    def _containsPrefix(self, prefix, name):
        """ Checks an input name and determines whether it contains a prefix """
        if name[:len(prefix)] == prefix:
            return True
        else:
            return False

    def run(self):
        while not rospy.is_shutdown():
            self.update_rate.sleep()

if __name__ == "__main__":
    data_recorder = DataRecorder()
    data_recorder.run()
