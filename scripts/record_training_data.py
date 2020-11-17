#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates

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
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states",
            ModelStates, self.modelStateCB)
        # Name of the dodgeballs - used to extract model states from gazebo
        self.dodgeball_prefix = rospy.get_param('dodgeball_prefix', 'unit_sphere')

        # Loop update rate
        rate = rospy.get_param('~rate', 10) # in hz
        self.update_rate = rospy.Rate(rate)

    def modelStateCB(self, msg):
        """ Extract info for dodgeballs """
        model_idxs = []
        for i, name in enumerate(msg.name):
            if self._containsPrefix(self.dodgeball_prefix, name):
                model_idxs.append(i)


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
