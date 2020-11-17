#!/usr/bin/env python3

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped

class TFPublisher():
    """ Publishes the transform between the simulated dodgeballs and the map
    frame. Currently uses ground-truth data from gazebo, should eventually
    be edited to use camera or something else """

    def __init__(self):
        rospy.init_node("tf_publisher")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.model_state_sub = rospy.Subscriber("/gazebo/model_states",
            ModelStates, self.modelStateCB)
        # Name of the dodgeballs - used to extract model states from gazebo
        self.dodgeball_prefix = rospy.get_param('dodgeball_prefix', 'unit_sphere')

        # Loop update rate
        rate = rospy.get_param('~rate', 10) # in hz
        self.update_rate = rospy.Rate(rate)

    def _containsPrefix(self, prefix, name):
        """ Checks an input name and determines whether it contains a prefix """
        if name[:len(prefix)] == prefix:
            return True
        else:
            return False

    def modelStateCB(self, msg):
        """ Save incoming model state msg. Does not handle any computation
        to ensure the most recent messages are used """
        self.model_state_msg = msg

    def updateTFs(self):
        """ Update transform of balls """
        # Extract dodgeball info from gazebo state
        # TODO: Replace this with a CV-based approach
        self.model_idxs = []

        for i, name in enumerate(msg.name):
            if self._containsPrefix(self.dodgeball_prefix, name):
                model_idxs.append(i)

        # Publish tf for each dodgeball
        for idx in self.model_idxs:
            t = TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "/odom" # this is temporary
            t.child_frame_id = self.model_state_msg.name[idx]
            t.transform.translation.x = msg.x
            t.transform.translation.y = msg.y
            t.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)



    def run(self):
        while not rospy.is_shutdown():
            self.updateTFs()
            self.update_rate.sleep()

if __name__ == "__main__":
    data_recorder = DataRecorder()
    data_recorder.run()
