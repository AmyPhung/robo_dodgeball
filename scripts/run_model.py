#!/usr/bin/env python3

# ROS Imports
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist,  Vector3
from std_msgs.msg import Bool

# Python Imports
import numpy as np
import os
import itertools
from operator import itemgetter
import tensorflow as tf
from tensorflow import keras
#import tensorflow_hub as hub


class DataRecorder():
    def __init__(self):
        rospy.init_node("data_recorder")

        # Publishers and Subscribers ------------------------------------------
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states",
            ModelStates, self.modelStateCB)
        self.run_sub = rospy.Subscriber("spawn_cmd", Bool, self.process_run_sub)  # Spawnin will start or stop when this is recieved
        # self.twist_sub = rospy.Subscriber("/cmd_vel",
        #     Twist, self.twistCB)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.save_sub = rospy.Subscriber("save_cmd",
            Bool, self.saveCB) # Data collection will end when True is published

        # ROS messages --------------------------------------------------------
        self.model_state_msg = ModelStates()
        self.twist_msg = Twist()

        # ROS Parameters ------------------------------------------------------
        # Name of the dodgeballs - used to extract model states from gazebo
        self.dodgeball_prefix = rospy.get_param('dodgeball_prefix', 'ball')
        # Name of the robot model - used to extract model state from gazebo
        self.robot_name = rospy.get_param('robot_name', 'mobile_base')
        # Number of closest dodgeballs to keep track of
        self.num_dodgeballs = rospy.get_param('num_dodgeballs', 5)
        # Save file location
        if rospy.has_param('~save_filename'):
            self.save_filename = rospy.get_param('~save_filename')
        else:
            self.save_filename = None
            rospy.logwarn("save_filename parameter has not been set! Data will not be saved")

        # Loop update rate
        rate = rospy.get_param('~rate', 10) # in hz
        self.update_rate = rospy.Rate(rate)

        # Internal vars -------------------------------------------------------
        self.output_data = []
        self.done = False
        self.spawn = False
        self.model_inputs = None
        self.model_output = None
        self.model = keras.models.load_model(os.path.expanduser("~/catkin_ws/src/ml_comprobofinal/ml_models/LTSM_straight"))
        self.model.layers[0]
        self.model.summary()
        # reload_sm_keras = tf.keras.models.load_model(
        #     "~/catkin_ws/src/ml_comprobofinal/ml_models/LTSM_straight",
        #     custom_objects={'KerasLayer': hub.KerasLayer})
        #
        # reload_sm_keras.summary()

    def process_run_sub(self, msg):
        # Change the running flag!
        self.spawn = msg.data

    def modelStateCB(self, msg):
        """ Save incoming model state msg. Does not handle any computation
        to ensure the most recent messages are used """
        self.model_state_msg = msg

    def twistCB(self, msg):
        """ Save incoming twist msg. Does not handle any computation
        to ensure the most recent messages are used """
        self.twist_msg = msg

    def saveCB(self, msg):
        self.done = True

    def _computeModelDistance(self, model1_idx, model2_idx):
        """ Computes distance between two models in the current model
        state message """
        m1_pos = self.model_state_msg.pose[model1_idx].position
        m2_pos = self.model_state_msg.pose[model2_idx].position

        m1_vec = np.array([m1_pos.x, m1_pos.y, m1_pos.z])
        m2_vec = np.array([m2_pos.x, m2_pos.y, m2_pos.z])

        squared_dist = np.sum((m1_vec-m2_vec)**2, axis=0)
        dist = np.sqrt(squared_dist)
        return dist

    def _computeModelVecs(self, robot_idx, ball_idx):
        """Position of ball and velocity of ball"""
        r_pos = self.model_state_msg.pose[robot_idx].position
        b_pos = self.model_state_msg.pose[ball_idx].position
        b_vel = self.model_state_msg.twist[ball_idx].linear

        r_pos_vec = np.array([r_pos.x, r_pos.y])
        b_pos_vec = np.array([b_pos.x, b_pos.y])
        vec_to_robot = r_pos_vec - b_pos_vec  # This is what we care about!
        b_vel_vec = np.array([b_vel.x, b_vel.y])

        # Compute the angle between the ball trajectory and direction to robot
        #return np.flatten(vec_to_robot, b_vel_vec)

        vec_r_x =  r_pos.x - b_pos.x
        vec_r_y = r_pos.y - b_pos.y
        return np.array([vec_r_x, vec_r_y, b_vel.x, b_vel.y])

    def _computeRobotVecs(self, robot_idx):
        r_pos = self.model_state_msg.pose[robot_idx].position
        r_pos_vec = [r_pos.x, r_pos.y]
        return r_pos_vec

    def _computeModelAngle(self, robot_idx, ball_idx):
        """ Computes angle between current ball trajectory (assuming linear)
        and position of the robot. 0 means the ball is headed directly
        towards the robot, -180 or 180 means the ball is headed
        directly away from the robot. Currently only handles 2d case """
        # Assumes these are all in world coords
        r_pos = self.model_state_msg.pose[robot_idx].position
        b_pos = self.model_state_msg.pose[ball_idx].position
        b_vel = self.model_state_msg.twist[ball_idx].linear

        r_pos_vec = np.array([r_pos.x, r_pos.y])
        b_pos_vec = np.array([b_pos.x, b_pos.y])
        b_vel_vec = np.array([b_vel.x, b_vel.y])

        # Compute the angle between the ball trajectory and direction to robot
        vec_to_robot = r_pos_vec - b_pos_vec
        vec_to_robot, b_vel_vec

        # If ball isn't moving, return pi
        if (b_vel_vec[0]==0 and b_vel_vec[1]==0):
            return np.pi

        unit_vec_to_robot = vec_to_robot / np.linalg.norm(vec_to_robot)
        unit_b_vel_vec = b_vel_vec / np.linalg.norm(b_vel_vec)
        dot_product = np.dot(unit_vec_to_robot, unit_b_vel_vec)
        angle = np.arccos(dot_product)

        # Append sign to calculation
        cross_product = np.cross(vec_to_robot, b_vel_vec)
        if (cross_product < 0):
            angle = -angle

        return angle

    def _computeModelVelocity(self, ball_idx):
        """ Computes magnitude of the velocity """
        b_vel = self.model_state_msg.twist[ball_idx].linear
        b_vel_vec = np.array([b_vel.x, b_vel.y, b_vel.z])
        vel_magnitude = np.linalg.norm(b_vel_vec)

        return vel_magnitude

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
            - magnitude of velocity
        """
        vel_cmd = self.twist_msg.linear.x

        # Get model index
        m_idx = 0
        try:
            m_idx = self.model_state_msg.name.index(self.robot_name)
        except ValueError:
            return

        # Temporarily store all ball distances, angles, and velocities - will filter later
        dists, angles, vels = [], [], []

        balls = []
        # Parse gazebo model message
        for b_idx, m_name in enumerate(self.model_state_msg.name):
            # Check to make sure we're looking at a ball
            if self._containsPrefix(self.dodgeball_prefix, m_name):
                dist = self._computeModelDistance(m_idx, b_idx)
                dists.append(dist)
                # angle = self._computeModelAngle(m_idx, b_idx)
                # angles.append(angle)
                vel = self._computeModelVelocity(b_idx)
                vels.append(vel)

                # nathan's tests
                ball_vecs = self._computeModelVecs(m_idx, b_idx)
                balls.append(ball_vecs)
        robot_pos = self._computeRobotVecs(m_idx)
        # If we don't have enough dodgeballs, add in a few "dummy" values
        # These dummy values are equivalent to a dodgeball that would be
        # sufficiently far away headed in the wrong direction
        # Don't need to worry about negative indices because it'll just create
        # an empty list
        missing_balls = self.num_dodgeballs - len(dists)
        dists += missing_balls*[1000] # Add dummy balls 1000 meters away
        # angles += missing_balls*[np.pi] # Add dummy balls moving away from robot
        # vels += missing_balls*[0] # Add dummy balls with 0 velocity

        # Add dummy balls in
        dummy_ball = [[0,100,0,0]]
        balls += missing_balls*dummy_ball

        # Get indices of n closest balls
        nearest_idxs = np.argpartition(dists, -self.num_dodgeballs)[:self.num_dodgeballs]
        # Get distances and angles of n closest balls
        # n_dists = list(itemgetter(*nearest_idxs)(dists))
        # n_angles = list(itemgetter(*nearest_idxs)(angles))
        # n_vels = list(itemgetter(*nearest_idxs)(vels))

        n_balls = list(itemgetter(*nearest_idxs)(balls))
        n_balls = list(itertools.chain(*n_balls))
        # Record data point
        new_pt = [vel_cmd] + n_balls + robot_pos # n_dists + n_angles + n_vels + n_robot_pos + n_ball_pos + n_ball_vel
        self.output_data.append(new_pt)

    def writeDataToFile(self):
        if self.save_filename != None:
            np_data = np.array(self.output_data)
            np_data_t = np_data #.transpose()
            np.savetxt(self.save_filename, np_data_t)
            np.save(self.save_filename, np_data_t)
            rospy.loginfo("Dataset saved!")
        else:
            rospy.logwarn("Dataset not saved - no filename specified")

    def _containsPrefix(self, prefix, name):
        """ Checks an input name and determines whether it contains a prefix """
        if name[:len(prefix)] == prefix:
            return True
        else:
            return False
    def prepare_data(self, model_depth=5):
        if len(self.output_data) < model_depth:
            self.model_inputs = None
        else:
            # There is enough data
            self.model_inputs = self.output_data[-model_depth:]
            # self.model_inputs = [past_step[1:] for past_step in self.model_inputs]
            self.model_inputs = np.array(self.model_inputs)[:, 1:]
    def run(self):
        while not rospy.is_shutdown():
            # if self.done:
            #     self.writeDataToFile()
            #     return
            if self.spawn:
                self.recordDataPoint()

                # Prepare model inputs
                self.prepare_data()
                if self.model_inputs is None:
                    continue
                # Run the inputs through the model
                self.model_inputs = self.model_inputs.reshape((1, self.model_inputs.shape[0], self.model_inputs.shape[1]))
                self.model_output = self.model.predict(np.array(self.model_inputs))
                self.pub.publish(Twist(linear=Vector3(x=self.model_output)))
            self.update_rate.sleep()

if __name__ == "__main__":
    data_recorder = DataRecorder()
    data_recorder.run()
