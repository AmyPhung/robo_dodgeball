#!/usr/bin/env python3
"""
Process Gazebo
Handles all gazebo communications. Has 2 primary modes: a data-collecting mode,
and a model testing mode. In data collection mode, this node records position
and velocity data for the top n balls (in robot coordinate frame), and the
x component of the human velocity command. In model testing mode, it doesn't
listen to human velocity commands, and instead sends its own commands based
on the trained machine-learning model

ROS Parameters:
- dodgeball_prefix: name of dodgeballs in gazebo
- robot_name: name of robot in gazebo
- num_dodgeballs: number of dodgeballs to record in dataset
- run_model: 0 for data collection mode, 1 for model testing mode
- save_filename: Location to save recorded data to
- rate: Rate in Hz for node to run at
- model_path: Machine learning model path
"""
# ROS Imports
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from tf.transformations import quaternion_matrix

# Python Imports
import numpy as np
from numpy.linalg import inv # Matrix inverse function
import itertools
import os
from operator import itemgetter

class ProcessGazebo():
    def __init__(self):
        rospy.init_node("process_gazebo")

        # Publishers and Subscribers ------------------------------------------
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states",
            ModelStates, self.modelStateCB)
        self.spawn_sub = rospy.Subscriber("spawn_cmd",
            Bool, self.spawnCB)  # Spawning will start or stop when this is recieved
        self.save_sub = rospy.Subscriber("save_cmd",
            Bool, self.saveCB) # Data collection will end when True is published

        # ROS messages --------------------------------------------------------
        self.model_state_msg = ModelStates()
        self.twist_msg = Twist()

        # ROS Parameters ------------------------------------------------------
        # Name of the dodgeballs - used to extract model states from gazebo
        self.dodgeball_prefix = rospy.get_param('~dodgeball_prefix', 'ball')
        # Name of the robot model - used to extract model state from gazebo
        self.robot_name = rospy.get_param('~robot_name', 'mobile_base')
        # Number of closest dodgeballs to keep track of
        self.num_dodgeballs = rospy.get_param('~num_dodgeballs', 5)
        # Get whether this is a recording or inference
        self.run_model = rospy.get_param('~run_model', 0)
        # Get what size model we should be using (the origin inputs?)
        self.use_origin = str(rospy.get_param('~use_origin', "True")).strip() == "True"
        print("Using origin: ", self.use_origin)
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

        if self.run_model: # Use for machine-learning-based controller
            self.model_path = rospy.get_param('~model_path', "LSTM_05_002")
            self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.model_depth = int(self.model_path.split("LSTM_", 1)[1].split("-", 1)[0])
            # print("Model depth: {}".format(self.model_depth))
            if self.model_path is not None:
                from tensorflow import keras
                self.model_inputs = None
                self.model_output = None
                self.model = keras.models.load_model(self.model_path)
                self.model.summary()
        else: # Use human controller
            self.twist_sub = rospy.Subscriber("/cmd_vel", Twist, self.twistCB)


    def spawnCB(self, msg):
        """ Update the spawn running flag. Does not handle any computation
        to ensure the most recent messages are used """
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
        """ Extract the position of ball (relative to robot)
        and velocity of ball (relative to robot) and return
        as a numpy array """
        r_pos_world = self.model_state_msg.pose[robot_idx].position
        r_quat_world = self.model_state_msg.pose[robot_idx].orientation
        b_pos_world = self.model_state_msg.pose[ball_idx].position
        b_vel_world = self.model_state_msg.twist[ball_idx].linear

        # Input order: x,y,z,w
        rot_matrix = quaternion_matrix([r_quat_world.x,
                                        r_quat_world.y,
                                        r_quat_world.z,
                                        r_quat_world.w])

        translation_matrix = [[1, 0, 0, r_pos_world.x],
                              [0, 1, 0, r_pos_world.y],
                              [0, 0, 1, r_pos_world.z],
                              [0, 0, 0, 1            ]]

        # Rewrite ball position and velocity as a vector
        b_pos_vector = [b_pos_world.x, b_pos_world.y, b_pos_world.z, 1]
        b_vel_vector = [b_vel_world.x, b_vel_world.y, b_vel_world.z, 1]

        # Matrix multiplication order for composite transform:
        # SRT (scale, rotate, translate)
        T = np.matmul(inv(rot_matrix), inv(translation_matrix))

        # Compute ball position and velocity in robot coords
        b_pos_robot = np.matmul(T, b_pos_vector)
        b_vel_robot = np.matmul(T, b_vel_vector)

        return np.array([b_pos_robot[0], b_pos_robot[1],
                         b_vel_robot[0], b_vel_robot[1]])

    def _computeRobotVecs(self, robot_idx):
        """ Extracts the global position of the robot """
        r_pos = self.model_state_msg.pose[robot_idx].position
        r_pos_vec = [r_pos.x, r_pos.y]
        return r_pos_vec

    def recordDataPoint(self):
        """ Get n closest dodgeballs, save to dataset.

        Recorded Data:
            - human velocity command (magnitude only - assumes 1D case.
            Negative values for backwards, positive for forwards)
            - data for the nearest n balls, including:
                > x,y position of balls relative to robot
                > vx,vy velocity of balls relative to global coords
        """
        vel_cmd = self.twist_msg.linear.x

        # Get model index
        m_idx = 0
        try:
            m_idx = self.model_state_msg.name.index(self.robot_name)
        except ValueError:
            return

        # Temporarily store all ball data - will filter these later using
        # the distances
        balls, dists = [], []

        # Parse gazebo model message
        for b_idx, m_name in enumerate(self.model_state_msg.name):
            # Check to make sure we're looking at a ball
            if self._containsPrefix(self.dodgeball_prefix, m_name):
                dist = self._computeModelDistance(m_idx, b_idx)
                dists.append(dist)

                # Record ball data in vector form
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

        # Add dummy balls in
        dummy_ball = [[0,100,0,0]]
        balls += missing_balls*dummy_ball

        # Get indices of n closest balls
        nearest_idxs = np.argpartition(dists, -self.num_dodgeballs)[:self.num_dodgeballs]

        # Get info of n closest balls
        n_balls = list(itemgetter(*nearest_idxs)(balls))
        n_balls = list(itertools.chain(*n_balls))

        # Record data point

        if self.use_origin:
            new_pt = [vel_cmd] + n_balls + robot_pos
        else:
            new_pt = [vel_cmd] + n_balls
        self.output_data.append(new_pt)

    def prepareData(self, model_depth=5):
        if len(self.output_data) < model_depth:
            self.model_inputs = None
        else:
            # There is enough data
            self.model_inputs = self.output_data[-model_depth:]
            # self.model_inputs = [past_step[1:] for past_step in self.model_inputs]
            self.model_inputs = np.array(self.model_inputs)[:, 1:]

    def writeDataToFile(self):
        if self.save_filename != None:
            np_data = np.array(self.output_data)
            print(np_data.shape)
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

    def run(self):
        while not rospy.is_shutdown():
            if self.done and not self.run_model:
                self.writeDataToFile()
                return
            if self.spawn:
                self.recordDataPoint()

                # Send the model inputs if testing the model
                if self.run_model:
                    # Prepare model inputs

                    self.prepareData(self.model_depth)
                    if self.model_inputs is None:
                        continue
                    # Run the inputs through the model
                    print(self.model_inputs.shape)
                    self.model_inputs = self.model_inputs.reshape(
                        (1, self.model_inputs.shape[0], self.model_inputs.shape[1]))
                    self.model_output = self.model.predict(np.array(self.model_inputs))
                    print("MODEL OUTPUT: ", self.model_output)
                    self.twist_pub.publish(Twist(linear=Vector3(x=self.model_output)))
            self.update_rate.sleep()

if __name__ == "__main__":
    process_gazebo = ProcessGazebo()
    process_gazebo.run()
