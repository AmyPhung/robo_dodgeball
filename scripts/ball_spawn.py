#!/usr/bin/env python3
from __future__ import print_function, division
import rospy
from neato_node.msg import Bump
from std_msgs.msg import Int8MultiArray, Bool
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
import random
import tf
import math
import os

settings = termios.tcgetattr(sys.stdin)

class ball_spawn(object):
    def __init__(self):
        rospy.init_node('ball_spawn')
        rospy.loginfo("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/set_model_state")
        rospy.wait_for_service("gazebo/get_model_state")
        rospy.wait_for_service("gazebo/get_world_properties")
        rospy.loginfo("Services done")

        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.set_model = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        self.get_model = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        self.get_world_properties = rospy.ServiceProxy("gazebo/get_world_properties", GetWorldProperties)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.run_sub = rospy.Subscriber("spawn_cmd", Bool, self.process_run_sub)  # Spawnin will start or stop when this is recieved
        self.save_sub = rospy.Subscriber("save_cmd",
            Bool, self.saveCB) # Data collection will end when True is published lets shut down this simulation

        # ROS Parameters ------------------------------------------------------
        # Name of the dodgeballs - used to extract model states from gazebo
        self.dodgeball_prefix = rospy.get_param('~dodgeball_prefix', 'ball')
        # Name of the robot model - used to extract model state from gazebo
        self.robot_name = rospy.get_param('~robot_name', 'mobile_base')
        # Number of closest dodgeballs to keep track of
        self.num_dodgeballs = rospy.get_param('~num_dodgeballs', 5)
        # Grab the spawn Location!!
        self.spawn_x_min = rospy.get_param('~spawn_x_min', -3)
        self.spawn_x_max = rospy.get_param('~spawn_x_max', 3)
        self.spawn_y_min = rospy.get_param('~spawn_y_min', 5)
        self.spawn_y_max = rospy.get_param('~spawn_y_max', 5)
        # Grab the spawn velocities!!
        self.a_vel_min = rospy.get_param('~a_vel_min', 0)
        self.a_vel_max = rospy.get_param('~a_vel_max', 0)
        self.l_vel_min = rospy.get_param('~l_vel_min', 3)
        self.l_vel_max = rospy.get_param('~l_vel_max', 5)
        # Grab the targeting method
        self.targeting = rospy.get_param('~targeting', "random")

        # Other dodgeball tracking stuff
        self.ball_num = 0
        self.ball_names = []  # List to keep track of which balls are in frame
        if rospy.has_param('~ball_model_file'):
            self.dodgeball = open(os.path.expanduser(rospy.get_param('~ball_model_file')), 'r').read()
        else:
            self.dodgeball = open(os.path.expanduser('~/catkin_ws/src/ml_comprobofinal/model/dodgeball/model.sdf'), 'r').read()
        self.running = False
        self.done = False

    def process_run_sub(self, msg):
        # Change the running flag!
        self.running = msg.data

    def saveCB(self, msg):
        self.done = True

    def gen_pose(self,x, y, z=0, theta=0):
        # Returns particle as Pose object
        orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, theta) # generates orientation from euler
        return Pose(position=Point(x=x, y=y, z=z),
                    orientation=Quaternion(x=orientation_tuple[0],
                                           y=orientation_tuple[1],
                                           z=orientation_tuple[2],
                                           w=orientation_tuple[3]))

    def gen_ball_loc(self):
        """Generate the starting location of the ball based on mode"""
        x = random.uniform(self.spawn_x_min, self.spawn_x_max)
        y = random.uniform(self.spawn_y_min, self.spawn_y_max)
        z = random.uniform(0, 0)
        return x, y, z

    def gen_ball_vel(self, loc):
        """Generates the ball's velocity depending on the mode
        returns Velocity to be set in Twist message """
        a_vel = random.uniform(self.a_vel_min, self.a_vel_max)
        l_vel = random.uniform(self.l_vel_min, self.l_vel_max)

        # Adjust the direction of the l_vel depending on targetting
        if self.targeting == "straight":
            x_tar = loc[0]
            y_tar = 0
        elif self.targeting == "neato":
            neato_pose = self.get_model("mobile_base", "world").pose
            x_tar = neato_pose.position.x
            y_tar = 0
        elif self.targeting == "center":
            x_tar = 0
            y_tar = 0
        else: # method == "random":
            x_tar = random.uniform(self.spawn_x_min, self.spawn_x_max)
            y_tar = 0
        # Generate the velocities
        y_to_target = y_tar - loc[1]
        x_to_target = x_tar - loc[0]
        magnitude = math.sqrt(y_to_target ** 2 + x_to_target ** 2)
        l_vel_x = l_vel * x_to_target / magnitude
        l_vel_y = l_vel * y_to_target / magnitude
        return l_vel_x, l_vel_y, a_vel

    def gen_ball(self):
        x, y, z = self.gen_ball_loc()
        l_vel_x, l_vel_y, a_vel = self.gen_ball_vel((x, y))
        # print("x:{} y:{} x:{}".format(x, y, z))
        # print("a_vel: {} l_vel_x: {} l_vel_y: {}".format(a_vel, l_vel_x, l_vel_y))
        ball_pose = self.gen_pose(x, y)
        ball_name = self.dodgeball_prefix + str(self.ball_num)
        self.ball_names.append(ball_name) # Add the ball to keep track of it
        ball_twist = Twist(linear=Vector3(x=l_vel_x, y=l_vel_y), angular=Vector3(z=0))
        model_state = ModelState(
            model_name=ball_name,
            pose=ball_pose,
            twist=ball_twist)
        self.spawn_model(
            model_name=ball_name,
            model_xml=self.dodgeball,
            robot_namespace='',
            initial_pose=ball_pose
        )
        self.set_model(model_state)
        self.ball_num += 1

    def cleanup_balls(self):
        """
        Removes balls if they have past the robot.
        returns: True if a new ball should be spawned, False if not
        """
        for num, ball in enumerate(self.ball_names):
            #Get the model state (check if it has gone past the robot)
            world_pose = self.get_model(ball, "world").pose #
            neato_pose = self.get_model(ball, "mobile_base").pose
            #print("World: {} \nNeato: {}".format(world_pose, neato_pose))

            # Delete ones that are gone
            if neato_pose.position.y <= 0:
                self.delete_model(ball)
                del self.ball_names[num]

    def manage_balls(self):
        # Check if we have less than the max num balls
        #print("ball list: {} Mode_max: {}".format(self.ball_names, self.num_dodgeballs))
        while len(self.ball_names) < self.num_dodgeballs:
            #print("generating more balls!!!")
            self.gen_ball()

    def remove_all_balls(self):
        world_data = self.get_world_properties()
        for name in world_data.model_names:
            if name[:len(self.dodgeball_prefix)] == self.dodgeball_prefix:
                 self.delete_model(name)
                 rospy.loginfo("Removed model " + name)
        self.ball_names = []

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def run(self):
        r = rospy.Rate(10)
        rospy.loginfo('Cleaning the playpen')
        self.remove_all_balls()
        rospy.loginfo("Time for some dodgeball!")

        while not rospy.is_shutdown() and not self.done:
            r.sleep()
            self.cleanup_balls()
            if self.running:
                self.manage_balls()

        self.remove_all_balls()
        self.pub.publish(Twist(linear=Vector3(x=0), angular=Vector3(z=0)))

if __name__ == '__main__':
    ball_spawner = ball_spawn()

    ball_spawner.run()
