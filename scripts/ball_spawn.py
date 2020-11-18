#!/usr/bin/env python3
from __future__ import print_function, division
import rospy
from neato_node.msg import Bump
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState, GetModelState
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
        print("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/set_model_state")
        rospy.wait_for_service("gazebo/get_model_state")
        print("Services done")

        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.set_model = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
        self.get_model = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
        # rospy.Subscriber('/bump', Int8MultiArray, self.process_bump)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.naming = "ball"
        self.ball_num = 0
        self.ball_names = [] # List to keep track of which balls are in frame
        straight = {"max_balls":1, "spawn_rg": (-2,2, 5,5), "target": "straight", "l_vel_rg": (5,2), "a_vel_rg": (-5,-5), "color": None, "size_rg": (1,1)}

        # Set the mode of ball spawnign
        self.mode = straight

        self.dodgeball = open(os.path.expanduser('~/catkin_ws/src/ml_comprobofinal/model/dodgeball/model.sdf'), 'r').read()

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
        spawn_rg = self.mode["spawn_rg"]
        x = random.uniform(spawn_rg[0], spawn_rg[1])
        y = random.uniform(spawn_rg[2], spawn_rg[3])
        z = random.uniform(0, 0)
        return x, y, z

    def gen_ball_vel(self, loc):
        """Generates the ball's velocity depending on the mode
        returns Velocity to be set in Twist message """
        method = self.mode["target"]
        l_vel_rg = self.mode["l_vel_rg"]
        a_vel_rg = self.mode["a_vel_rg"]
        a_vel = random.uniform(a_vel_rg[0], a_vel_rg[1])
        l_vel = random.uniform(l_vel_rg[0], l_vel_rg[1])
        spawn_rg = self.mode["spawn_rg"]

        # Adjust the direction of the l_vel depending on targetting
        if method == "straight":
            x_tar = loc[0]
            y_tar = 0
        elif method == "neato":
            neato_pose = self.get_model("mobile_base", "world").pose
            print(neato_pose)
            x_tar = neato_pose.position.x
            y_tar = 0
        elif method == "center":
            x_tar = 0
            y_tar = 0
        else: # method == "random":
            x_tar = random.uniform(spawn_rg[0], spawn_rg[1])
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
        print("x:{} y:{} x:{}".format(x, y, z))
        print("a_vel: {} l_vel_x: {} l_vel_y: {}".format(a_vel, l_vel_x, l_vel_y))
        ball_pose = self.gen_pose(x, y)
        ball_name = self.naming + str(self.ball_num)
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

            # Delwte ones that are gone
            if neato_pose.position.y <= 0:
                self.delete_model(ball)
                del self.ball_names[num]

    def manage_balls(self):
        # CLeanup the balls
        self.cleanup_balls()

        # Check if we have less than the max num balls
        #print("ball list: {} Mode_max: {}".format(self.ball_names, self.mode["max_balls"]))
        while len(self.ball_names) < self.mode["max_balls"]:
            #print("generating more balls!!!")
            self.gen_ball()

    def remove_all_balls(self, num=20):
        for ball_num in range(num):
            self.delete_model(self.naming + str(ball_num))
        self.ball_names = []


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
        r = rospy.Rate(100)
        print('Cleaning the playpen')
        self.remove_all_balls(200)
        print("Time for some dodgeball!")

        while not rospy.is_shutdown():
            r.sleep()
            self.manage_balls()
            # key = self.getKey()
            #
            # if key ==' ':
            #     print("Generate ball!!!!")
            #
            #
            # elif key == 'q':
            #     self.remove_all_balls(self.ball_num+1)
            #     break

        self.pub.publish(Twist(linear=Vector3(x=0), angular=Vector3(z=0)))

if __name__ == '__main__':
    ball_spawner = ball_spawn()

    ball_spawner.run()

