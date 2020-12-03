#!/usr/bin/env python3
"""
Joystick Teleop
Uses a usb joystick for teleoperation. Publishes twist command to /cmd_vel
topic (currently only 1D). Click A to start spawner and data recording and
B to stop data recording

Designed for and tested on an Xbox 360 controller

ROS Parameters:
- x_threshold = joystick deadzone for x
- y_threshold = joystick deadzone for y
- rate = node update rate
"""
import inputs
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def rescale(in_min, in_max, out_min, out_max, in_val):
    scale = (float(in_val) - float(in_min)) / (float(in_max) - float(in_min))
    output = (float(out_max) - float(out_min))*scale + float(out_min)
    return output

class JoystickTeleop():
    def __init__(self):
        rospy.init_node("joystick_teleop")

        rate = rospy.get_param('~rate', 1000)
        self.update_rate = rospy.Rate(rate)

        self.max_vel = rospy.get_param('~max_vel', 1)
        self.x_thresh = rospy.get_param('~x_threshold', 1000)
        self.y_thresh = rospy.get_param('~y_threshold', 6000)

        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.save_publisher = rospy.Publisher('save_cmd', Bool, queue_size = 1)
        self.run_spawn_publisher = rospy.Publisher('spawn_cmd', Bool, queue_size = 1)

        self.pads = inputs.devices.gamepads
        if len(self.pads) == 0:
            raise Exception("Couldn't find any Gamepads!")
        else:
            rospy.loginfo("Found %s", self.pads)
            rospy.loginfo("Starting teleop! \n" + \
                          "Keybindings: \n" + \
                          "- A: Start Ball Spawner \n" + \
                          "- B: Stop recording & save data set")

        self.curr_x_input = 0
        self.curr_y_input = 0
        self.curr_btn_south = 0 # a button
        self.curr_btn_east = 0 # b button

        # Used to make sure we only detect changes in state
        self.prev_curr_btn_south = 0 # a button
        self.prev_curr_btn_east = 0 # b button

    def computeTwistCmd(self, x, y):
        """ Compute twist command based on current x and y joystick values
        y input range: (32768, -32768) (backwards, forwards)
        x input range: (-32768, 32768) (left, right)
        linear velocity output range: (-1, 1) (backwards, forwards)
        angular velocity output range: (1, -1) (left, right)
        """
        lin_vel = rescale(32768, -32768, -self.max_vel, self.max_vel, y)
        ang_vel = rescale(-32768, 32768, 1, -1, x)

        output = Twist()
        output.linear.x = lin_vel
        output.angular.z = ang_vel

        return output

    def run(self):
        while not rospy.is_shutdown():
            events = inputs.get_gamepad()

            for event in events:
                # print(event.ev_type, event.code, event.state)
                if event.code == 'ABS_Y':
                    self.curr_y_input = event.state
                elif event.code == 'ABS_X':
                    self.curr_x_input = event.state
                elif event.code == 'BTN_SOUTH':
                    self.curr_btn_south = event.state
                elif event.code == 'BTN_EAST':
                    self.curr_btn_east = event.state

            # Only detect "new" presses
            if self.curr_btn_south == 1 and self.prev_curr_btn_south == 0: # a button
                spawn_cmd = Bool(data = True)
                self.run_spawn_publisher.publish(spawn_cmd)
            if self.curr_btn_east == 1 and self.prev_curr_btn_east == 0: # b button
                save_cmd = Bool(data = True)
                self.save_publisher.publish(save_cmd)

            # 2D Drive
            # new_cmd = self.computeTwistCmd(self.curr_x_input, self.curr_y_input)
            # 1D Drive
            new_cmd = self.computeTwistCmd(0, self.curr_y_input)
            self.twist_pub.publish(new_cmd)

            # Update button states
            self.prev_curr_btn_south = self.curr_btn_south
            self.prev_curr_btn_east = self.curr_btn_east
            self.update_rate.sleep()


if __name__ == "__main__":
    jt = JoystickTeleop()
    jt.run()
