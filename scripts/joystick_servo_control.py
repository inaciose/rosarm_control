#!/usr/bin/env python
import rospy
#import sys, select, termios, tty
#from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Joy
#import yaml
#import rospkg
#import glob, os

class JoystickServoControl:
    def __init__(self):
        # global variables
        self.robot_joints = {}
        self.joints_limits = {}

        self.allow_joystick_input = False

        # ROS related init stuff
        self.pub_motors = rospy.Publisher('motors', UInt16MultiArray, queue_size=10)
        rospy.Subscriber("joy", Joy, self.joy_cb)
        self.rate = rospy.get_param('rate', 25)

        # set joint limits from parameters
        self.joints_limits["x"] = rospy.get_param('xjoint_limits', [55, 145])
        self.joints_limits["y"] = rospy.get_param('yjoint_limits', [20, 160])
        self.joints_limits["z"] = rospy.get_param('zjoint_limits', [10, 105])
        self.joints_limits["g"] = rospy.get_param('gjoint_limits', [60, 140])

        # set joint start values
        self.robot_joints["x"] = 90
        self.robot_joints["y"] = 90
        self.robot_joints["z"] = 90
        self.robot_joints["g"] = 90
        self.robot_joints["v"] = 10.0

        #self.resolution = 10.0 # poses per second
        #self.max_acceleration = 100.0

    def joy_cb(self, msg):

        if not self.allow_joystick_input:
            return

        self.allow_joystick_input = False
        
        if msg.axes[0] > 0:
            if self.robot_joints["y"] < self.joints_limits["y"][1]:
                self.robot_joints["y"] += 1

        if msg.axes[0] < 0:
            if self.robot_joints["y"] > self.joints_limits["y"][0]:
                self.robot_joints["y"] -= 1

        if msg.axes[1] > 0:
            if self.robot_joints["x"] < self.joints_limits["x"][1]:  
                self.robot_joints["x"] += 1

        if msg.axes[1] < 0:
            if self.robot_joints["x"] > self.joints_limits["x"][0]:
                self.robot_joints["x"] -= 1

        if msg.buttons[0] > 0:
           if self.robot_joints["z"] < self.joints_limits["z"][1]:  
                self.robot_joints["z"] += 1

        if msg.buttons[2] > 0:
           if self.robot_joints["z"] > self.joints_limits["z"][0]:
                 self.robot_joints["z"] -= 1

        if msg.buttons[1] > 0:
           if self.robot_joints["g"] < self.joints_limits["g"][1]:  
                self.robot_joints["g"] += 1

        if msg.buttons[3] > 0:
           if self.robot_joints["g"] > self.joints_limits["g"][0]:
                 self.robot_joints["g"] -= 1

        #print(self.robot_joints)

    def publish_robot_state(self, state):
        array_msg = UInt16MultiArray()
        array_msg.data = [state["y"], state["x"], state["z"], state["g"]]
        self.pub_motors.publish(array_msg)

    def loop_node(self):
        self.publish_robot_state(self.robot_joints)
        self.allow_joystick_input = True

if __name__ == '__main__':
    rospy.init_node('joystick_servo_control', anonymous=True)
    joyctl = JoystickServoControl()
    rate = rospy.Rate(joyctl.rate)
    while not rospy.is_shutdown():
        joyctl.loop_node()
        rate.sleep()