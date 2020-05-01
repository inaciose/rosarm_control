#!/usr/bin/env python
import rospy
#import sys, select, termios, tty
#from std_msgs.msg import String
import math
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
        self.desired_position = {}
        #self.end_position = {}
        self.position_limits = {}

        self.allow_joystick_input = False

        # ROS related init stuff
        self.pub_motors = rospy.Publisher('motors', UInt16MultiArray, queue_size=10)
        rospy.Subscriber("joy", Joy, self.joy_cb)
        self.rate = rospy.get_param('rate', 25)

        self.L1 = rospy.get_param('L1', 94) # servo axis vertical offset eg: 50+35
        self.L2 = rospy.get_param('L2', 135) # be = 150
        self.L3 = rospy.get_param('L3', 147) # ef = 150

        # set position limits from parameters
        self.position_limits["x"] = rospy.get_param('xposition_limits', [0, 200])
        self.position_limits["y"] = rospy.get_param('yposition_limits', [-100, 100])
        self.position_limits["z"] = rospy.get_param('zposition_limits', [0, 200])

        # set joint limits from parameters
        self.joints_limits["x"] = rospy.get_param('xjoint_limits', [55, 145])
        self.joints_limits["y"] = rospy.get_param('yjoint_limits', [20, 160])
        self.joints_limits["z"] = rospy.get_param('zjoint_limits', [10, 105])
        self.joints_limits["g"] = rospy.get_param('gjoint_limits', [60, 140])

        # set desired position start values
        self.desired_position["x"] = 100
        self.desired_position["y"] = 0
        self.desired_position["z"] = 100

        # set joint start values
        self.ik()
        self.robot_joints["g"] = 90

    # Hipotenusa:
    def hipo(self, x,y):
        return math.sqrt(x*x + y*y)

    # Cosines law:
    def lawOfCosines(self, a,b,c):
        rate = (a*a + b*b - c*c) / (2 * a * b)
        if abs(rate) > 1:
            if max(rate,0) == 0:
                rate = -1
            if max(rate,0) == rate:
                        rate = 1
        return math.acos(rate)

    # Radians conversion
    def deg(self, rad):
        return rad * 180 / math.pi

    def ik(self):
        # Desired position in cartesian coordinates taken from global self.desired_position
        cartP = {'xEE': float(self.desired_position["x"]), 'yEE': float(self.desired_position["y"]), 'zEE': float(self.desired_position["z"])}
        
        # Desired position converted to polar/cylindrical coordinates
        cylP = {'theta': math.atan(cartP['yEE'] / cartP['xEE']), 'r':self.hipo(cartP['xEE'], cartP['yEE']), 'zhat':cartP['zEE'] - self.L1}  
        # horizontal angle, horizontal radius, vertical offset

        # vertical radius based on horizontal radius and vertical offset
        rho = self.hipo(cylP['r'], cylP['zhat'])

        # calculate the base servo #1 angle y
        M1 = 2*cylP['theta'] + math.pi / 2

        # calculate the mainarm servo #2 angle x
        M2 = math.atan(cylP['zhat'] / cylP['r']) + self.lawOfCosines(self.L2, rho, self.L3)
        
        # calculate the varm servo #3 angle z
        M3 = M2 + self.lawOfCosines(self.L2, self.L3, rho) - math.pi/2
        
        #Physical adjustments
        angles = [M1, math.pi - M2, M3]

        #Radians to Degrees conversion
        angles = [self.deg(angle) for angle in angles]

        # store calculated joint angles to global self.robot_joints
        self.robot_joints["y"] = angles[0]
        self.robot_joints["x"] = angles[1]
        self.robot_joints["z"] = angles[2]

    def joy_cb(self, msg):

        if not self.allow_joystick_input:
            return

        self.allow_joystick_input = False
        
        if msg.axes[0] > 0:
            if self.desired_position["y"] < self.position_limits["y"][1]:
                self.desired_position["y"] += 1

        if msg.axes[0] < 0:
            if self.desired_position["y"] > self.position_limits["y"][0]:
                self.desired_position["y"] -= 1

        if msg.axes[1] > 0:
            if self.desired_position["x"] < self.position_limits["x"][1]:  
                self.desired_position["x"] += 1

        if msg.axes[1] < 0:
            if self.desired_position["x"] > self.position_limits["x"][0]:
                self.desired_position["x"] -= 1

        if msg.buttons[0] > 0:
           if self.desired_position["z"] < self.position_limits["z"][1]:  
                self.desired_position["z"] += 1

        if msg.buttons[2] > 0:
           if self.desired_position["z"] > self.position_limits["z"][0]:
                 self.desired_position["z"] -= 1

        if msg.buttons[1] > 0:
           if self.robot_joints["g"] < self.joints_limits["g"][1]:  
                self.robot_joints["g"] += 1

        if msg.buttons[3] > 0:
           if self.robot_joints["g"] > self.joints_limits["g"][0]:
                 self.robot_joints["g"] -= 1

        # calculate inverse kinematics to self.robot_joints 
        self.ik()

        # apply defined joint limits
        if self.robot_joints["y"] < self.joints_limits["y"][0]:
            self.robot_joints["y"] = self.joints_limits["y"][0]

        if self.robot_joints["y"] > self.joints_limits["y"][1]:
            self.robot_joints["y"] = self.joints_limits["y"][1]

        if self.robot_joints["x"] < self.joints_limits["x"][0]:  
            self.robot_joints["x"] = self.joints_limits["x"][0]

        if self.robot_joints["x"] > self.joints_limits["x"][1]:
            self.robot_joints["x"] = self.joints_limits["x"][1]

        if self.robot_joints["z"] < self.joints_limits["z"][0]:  
            self.robot_joints["z"] = self.joints_limits["z"][0]

        if self.robot_joints["z"] > self.joints_limits["z"][1]:
            self.robot_joints["z"] = self.joints_limits["z"][1] 

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