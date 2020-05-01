#!/usr/bin/env python
import rospy
import sys, select, termios, tty
from std_msgs.msg import String
import math
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Joy
import yaml
import rospkg
import time

class JoystickServoControl:
    def __init__(self):
        # global variables
        self.robot_joints = {}
        self.joints_limits = {}
        self.desired_position = {}
        #self.end_position = {}
        self.position_limits = {}

        self.allow_joystick_input = False

        self.current_poses = []
        self.lastbutton = 100

        rospack = rospkg.RosPack()
        self.path = rospack.get_path('rosarm_control')

        self.resolution = 10.0 # poses per second
        self.max_acceleration = 100.
        rospy.Subscriber("load", String, self.load_cb)

        self.play_sequence = False

        # ROS related init stuff
        self.pub_motors = rospy.Publisher('motors', UInt16MultiArray, queue_size=10)
        rospy.Subscriber("joy", Joy, self.joy_cb)
        self.rate = rospy.get_param('rate', 25)

        self.L1 = rospy.get_param('L1', 94) # servo axis vertical offset eg: 50+35
        self.L2 = rospy.get_param('L2', 135) # be = 150
        self.L3 = rospy.get_param('L3', 147) # ef = 150

        # set start values
        self.init_joint_limits()
        self.init_robot_joints()

    def init_joint_limits(self):
        # set position limits from parameters
        self.position_limits["x"] = rospy.get_param('xposition_limits', [50, 250])
        self.position_limits["y"] = rospy.get_param('yposition_limits', [-100, 100])
        self.position_limits["z"] = rospy.get_param('zposition_limits', [20, 250])

        # set joint limits from parameters
        self.joints_limits["x"] = rospy.get_param('xjoint_limits', [55, 145])
        self.joints_limits["y"] = rospy.get_param('yjoint_limits', [20, 160])
        self.joints_limits["z"] = rospy.get_param('zjoint_limits', [10, 105])
        self.joints_limits["g"] = rospy.get_param('gjoint_limits', [60, 140])
        self.joints_limits["v"] = rospy.get_param('vjoint_limits', [5, 50])

    def init_robot_joints(self):
        # set desired position values
        self.desired_position["x"] = 100
        self.desired_position["y"] = 0
        self.desired_position["z"] = 100

        # set joint start values
        self.ik()

        # set claw joint and general velocity
        self.robot_joints["g"] = 90
        self.robot_joints["v"] = 10.0

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
        self.robot_joints["y"] = int(angles[0])
        self.robot_joints["x"] = int(angles[1])
        self.robot_joints["z"] = int(angles[2])

    def apply_joint_limits(self):
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

    def joy_cb(self, msg):

        if self.play_sequence:
            return

        if not self.allow_joystick_input:
            return

        self.allow_joystick_input = False
        
        if msg.axes[0] > 0:
            self.lastbutton = 100
            if self.desired_position["y"] > self.position_limits["y"][0]:
                self.desired_position["y"] -= 1

        if msg.axes[0] < 0:
            self.lastbutton = 100
            if self.desired_position["y"] < self.position_limits["y"][1]:
                self.desired_position["y"] += 1

        if msg.axes[1] > 0:
            self.lastbutton = 100
            if self.desired_position["x"] < self.position_limits["x"][1]:  
                self.desired_position["x"] += 1

        if msg.axes[1] < 0:
            self.lastbutton = 100
            if self.desired_position["x"] > self.position_limits["x"][0]:
                self.desired_position["x"] -= 1

        if msg.buttons[0] > 0:
            self.lastbutton = 100
            if self.desired_position["z"] < self.position_limits["z"][1]:  
                self.desired_position["z"] += 1

        if msg.buttons[2] > 0:
            self.lastbutton = 100
            if self.desired_position["z"] > self.position_limits["z"][0]:
                self.desired_position["z"] -= 1

        # claw
        if msg.buttons[1] > 0:
            self.lastbutton = 100
            if self.robot_joints["g"] < self.joints_limits["g"][1]:  
                self.robot_joints["g"] += 1

        if msg.buttons[3] > 0:
            self.lastbutton = 100
            if self.robot_joints["g"] > self.joints_limits["g"][0]:
                self.robot_joints["g"] -= 1

        # velocity
        if msg.buttons[4] > 0:
            self.lastbutton = 100
            if self.robot_joints["v"] > self.joints_limits["v"][0]:
                self.robot_joints["v"] -= 1

        if msg.buttons[5] > 0:
            self.lastbutton = 100
            if self.robot_joints["v"] < self.joints_limits["v"][1]:  
                self.robot_joints["v"] += 1

        # store pose memory of file
        if msg.buttons[8] > 0:
            # assure that we only process 9 one time
            if self.lastbutton == 8:
                return
            self.current_poses.append(self.robot_joints.copy())
            print(self.current_poses)
            rospy.loginfo("Record next pose")
            self.lastbutton = 8

        if msg.buttons[9] > 0:
            if self.lastbutton == 9:
                return
            self.save_pose_sequence()
            self.lastbutton = 9

        # calculate inverse kinematics to self.robot_joints 
        self.ik()

        # apply defined joint limits
        self.apply_joint_limits()

        #print(self.robot_joints)

    def publish_robot_state(self, state):
        array_msg = UInt16MultiArray()
        array_msg.data = [state["y"], state["x"], state["z"], state["g"]]
        self.pub_motors.publish(array_msg)

    def loop_node(self):
        self.publish_robot_state(self.robot_joints)
        self.allow_joystick_input = True

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def load_cb(self, msg):
        filename = msg.data + '.yaml'
        rospy.loginfo("trying to load " + filename + '...')
        with open(self.path+"/stored_poses/"+filename, 'r') as stream:
            try:
                robot_poses = yaml.load(stream, Loader=yaml.FullLoader)
                rospy.loginfo("Playing sequence " + msg.data)
                self.play_pose_sequence(robot_poses)
            except yaml.YAMLError as exc:
                print(exc)

    def save_pose_sequence(self):
        rospy.loginfo("Saving Sequence of Poses")
        print ("Saving Sequence of Poses. Choose Filename:")
        filename = raw_input()
        yaml.dump(self.current_poses, open(self.path + "/stored_poses/" + filename+".yaml", "w"), default_flow_style=False)
        rospy.loginfo(self.path + "/" + filename+".yaml saved...")
        self.current_poses = []

    def play_pose_sequence(self, sequence):
        self.play_sequence = True
        # add current state
        self.current_poses = []
        self.current_poses.append(self.robot_joints.copy())
        poses = self.current_poses
        poses[0]["v"] = 0.0
        poses += sequence
        #print(poses)
        #time.sleep(5.05)
        for i in range(0, len(poses) - 1):
            goal_reached = False
            goal_reached_x = False
            goal_reached_y = False
            goal_reached_z = False
            goal_reached_g = False
            rospy.loginfo("Next Pose: ")
            increment_x = float((poses[i+1]["x"] - poses[i]["x"]))
            increment_y = float((poses[i+1]["y"] - poses[i]["y"]))
            increment_z = float((poses[i+1]["z"] - poses[i]["z"]))
            increment_g = float((poses[i+1]["g"] - poses[i]["g"]))
            increment_v = float((poses[i+1]["v"] - poses[i]["v"]))
            if increment_x > 0:
                increment_x = 1
            else:
                increment_x = -1
            if (increment_y > 0):
                increment_y = 1
            else :
                increment_y = -1
            if (increment_z > 0):
                increment_z = 1
            else :
                increment_z = -1
            if (increment_g > 0):
                increment_g = 1
            else :
                increment_g = -1
            if (increment_v > 0):
                increment_v = 1
            else :
                increment_v = -1
            x = poses[i]["x"]
            y = poses[i]["y"]
            z = poses[i]["z"]
            g = poses[i]["g"]
            v = poses[i]["v"]
            print poses[i]
            while not goal_reached:
                v += increment_v * self.max_acceleration
                #print v, increment_v 
                #time.sleep(1)
                if (v > poses[i+1]["v"] and increment_v > 0 or v < poses[i+1]["v"] and increment_v < 0):
                    v = poses[i+1]["v"]
                x += increment_x * v / self.resolution
                y += increment_y * v / self.resolution
                z += increment_z * v / self.resolution
                g += increment_g * v / self.resolution
                if (x > poses[i+1]["x"] and increment_x > 0 or x < poses[i+1]["x"] and increment_x < 0):
                    x = poses[i+1]["x"]
                    goal_reached_x = True
                if (y > poses[i+1]["y"] and increment_y > 0 or y < poses[i+1]["y"] and increment_y < 0):
                    y = poses[i+1]["y"]
                    goal_reached_y = True
                if (z > poses[i+1]["z"] and increment_z > 0 or z < poses[i+1]["z"] and increment_z < 0):
                    z = poses[i+1]["z"]
                    goal_reached_z = True
                if (g > poses[i+1]["g"] and increment_g > 0 or g < poses[i+1]["g"] and increment_g < 0):
                    g = poses[i+1]["g"]
                    goal_reached_g = True
                goal_reached = goal_reached_x and goal_reached_y and goal_reached_z and goal_reached_g

                self.robot_joints["x"] = x
                self.robot_joints["y"] = y
                self.robot_joints["z"] = z
                self.robot_joints["g"] = g
                self.robot_joints["v"] = v

                self.publish_robot_state(self.robot_joints)
                rospy.sleep(1.0 / self.resolution)
        self.play_sequence = False

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('joystick_servo_control', anonymous=True)
    joyctl = JoystickServoControl()
    rate = rospy.Rate(joyctl.rate)
    while not rospy.is_shutdown():
        joyctl.loop_node()
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)