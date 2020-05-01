#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import JointState

pub = rospy.Publisher('joint_states', JointState, queue_size=10)
joints = JointState()
joints_dir = [-1, 1, 1]

def rad(deg):
    return deg * math.pi/180

def callback(motors_val):
    global joints
    motors_val = [value for value in motors_val.data]
    joints.position = [(motors_val[0]-90)/2, 180 -  motors_val[1] - 90, motors_val[2] - (180 - motors_val[1]) + 90 -90]
    joints.position = [rad(joint) for joint in joints.position]
    for i in range(3):
        joints.position[i] = joints.position[i] * joints_dir[i]

def listener():
    global joints
    rospy.init_node('joint_state_publisher', anonymous=True)
    rospy.Subscriber("motors", UInt16MultiArray, callback)

    joints.name =  ['joint_1', 'joint_2', 'joint_3']
    joints.position = [0, 0, 0]
    joints.position = [rad(joint) for joint in joints.position]

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        joints.header.stamp = rospy.Time.now() 
        pub.publish(joints)
        rate.sleep()

if __name__ == '__main__':
    listener()
