#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import JointState

motors_llim = 0
motors_ulim = 180

pub = rospy.Publisher('motors',  UInt16MultiArray, queue_size=10)

def deg(rad):
    #return 180 / math.pi * rad
    rad = rad + 1.6
    return rad * 57.2958

def callback(msg):
    global motors_llim
    global motors_ulim

    motors_val = UInt16MultiArray()
    motors_val.data = [value for value in msg.position]
    motors_val.data = [int(deg(value)) for value in motors_val.data]

    # Motors lower limits: 
    i = 0
    for llim in motors_llim:
        if motors_val.data[i] < llim:
            motors_val.data[i] = llim
        i = i + 1

    # Motors upper limits:
    j = 0
    for ulim in motors_ulim:
        if motors_val.data[j] > ulim:
            motors_val.data[j] = ulim
        j = j + 1    

    print(motors_val)

    pub.publish(motors_val)
    
def listener():

    global motors_llim
    global motors_ulim
    rospy.init_node('joint_state_controller', anonymous=True)
    
    motors_llim = rospy.get_param('~motors_llim', [0, 0, 0])
    motors_ulim = rospy.get_param('~motors_ulim', [180, 180, 180])

    rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
