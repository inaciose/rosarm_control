#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import JointState

motors_llim = 0
motors_ulim = 180

# define ignore (0) direction (-/+) and multiplication for each joint received on message
actuated_joints = [-2, -1, 0, 1, 0, 1, 0] # default for ebamk2

pub = rospy.Publisher('motors',  UInt16MultiArray, queue_size=10)

def deg(rad):
    #return 180 / math.pi * rad
    rad = rad + 1.578
    return rad * 57.2958

def callback(msg):
    global motors_llim
    global motors_ulim
    global actuated_joints

    # Calculate angles to sent to motors
    i = 0
    motors_val = UInt16MultiArray()
    #print msg.position
    #motors_val.data = [value for value in msg.position]
    for value in msg.position:
        # Skip mimic joints
        if actuated_joints[i] != 0:
            # Set angle based on transmition
            motors_val.data.append(value * actuated_joints[i])
        i = i + 1

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

    #print(motors_val)

    pub.publish(motors_val)
    
def listener():

    global motors_llim
    global motors_ulim
    global actuated_joints

    rospy.init_node('joint_state_controller', anonymous=True)
    
    motors_llim = rospy.get_param('~motors_llim', [0, 0, 0])
    motors_ulim = rospy.get_param('~motors_ulim', [180, 180, 180])

    # if param type="yaml" on launch doesnt work need to change defaults in next line
    actuated_joints = rospy.get_param('~actuated_joints', [-2, -1, 0, 1, 0, 1, 0])

    #rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, callback)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
