#!/usr/bin/env python

# ROS imports
import rospy
import actionlib
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Header, String
from control_msgs.msg    import FollowJointTrajectoryAction
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg     import JointState

# non-ROS imports
import eba_arm_config as cal
import math

class JointController():
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()

    def __init__(self):
        self.name = rospy.get_name()

        # store angles to send to servos
        self.robot_joints = {}

        # publishers
        self.pub_servos = rospy.Publisher('motors', UInt16MultiArray, queue_size=10) # xsi
        self.publisher = rospy.Publisher('joint_states', JointState, queue_size=10)

        # Set up action client 
        self.action_server = actionlib.SimpleActionServer('%s/follow_joint_trajectory'%self.name, 
                                            FollowJointTrajectoryAction, 
                                            self.do_action_callback, False)
        self.action_server.start()         

        self.rate = rospy.Rate(10) # 10hz
        self.joint_state = JointState()
        self.joint_state.header = Header()
        self.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_6']
        #self.joint_state.name = ['joint_1', 'joint_2', 'joint_3']
        self.joint_state.position = [0, 0, 0, 0] # will be updated in home() before publication in the loop
        #self.joint_state.position = [0, 0, 0] # will be updated in home() before publication in the loop
        self.joint_state.velocity = []
        self.joint_state.effort = []

        # define ignore (0) direction (-/+) and multiplication for each joint received on message
        self.actuated_joints = [-2, -1, 1] # default for ebamk2


        # Initialise
    	self.home()

    def deg(self, rad):
        rad = rad + 1.578
        return rad * 57.2958

    def do_action_callback(self, goal):
        print 'Received goal %d'%len(goal.trajectory.points[0].positions)
        timePassed = rospy.Duration(0)
        self.joint_state.name = goal.trajectory.joint_names

        # we do not handle the claw, need to learn, and need to add fake data
        # we do not have joint_6 in the trajectory message so check and add
        if not "joint_6" in self.joint_state.name:
            self.joint_state.name.append("joint_6")

        #print(goal)

        for point in goal.trajectory.points:
           self.joint_state.header.stamp = rospy.Time.now()
           self.move_arm(goal.trajectory.joint_names, point.positions)
           self.publisher.publish(self.joint_state)
           rospy.sleep(point.time_from_start - timePassed)
           timePassed = point.time_from_start
           #print('step')


        self._result.error_code = 0
        self.action_server.set_succeeded(self._result)
        print('move complete')
   
    def get_joint_goal(self, joint_name, joint_list, goal_list):
        if joint_name in joint_list:
            index = joint_list.index(joint_name)
            return goal_list[index]
        print 'ERROR!! Could not find %s goal'% joint_name

    def set_joint_state(self, joint_name, angle): 
        index = self.joint_state.name.index(joint_name)
        print '%s at %d is %0.5f (%0.2f)' % (joint_name, index, angle, math.degrees(angle))
        self.joint_state.position[index] = angle

    def move_arm(self, names, goal):
        base = self.get_joint_goal('joint_1', names, goal)
        shoulder = self.get_joint_goal('joint_2', names, goal)
        elbow = self.get_joint_goal('joint_3', names, goal)
        #claw = self.get_joint_goal('joint_6', names, goal)
        self.setJointAngles(base, shoulder, elbow)

    def check_min_max(self, minVal, maxVal, test):
        if (test < minVal):
	        test = minVal
	        print "min %d" % (minVal)		
        if (test > maxVal):
	        test = maxVal
	        print "max %d" % (maxVal)		
        return test

    # Helper function to make setting a servo pulse width simpler.
    def set_servo_angle(self, channel, angle):
        self.robot_joints[channel] = int(self.deg(angle))

    def b(self, angle):
        angle = self.check_min_max(cal.MIN_LINK_1, cal.MAX_LINK_1, angle)
        self.set_joint_state('joint_1', angle) 
        # prepare the angle sent to mcu (in degrees)
        angle = angle * self.actuated_joints[0] # joint_1 index
        self.set_servo_angle('joint_1', angle)

    def s(self, angle):
        angle = self.check_min_max(cal.MIN_LINK_2, cal.MAX_LINK_2, angle)
        self.set_joint_state('joint_2', angle) 
        # prepare the angle sent to mcu (in degrees)
        angle = angle * self.actuated_joints[1] # joint_2 index
        self.set_servo_angle('joint_2', angle)

    def e(self, angle):
        angle = self.check_min_max(cal.MIN_LINK_3, cal.MAX_LINK_3, angle)
        self.set_joint_state('joint_3', angle)
        # prepare the angle sent to mcu (in degrees)
        angle = angle * self.actuated_joints[2] # joint_3 index
        self.set_servo_angle('joint_3', angle)

    def c(self, angle):
        angle = self.check_min_max(cal.MIN_LINK_6, cal.MAX_LINK_6, angle)
        self.set_joint_state('joint_6', angle)         
        self.set_servo_angle('joint_6', angle)

    def setJointAngles(self, joint_1, joint_2, joint_3):
        self.b(joint_1)
        self.s(joint_2)
        self.e(joint_3)
        self.publish_servos_state(self.robot_joints)

    def home(self):
        self.b(cal.HOME_LINK_1)
        self.s(cal.HOME_LINK_2)
        self.e(cal.HOME_LINK_3)
        self.c(cal.HOME_LINK_6)
        self.publish_servos_state(self.robot_joints)

    def off(self):
        self.b(cal.HOME_LINK_1)
        self.s(cal.HOME_LINK_2)
        self.e(cal.HOME_LINK_3)
        self.c(cal.HOME_LINK_6)
        self.publish_servos_state(self.robot_joints)

    def publish_servos_state(self, state):
        array_msg = UInt16MultiArray()
        array_msg.data = [state["joint_1"], state["joint_2"], state["joint_3"], state["joint_6"]]
        #array_msg.data = [state["joint_1"], state["joint_2"], state["joint_3"]]
        self.pub_servos.publish(array_msg)

    def loop(self):
        # publish states regularly though this is also done on commands
        while not rospy.is_shutdown():
            self.joint_state.header.stamp = rospy.Time.now()
            self.publisher.publish(self.joint_state)
            self.rate.sleep()
        self.off()

if __name__ == '__main__':
    try:
        rospy.init_node('arm_controller')
        jc = JointController()
        jc.loop()
    except rospy.ROSInterruptException:
        pass



