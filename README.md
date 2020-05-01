# rosarm_control
ROS package with scripts to control simple robotic arms

This package requires one of the following arm description packages for the specific robotic arm:
- ebamk2_description

The robotic arm must be controlled by a mcu runing compatible firmware availabe in rosarm_firmware

Non ros_control servo based robotic arms firmware subscribe to one of the following message types
- std_msgs/UInt16MultiArray
- sensor_msgs/JointState

Nodes, joystick_servo_control.py and joystick_position_control_eba(_poses).py require one of the following sketches:
- ros_arm_servo_multiarray.ino
- ros_arm_servo_pca9685_multiarray.ino
