#!/usr/bin/env python3

import rospy
# we are going to read turtlesim/Pose messages this time
from turtlesim.msg import Pose
#importing the new message from our package
from robotics_lab1.msg import Turtlecontrol
# for radians to degrees conversions
import math
from geometry_msgs.msg import Twist


# Constants
CONTROL_TOPIC = '/turtle1/control_params'
VELOCITY_TOPIC = '/turtle1/cmd_vel'

# Global variables
desired_position = 0.0
control_gain = 0.0

def control_params_callback(data):
    global desired_position, control_gain
    desired_position = data.xd
    control_gain = data.kp

def pose_callback(data):
    global desired_position, control_gain

    # Calculate the proportional control
    error = desired_position - data.x
    velocity_command = control_gain * error

    # Publish the velocity command
    vel_msg = Twist()
    vel_msg.linear.x = velocity_command
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('control_node', anonymous=True)

    # Subscribe to the control parameters topic
    rospy.Subscriber(CONTROL_TOPIC, Turtlecontrol, control_params_callback)

    # Subscribe to the turtle position topic
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    # Publisher for velocity command
    vel_pub = rospy.Publisher(VELOCITY_TOPIC, Twist, queue_size=10)

    # Set control loop frequency to 10 Hz
    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Continue the control loop
        loop_rate.sleep()
