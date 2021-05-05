#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from jetbotcar.msg import Jetdrivemsg # float64 left, right
from jetbotcar.msg import jetRacerDriveMsg # float64 throttle, steering
import rospy
import time

######
import sys
import math
import numpy as np


#PID CONTROL PARAMS
kp = 0
kd = 0
ki = 0
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

# def pid_pub(): # publish message to the lower level motor controller

# Get prameters from the yaml file
driveTopic = rospy.get_param("/jetRacerDriveNode/jetracer_drive_topic")


pub = rospy.Publisher(driveTopic,jetRacerDriveMsg, queue_size=10)

rospy.loginfo("pid message")



def pidAlgorithm(): #main PID algorithm
    rospy.loginfo("pid message")


def error_sub():
    rospy.init_node("pid_controller")
    rospy.Subscriber('lateral_error', String, pidAlgorithm)
    rospy.spin()


if __name__ == '__main__':
    error_sub()    
