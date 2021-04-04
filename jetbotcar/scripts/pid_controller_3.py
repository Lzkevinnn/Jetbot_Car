#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from jetbotcar.msg import Jetdrivemsg # float64 left, right
from jetbotcar.msg import jetRacerDriveMsg # float64 throttle, steering
from jetbotcar.msg import jetErrorMsg # float64 lateralError
import rospy
import time

from std_msgs.msg import Float64

######
import sys
import math
import numpy as np


#PID CONTROL PARAMS
kp = 0
kd = 0
ki = 0
lateralError = 0.0
previousTime = 0.0
Setpoint = 0.0 # follow the middle of the line is set as 0.0

# INITIALISE THE THROTTLE AND STEERING CMDs
throttleCmd = 0.0
steeringCmd = 0.0
steeringGain = 0.0


def pidCallback(msg):
    
    global previousTime
    global Setpoint
    global lateralError
    global steeringGain
    global throttleCmd


    global kp
    global ki
    global kd


    publish = bool(True)

    # assign error messages from image processing to lateralError
    lateralError = msg.lateralError

    # Use kp, ki & kd to implement a PID controller for 

    # Time stamp
    currentTime = rospy.get_rostime()
    elapsedTime = currentTime - previousTime

    # Compute all the working error variables, careful with the sign convension
    error = Setpoint - lateralError # here lateral error is the error input from image processing
    cumError += error * elapsedTime #
    rateError = (error - lastError)/elapsedTime

    # PID output computation
    output = kp*error + ki*cumError + kd*rateError

    # Remember some variables for next time
    lastError = error
    previousTime =currentTime

    # output conditions need to be modified

    ###################################
    if output > 0 and output < 5:
        throttleCmd = 0.5
        steeringCmd = output * steeringGain # probably 
        
    elif output > 5 and output < 10:
        throttleCmd = 0.5
        steeringCmd = output * steeringGain

    elif output > -5 and output < 0:
        throttleCmd = 0.5
        steeringCmd = output * steeringGain

    elif output > -10 and output < -5:
        throttleCmd = 0.5
        steeringCmd = output * steeringGain

    elif output == 0:
        throttleCmd = 0.5
        steeringCmd = output * steeringGain
    else:
        publish = bool(False)
    ###################################


    # if publish:
    #     self.publishJetracer(throttle, steering)


def main():
    global throttleCmd, steeringCmd

    # initialise pid_controller node
    rospy.init_node("pid_controller", anonymous=True)
    # subscribes to the lateral_error topic
    rospy.Subscriber("lateral_error", Float64, pidCallback)

    # # Get topic name from the yaml file 
    # (just a topic title, doesn't include anything)
    driveTopic = rospy.get_param("/jetRacerDriveNode/jetracer_drive_topic")

    # jetRacerDriveMsg contains throttle and steering cmds
    drive_pub = rospy.Publisher(driveTopic,jetRacerDriveMsg, queue_size=10)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():

        # Write a publishing function
        drive_msg = jetRacerDriveMsg()

        drive_msg.throttle = throttleCmd
        drive_msg.steering = steeringCmd

        drive_pub.publish(drive_msg)

        # print("Publishing control commands for throttle and steering")
        rate.sleep()

if __name__=='__main__':
	main()

