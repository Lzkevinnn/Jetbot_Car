#!/usr/bin/env python
import rospy
from jetbotcar.msg import Jetdrivemsg # float64 left, right
from jetbotcar.msg import jetRacerDriveMsg # float64 throttle, steering
from jetbotcar.msg import jetErrorMsg # float64 lateralError
# from scipy import integrate

import rospy
import time

from std_msgs.msg import Float64, String

######
import sys
import math
import numpy as np




#PID CONTROL PARAMS
kp = 1
kd = 0.5
ki = 0
# lateralError = 0.0
# cumError = 0.0
Setpoint = 0.0 # follow the middle of the line is set as 0.0

Ts = 1
lastError = 0.0

# INITIALISE THE THROTTLE AND STEERING CMDs
throttleCmd = 0.0
steeringCmd = 0.0
steeringGain = 0.0


def pidCallback(msg):

    # global startTime
    global previousTime
    global currentTime
    global Setpoint
    global Ts
    # global cumError
    global lastError
    # global lateralError

    global kp, ki, kd
    global throttleCmd, steeringCmd

    cumError = 0.0

    publish = bool(True)

    lateralErrorCmd = msg.data # assign error messages from image processing to lateralError
   
    # Use kp, ki & kd to implement a PID controller for 

    # Time stamp

    currentTime = rospy.get_time()
    

    elapsedTime = currentTime - previousTime
    

    # Compute all the working error variables, careful with the sign convension
    error = lateralErrorCmd #lateral error is the error input from image processing
    cumError = cumError + error * elapsedTime
    # cumError = integrate.quad(error, 0, 0.1)
    rateError = (error - lastError)/elapsedTime
    
    # PID output computation
    pidOutput = kp * error + ki * cumError + kd * rateError # need satruation


    # rospy.loginfo("Current time: %.2f", currentTime)
    # rospy.loginfo("Elapsed time: %.2f", elapsedTime)
    # rospy.loginfo("lateral error: %.2f", error)
    # rospy.loginfo("Cumulative Error: %.2f", cumError)
    # rospy.loginfo("rate error: %.2f", rateError)
    # rospy.loginfo("PID pidOutput: %.2f\n", pidOutput)


    # Remember some variables for next time
    lastError = error
    previousTime = currentTime



    # output conditions need to be modified
    ###################################
    if pidOutput > 0 and pidOutput < 1:
        throttleCmd = 0.0
        steeringCmd = pidOutput # probably 
        
    elif pidOutput > 1:
        throttleCmd = 0.0
        steeringCmd = 0.8 # saturation the max steering, condition need further calibration

    elif pidOutput > -1 and pidOutput < 0:
        throttleCmd = 0.0
        steeringCmd = pidOutput

    elif pidOutput < -1:
        throttleCmd = 0.0
        steeringCmd = -0.8 # saturation the max steering, condition need further calibration

    elif pidOutput == 0:
        throttleCmd = 0.0
        steeringCmd = pidOutput
    # no lane detection message output, 
    # the car will stop moving and waiting to be put back on track
    elif pidOutput == 1234: 
        throttleCmd = 0.0
        steeringCmd = 0.0
    
    else:
        publish = bool(False)
    ###################################

def stopCallback(msg):
    global throttleCmd, steeringCmd
    
    if msg.data == "n":
        rospy.on_shutdown(pidCallback)
        rospy.on_shutdown(main)
        throttleCmd = 0.0
        steeringCmd = 0.0
        rospy.loginfo("vehicle has been stopped")

# def publishCmdJetracer(msg):

#     drive_msg = jetRacerDriveMsg()
#     drive_msg.throttle = throttleCmd
#     drive_msg.steering = steeringCmd

#     # drive_pub.publish(drive_msg) # from now don't publish drive_msg to LLC
    

def main():
    global throttleCmd, steeringCmd
    global currentTime
    global previousTime

    # initialise pid_controller node
    rospy.init_node("pid_controller", anonymous=True)
    previousTime = rospy.get_time() # startTime is constant now, stamped straight after init_node

    # subscribes to the lateral_error topic
    rospy.Subscriber("/lateral_error", Float64, pidCallback)

    # *ADD* create the subscriber to the keyboard node
    keyTopic = rospy.get_param("/jetRacerDriveNode/keyboard_topic")
    rospy.Subscriber(keyTopic, String, stopCallback)

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

        # drive_pub.publish(drive_msg) # from now don't publish drive_msg to LLC

        # print("Publishing control commands for throttle and steering")
        rate.sleep()

if __name__=='__main__':
	main()

