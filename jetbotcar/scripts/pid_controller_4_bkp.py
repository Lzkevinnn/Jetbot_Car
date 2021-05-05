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
import matplotlib.pyplot as plt

#PID CONTROL PARAMS
kp = 1 # 1 
kd = 0.0 # 0.5 preiously
ki = 0.0

lastError = 0.0
publish = bool(False)


# INITIALISE THE THROTTLE AND STEERING Cmds
pidOutput = 0.0
pidOutput_original = 0.0
throttleCmd = 0.0
steeringCmd = 0.0
steeringGain = 0.0

def startCallback(msg):
    global publish
    if msg.data == "i":
        publish = bool(True)
        rospy.loginfo("operating racer car now")


def pidCallback(msg):

    # global startTime
    global previousTime
    global currentTime
    # global cumError
    global lastError
    # global lateralError

    global kp, ki, kd
    global throttleCmd, steeringCmd
    global publish
    global pidOutput
    global pidOutput_original
    cumError = 0.0

    

    lateralErrorCmd = msg.data # assign error messages from image processing to lateralError
    # Time stamp

    currentTime = rospy.get_time()
    elapsedTime = currentTime - previousTime
    
    # Compute all the working error variables, careful with the sign convension
    error = lateralErrorCmd #lateral error is the error input from image processing
    cumError = cumError + error * elapsedTime
    # cumError = integrate.quad(error, 0, 0.1)
    rateError = (error - lastError)/elapsedTime
    
    # PID output computation
    pidOutput_original = (kp * error + ki * cumError + kd * rateError)
    pidOutput = (kp * error + ki * cumError + kd * rateError)* 0.01 


    # rospy.loginfo("Current time: %.2f", currentTime)
    # rospy.loginfo("Elapsed time: %.2f", elapsedTime)
    # rospy.loginfo("lateral error: %.2f", error)
    # rospy.loginfo("Cumulative Error: %.2f", cumError)
    # rospy.loginfo("rate error: %.2f", rateError)
    # rospy.loginfo("PID pidOutput: %.2f", pidOutput_original)
    # rospy.loginfo("PID scaled output: %.2f\n", pidOutput)


    # Remember some variables for next time
    lastError = error
    previousTime = currentTime

    
    # output conditions need to be modified
    ###################################
    if pidOutput > 0.0 and pidOutput < 1.0:
        throttleCmd = -0.4
        steeringCmd = pidOutput
        
    elif pidOutput > 1.0 and pidOutput < 1000.0:
        throttleCmd = -0.4
        steeringCmd = 0.9 # saturation the max steering, condition need further calibration

    elif pidOutput > -1.0 and pidOutput < 0.0:
        throttleCmd = -0.4
        steeringCmd = pidOutput

    elif pidOutput < -1.0:
        throttleCmd = -0.4
        steeringCmd = -0.9 # saturation the max steering, condition need further calibration

    elif pidOutput == 0.0:
        throttleCmd = -0.4
        steeringCmd = pidOutput
    # no lane detection message output, 
    # the car will stop moving and waiting to be put back on track
    # elif error == 1234.0: 
    #     throttleCmd = 0.0
    #     steeringCmd = 0.0
    
    else:
        publish = bool(False)

    if publish:
        # rospy.loginfo("publish: %s", publish)
        publishCmdJetracer(throttleCmd, steeringCmd)

        # rospy.loginfo("throttle: %.2f", throttleCmd)
        # rospy.loginfo("steering: %.2f\n", steeringCmd)


def stopCallback(msg):
    global throttleCmd, steeringCmd
    global publish 
    if msg.data == "n":
        publish = bool(False)
        rospy.loginfo("publish: %s", publish)
        

        throttleCmd = 0.0
        steeringCmd = 0.0
        publishCmdJetracer(throttleCmd, steeringCmd)
        rospy.loginfo("Racer car has been stopped")
        rospy.loginfo("throttle: %.2f", throttleCmd)
        rospy.loginfo("steering: %.2f\n", steeringCmd)


def publishCmdJetracer(throttleCmd, steeringCmd): # publish function
    global drive_pub

    drive_msg = jetRacerDriveMsg()
    drive_msg.throttle = throttleCmd
    drive_msg.steering = steeringCmd
    drive_pub.publish(drive_msg) #
    

def main():
    global throttleCmd, steeringCmd
    global currentTime
    global previousTime
    global drive_pub
    global pidOutput
    global pidOutput_original

    # initialise pid_controller node
    rospy.init_node("pid_controller", anonymous=True)
    previousTime = rospy.get_time() # startTime is constant now, stamped straight after init_node

    # subscribes to the lateral_error topic
    rospy.Subscriber("/lateral_error", Float64, pidCallback)

    # *ADD* create the subscriber to the keyboard node
    keyTopic = rospy.get_param("/jetRacerDriveNode/keyboard_topic")
    rospy.Subscriber(keyTopic, String, stopCallback)
    rospy.Subscriber(keyTopic, String, startCallback)

    # # Get topic name from the yaml file 
    # (just a topic title, doesn't include anything)
    driveTopic = rospy.get_param("/jetRacerDriveNode/jetracer_drive_topic")

    # jetRacerDriveMsg contains throttle and steering cmds
    drive_pub = rospy.Publisher(driveTopic,jetRacerDriveMsg, queue_size=10)

    # try plot 
    cmdPlt_pub = rospy.Publisher("cmdPlt", Float64, queue_size=10)
    
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        cmdPlt_pub.publish(pidOutput_original)
        # rospy.loginfo("PID scaled output: %.2f\n", pidOutput_original)
        rate.sleep()



    publishCmdJetracer(throttleCmd, steeringCmd)
    rospy.spin()

if __name__=='__main__':
    main()
    # try:
    #     main()
    # except KeyboardInterrupt:
    #     print('Interrupted')
    #     try:
    #         sys.exit(0)
    #     except SystemExit:
    #         os._exit(0)


