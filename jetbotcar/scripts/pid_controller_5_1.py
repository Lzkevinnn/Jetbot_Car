#!/usr/bin/env python
import rospy
from jetbotcar.msg import Jetdrivemsg # float64 left, right
from jetbotcar.msg import jetRacerDriveMsg # float64 throttle, steering
from jetbotcar.msg import jetErrorMsg # float64 lateralError
# from scipy import integrate

import rospy
import time

from std_msgs.msg import Float64, String, Float32MultiArray

######
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

#PID CONTROL PARAMS
# kp = 2.7#  #0.0035
# ki = 0.05
# kd = 0.5 # 0.5 preiously

kp = 3#  #0.0035
ki = 0.115
kd = 0.125# 0.5 preiously

# kp = 2.3#  #0.0035
# ki = 0.0
# kd = 0.0# 0.5 preiously


# INITIALISE THE THROTTLE AND STEERING Cmds
pidOutput = 0.0
pidOutput_original = 0.0
throttleCmd = 0.0
steeringCmd = 0.0
steeringGain = 0.0
cumError = 0.0
lastError = 0.0


windupMax = 1.0
publish = bool(False)

def startstopCallback(msg):
    rospy.loginfo("startstopCallback")
    global publish
    if msg.data == "i":
        publish = bool(True)
        rospy.loginfo("operating racer car now")
    elif msg.data == "n":
        publish = bool(False)
        rospy.loginfo("publish: %s", publish)
        

        throttleCmd = 0.0
        steeringCmd = 0.0
        publishCmdJetracer(throttleCmd, steeringCmd)
        rospy.loginfo("Racer car has been stopped")
        rospy.loginfo("throttle: %.2f", throttleCmd)
        rospy.loginfo("steering: %.2f\n", steeringCmd)


def pidCallback(msg):

    global previousTime
    global currentTime
    global cumError
    global lastError
    # global lateralError

    global kp, ki, kd
    global throttleCmd, steeringCmd
    global publish
    global pidOutput
    global pidOutput_original
    global windupMax

    lateralErrorCmd = msg.data[0] # assign error messages from image processing to lateralError
    headingErrorCmd = msg.data[1] 

    # Time stamp
    currentTime = rospy.get_time()
    elapsedTime = currentTime - previousTime
    
    # Compute all the working error variables, careful with the sign convension
    error = lateralErrorCmd/1000.0
    cumError += error * elapsedTime
    rateError = (error - lastError)/elapsedTime
    

    #  anti-windup for integral item
    if cumError > windupMax:
        cumError = windupMax
    elif cumError < - windupMax:
        cumError = windupMax

    # PID output computation
    pidOutput_original = (kp * error + ki * cumError + kd * rateError)
    pidOutput = (kp * error + ki * cumError + kd * rateError) 

    ##########
    # print check codes
    # rospy.loginfo("Current time: %.2f", currentTime)
    # rospy.loginfo("Elapsed time: %.2f", elapsedTime)
    rospy.loginfo("lateral error: %.2f", lateralErrorCmd)
    # rospy.loginfo("Cumulative Error: %.2f", cumError)
    # rospy.loginfo("rate error: %.2f\n", rateError)
    # rospy.loginfo("PID pidOutput: %.2f", pidOutput_original)
    # rospy.loginfo("PID scaled output: %.2f\n", pidOutput)
    #########

    # Remember some variables for next time
    lastError = error
    previousTime = currentTime

    # steeringCmd = pidOutput
    # throttleCmd = -0.55
    throttleCmd = -0.55
    steeringCmd = 1.0
    
    
    # if headingErrorCmd > 40:
    #     throttleCmd = -0.45
    #     steeringCmd = pidOutput
    # elif headingErrorCmd < -40:
    #     throttleCmd = -0.45
    #     steeringCmd = pidOutput
    # else:
    #     throttleCmd = -0.52
    #     steeringCmd = pidOutput
    

    if publish:
        
        # rospy.loginfo("publish: %s", publish)
        publishCmdJetracer(throttleCmd, steeringCmd)
    else:
        # pass
        # publish = bool(False)
        publishCmdJetracer(0, 0)
        # rospy.loginfo("throttle: %.2f", throttleCmd)
        # rospy.loginfo("steering: %.2f\n", steeringCmd)



def publishCmdJetracer(throttleCmd, steeringCmd): # publish function
    global drive_pub

    drive_msg = jetRacerDriveMsg()
    drive_msg.throttle = throttleCmd
    drive_msg.steering = steeringCmd
    drive_pub.publish(drive_msg)
    

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
    rospy.loginfo(previousTime)


    # subscribes to the lateral_error topic
    rospy.Subscriber("/Error_msg", Float32MultiArray, pidCallback)

    # *ADD* create the subscriber to the keyboard node
    keyTopic = rospy.get_param("/jetRacerDriveNode/keyboard_topic")
    rospy.Subscriber(keyTopic, String, startstopCallback)
    # rospy.Subscriber(keyTopic, String, startCallback)

    # # Get topic name from the yaml file 
    # (just a topic title, doesn't include anything)
    driveTopic = rospy.get_param("/jetRacerDriveNode/jetracer_drive_topic")

    # jetRacerDriveMsg contains throttle and steering cmds
    drive_pub = rospy.Publisher(driveTopic,jetRacerDriveMsg, queue_size=0)

    # try plot 
    cmdlateral_pub = rospy.Publisher("cmdPlt_lateral", Float64, queue_size=2)
    cmdlong_pub = rospy.Publisher("cmdPlt_longi", Float64, queue_size=2)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        cmdlateral_pub.publish(steeringCmd)
        cmdlong_pub.publish(throttleCmd)
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


