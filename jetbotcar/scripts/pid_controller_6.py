#!/usr/bin/env python
import rospy
from jetbotcar.msg import Jetdrivemsg  # float64 left, right
from jetbotcar.msg import jetRacerDriveMsg  # float64 throttle, steering
from jetbotcar.msg import jetErrorMsg  # float64 lateralError
# from scipy import integrate

import rospy
import time

from std_msgs.msg import Float64, String, Float32MultiArray

######
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

lastError = 0.0
publish = bool(False)

pidOutput_original = 0.0
throttleCmd = 0.0


class PID:
    """PID controller."""
    def __init__(self, Kp, Ki, Kd, origin_time=None):
        if origin_time is None:
            origin_time = rospy.get_time()

        # Gains for each term
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Corrections (outputs)
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0

        self.previous_time = origin_time
        self.previous_error = 0.0

    def reset(self):
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0
        self.previous_error = 0.0
        self.previous_time = None

    def update(self, error, current_time=None):
        if current_time is None:
            current_time = rospy.get_time()
        if self.previous_time is None:
            self.previous_time = current_time
        dt = current_time - self.previous_time
        if dt <= 0.0:
            return 0
        de = error - self.previous_error

        self.Cp = error
        self.Ci += error * dt
        self.Cd = de / dt

        self.previous_time = current_time
        self.previous_error = error

        return (
                (self.Kp * self.Cp)  # proportional term
                + (self.Ki * self.Ci)  # integral term
                + (self.Kd * self.Cd)  # derivative term
        )


# initialise pid_controller node
rospy.init_node("pid_controller", anonymous=True)

# Creating objects for lateral and longitudinal control
lateral_pid = PID(3.0, 0.115, 0.125)
# longitudinal_pid = PID(0.8, 2, 0.0)
longitudinal_pid = PID(0.9, 0.2, 0.0)
#Offset to reduce initial acceleration for passenger comfort
Integral_offset = 0.0

#Giving initial value for Ci as -1 so that initially jetracer will slowly build acceleration
longitudinal_pid.Ci = Integral_offset
throttleCmd_pid = 0

# Start and stop function to manually control jetracer
def startCallback(msg):
    rospy.loginfo("startstopCallback")
    global publish
    if msg.data == "i":
        publish = bool(True)
        rospy.loginfo("operating racer car now")
    elif msg.data == "n":
        publish = bool(False)
        rospy.loginfo("publish: %s", publish)

        publishCmdJetracer(0, 0)
        rospy.loginfo("Racer car has been stopped")
        rospy.loginfo("throttle: %.2f", 0)
        rospy.loginfo("steering: %.2f\n", 0)

# Defining mapping variables
def map_range(a, b, s):
    (a1, a2), (b1, b2) = a, b
    return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))

# PID algorithm including lateral and heading error
def pidCallback(msg):
    global pidOutput_original, throttleCmd_pid, throttleCmd
    #Lateral error published from Image processing
    lateral_error = msg.data[0] / 1000.0
    #Heading error published from Image processing
    heading_error = msg.data[1]

    throttleCmd = -0.5
    
    if publish:
        #Steering command for the lateral control which has to be updated for each lateral error subscribed
        steeringCmd = lateral_pid.update(lateral_error)
        #
        pidOutput_original = steeringCmd
        
        
        throttle_ref = sorted((0.4,map_range([0, 90], [1, 0], abs(heading_error)), 0.7))[1]
        
        throttle_error = throttle_ref - throttleCmd_pid
        print("throttleCmd_pid: %.2f" % throttleCmd_pid)
        throttleCmd_pid = sorted((0.4, longitudinal_pid.update(throttle_error), 0.7))[1]
        # throttleCmd_pid = longitudinal_pid.update(throttle_error)

        print("throttleCmd: %.2f  heading_error: %.2f throttle_ref: %.2f Ci: %.2f\n" % (
            throttleCmd_pid, heading_error, throttle_ref,longitudinal_pid.Ci))
        
        # rospy.loginfo("publish: %s", publish)
        throttleCmd = -throttleCmd_pid
        publishCmdJetracer(throttleCmd, steeringCmd)
        # print("steeringCmd %.2f  lateral: %.2f Cp: %.2f  Ci: %.2f" % (
        #     steeringCmd, lateral_error, lateral_pid.Cp, lateral_pid.Ci))

    else:
        lateral_pid.reset()
        longitudinal_pid.reset()
        longitudinal_pid.Ci = Integral_offset
        throttle_ref = 0.0
        throttleCmd_pid = 0.0
        # pass
        # publish = bool(False)
        publishCmdJetracer(0, 0)
        # rospy.loginfo("throttle: %.2f", throttleCmd)
        # rospy.loginfo("steering: %.2f\n", steeringCmd)


def publishCmdJetracer(throttleCmd, steeringCmd):  # publish function
    global drive_pub
    drive_msg = jetRacerDriveMsg()
    drive_msg.throttle = throttleCmd
    drive_msg.steering = steeringCmd
    drive_pub.publish(drive_msg)  #


def main():
    global throttleCmd, steeringCmd
    global currentTime
    global previousTime
    global drive_pub
    global pidOutput
    global pidOutput_original

    previousTime = rospy.get_time()  # startTime is constant now, stamped straight after init_node
    rospy.loginfo(previousTime)

    # subscribes to the lateral_error topic
    rospy.Subscriber("/Error_msg", Float32MultiArray, pidCallback)

    # *ADD* create the subscriber to the keyboard node
    keyTopic = rospy.get_param("/jetRacerDriveNode/keyboard_topic")
    rospy.Subscriber(keyTopic, String, startCallback)
    # rospy.Subscriber(keyTopic, String, startCallback)

    # # Get topic name from the yaml file 
    # (just a topic title, doesn't include anything)
    driveTopic = rospy.get_param("/jetRacerDriveNode/jetracer_drive_topic")

    # jetRacerDriveMsg contains throttle and steering cmds
    drive_pub = rospy.Publisher(driveTopic, jetRacerDriveMsg, queue_size=0)

    # try plot 
    cmdlateral_pub = rospy.Publisher("cmdPlt_lateral", Float64, queue_size=2)
    cmdlong_pub = rospy.Publisher("cmdPlt_longi", Float64, queue_size=2)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        cmdlateral_pub.publish(pidOutput_original)
        cmdlong_pub.publish(throttleCmd)
        # rospy.loginfo("PID scaled output: %.2f\n", pidOutput_original)
        rate.sleep()

    publishCmdJetracer(0, 0)
    rospy.spin()


if __name__ == '__main__':
    main()
    # try:
    #     main()
    # except KeyboardInterrupt:
    #     print('Interrupted')
    #     try:
    #         sys.exit(0)
    #     except SystemExit:
    #         os._exit(0)