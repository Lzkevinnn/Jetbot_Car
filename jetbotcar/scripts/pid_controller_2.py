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
#lateralError = 0.0

previousTime = 0.0
Setpoint = 0.0 # follow the middle of the line is set as 0.0

# def pid_pub(): # publish message to the lower level motor controller

# Get prameters from the yaml file
driveTopic = rospy.get_param("/jetRacerDriveNode/jetracer_drive_topic")

class PIDControl: # don't need class

    def __init__(self):

        # Subscribe to image process
        self.error_sub = rospy.Subscriber('lateral_error',float64, queue_size=10)
        #Publish to drive
        self.drive_pub = rospy.Publisher(driveTopic,jetRacerDriveMsg, queue_size=10) 
    
    # def getRange(self, data, angle):
    #     # data: single message from topic /scan
    #     # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
    #     # Outputs length in meters to object with angle in lidar scan field of view
    #     #make sure to take care of nans etc.
    #     #TODO: implement
    #     return 0.0
    
    def pid_control(self, error, velocity):
        
        global prev_error
        global previousTime
        global Setpoint
        global kp
        global ki
        global kd
        publish = bool(True)


        # Use kp, ki & kd to implement a PID controller for 

        # Time stamp
        currentTime = rospy.get_rostime()
        elapsedTime = currentTime - previousTime

        # Compute all the working error variables, careful with the sign convension
        error = Setpoint - lateralError # here lateral error is the error input from image processing
        cumError += error * elapsedTime #
        rateError = (error - lastError)/elapsedTime

        # PID output computation
        out = kp*error + ki*cumError + kd*rateError

        # Remember some variables for next time
        lastError = error
        previousTime =currentTime

        # output conditions need to be modified

        

        ###################################
        # if error > 0 and error < 5:
        #     throttle = 0.5
        #     steering = 0.5
        # elif error > 5 and error < 10:
        #     throttle = 0.5
        #     steering = 1
        # elif error > -5 and error < 0:
        #     throttle = 0.5
        #     steering = -0.5
        # elif error > -10 and error < -5:
        #     throttle = 0.5
        #     steering = -1
        # elif error == 0:
        #     throttle = 0.5
        #     steering = 0
        # else:
        #     publish = bool(False)
        ###################################


        if publish:
            self.publishJetracer(throttle, steering)


    # Write a publishing function
    def publishJetracer(self, throttle, steering):# ??? how to write publish function
        drive_msg = jetRacerDriveMsg()
        drive_msg.throttle = throttle
        drive_msg.steering = steering
        self.drive_pub.publish(drive_msg)
        

    # def followLeft(self, data, leftDist):
    #     #Follow left wall as per the algorithm 
    #     #TODO:implement
    #     return 0.0 

    # def lidar_callback(self, data):
    #     """ 
    #     """
    #     error = 0.0 #TODO: replace with error returned by followLeft
    #     #send error to pid_control
    #     self.pid_control(error, VELOCITY)


def main(args):
    rospy.init_node("pid_controller", anonymous=True)
    pc = PIDControl()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)


################################################################

# pub = rospy.Publisher(driveTopic,jetRacerDriveMsg, queue_size=10)

# rospy.loginfo("pid message")



# def pidAlgorithm(): #main PID algorithm
#     rospy.loginfo("pid message")


# def error_sub():
#     rospy.init_node("pid_controller")
#     rospy.Subscriber('lateral_error', String, pidAlgorithm)
#     rospy.spin()


# if __name__ == '__main__':
#     error_sub()    
