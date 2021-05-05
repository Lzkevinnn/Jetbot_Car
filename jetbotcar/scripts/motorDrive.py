#!/usr/bin/env python3

import rospy
import time 
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from jetbotcar.msg import Jetdrivemsg

def set_speed(motor_ID, value):
    max_pwm = 115 #115.0
    speed = int(min(max(abs(value*max_pwm), 0), max_pwm))
    
    if motor_ID == 1:
        motor = motor_left
    elif motor_ID == 2:
        motor = motor_right
    else:
        rospy.logerror('set_speed(%d, %f) -> invalid motor_ID = %d', motor_ID, value, motor_ID)
        return

    motor.setSpeed(speed)

    if value > 0:
        motor.run(Adafruit_MotorHAT.FORWARD)
    else:
        motor.run(Adafruit_MotorHAT.BACKWARD)
  

def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)
    
	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)



# simple string commands (left/right/forward/backward/stop)
def driveCallback(msg):
	#rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)
    rospy.loginfo('left wheel speed command : %f \n right wheel speed command: %f\n', msg.left, msg.right)

    rightMotorSpeedRef = max(-1.0 , (min(1 ,msg.right)))
    leftMotorSpeedRef  = max(-1.0 , (min(1 , msg.left)))
    
    set_speed(motor_left_ID,  leftMotorSpeedRef)
    set_speed(motor_right_ID,  rightMotorSpeedRef)




if __name__ == "__main__":
    motor_driver  = Adafruit_MotorHAT(i2c_bus=1)

    motor_left_ID = 1
    motor_right_ID = 2

    motor_left  = motor_driver.getMotor(motor_left_ID)
    motor_right = motor_driver.getMotor(motor_right_ID)

    all_stop()
    rospy.init_node('motorDriveNode')    
    drive_topic = rospy.get_param("/motorDriveNode/diff_drive_topic")
    rospy.Subscriber(drive_topic, Jetdrivemsg, driveCallback)

    rospy.spin()
    all_stop()
