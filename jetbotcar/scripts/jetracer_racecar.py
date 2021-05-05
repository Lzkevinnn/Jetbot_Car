
from adafruit_servokit import ServoKit 
from jetbotcar.msg import Jetdrivemsg # float64 left, right
from jetbotcar.msg import jetRacerDriveMsg # float64 throttle, steering
import rospy
import time



class jetracer:

    i2c_address1 = 0x40
    i2c_address2 = 0x60
    steering_gain = 0.8 # 0.65
    steering_offset = 0.02 # 0.7 would be in the middle
    steering_channel = 0
    throttle_gain = 0.8

    def __init__(self , *args, **kwargs):
        # Set channels to the number of servo channels on your kit.
        # 16 for Shield/HAT/Bonnet  
        # print("jetracerLowLevel node initialising")
        self.kit = ServoKit(channels=16, address=self.i2c_address1)
        self.motor= ServoKit(channels=16, address=self.i2c_address2)
        self.motor._pca.frequency = 1600
        self.steering_motor = self.kit.continuous_servo[self.steering_channel]
        # print("jetracerLowLevel motor initialised")
        rospy.init_node('jetracerLowLevel')
        	
        self.driveTopic = rospy.get_param("/jetRacerDriveNode/jetracer_drive_topic")
        #!!!!
        self.commandSub = rospy.Subscriber(self.driveTopic,jetRacerDriveMsg, self.driveCallBack)
        rospy.loginfo("Robot ready to operate, you can control using W, A, S, D and space bar\n")
    
    def driveCallBack(self, msg):
        # rospy.loginfo("driveCallBack")
        steeringCmd = max(min(msg.steering , 1.0), -1.0)
        throttleCmd = max(min(msg.throttle , 1.0), -1.0)
        self.on_steering(steeringCmd)
        self.on_throttle(throttleCmd)
        




    def on_steering(self, steering_angle_cmd):
        self.steering_motor.throttle = steering_angle_cmd*self.steering_gain + self.steering_offset

    def on_throttle(self, throttleValue):
        if throttleValue > 0:
            self.motor._pca.channels[0].duty_cycle = int(0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[1].duty_cycle = 0xFFFF
            self.motor._pca.channels[2].duty_cycle = 0
            self.motor._pca.channels[3].duty_cycle = 0
            self.motor._pca.channels[4].duty_cycle = int(0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[7].duty_cycle = int(0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[6].duty_cycle = 0xFFFF
            self.motor._pca.channels[5].duty_cycle = 0
        else:
            self.motor._pca.channels[0].duty_cycle = int(-0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[1].duty_cycle = 0
            self.motor._pca.channels[2].duty_cycle = 0xFFFF
            self.motor._pca.channels[3].duty_cycle = int(-0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[4].duty_cycle = 0
            self.motor._pca.channels[7].duty_cycle = int(-0xFFFF * (throttleValue * self.throttle_gain))
            self.motor._pca.channels[6].duty_cycle = 0
            self.motor._pca.channels[5].duty_cycle = 0xFFFF
    
    

