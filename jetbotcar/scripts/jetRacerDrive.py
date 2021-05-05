#!/usr/bin/env python3
import rospy
import time 
from jetracer_racecar import jetracer


if __name__ == "__main__":
    car = jetracer()
    
    rospy.spin()
