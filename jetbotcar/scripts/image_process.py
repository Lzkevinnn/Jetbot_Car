#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def image_process():
    pub = rospy.Publisher('lateral_error', String, queue_size=10)
    rospy.init_node('image_process', anonymous=True)

    rospy.spin()


if __name__ == '__main__':
    image_process()   


