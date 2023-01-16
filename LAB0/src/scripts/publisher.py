#!/usr/bin/env

import rospy 
from std_msgs.msg import String

def speak_loud():
    rospy.init_node('publisher_node', anonymous = True)
    pub = rospy.Publisher('talking_topic', String, queue_size=10)
    rate = rospy.Rate(1)
    rospy.loginfo("publisher node start")
    while not rospy.is_shutdown():
        msg = "hello, this is dayuan wei's lab0 assignment."
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        speak_loud()
    except rospy.ROSInternalException:
        pass