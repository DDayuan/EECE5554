#!/usr/bin/env

import rospy 
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard: %s", data.data )

def listen_careful():
    rospy.init_node('lsubescriber_node', anonymous = True)
    rospy.loginfo("subscriber_node start")
    rospy.Subscriber('talking_topic', String, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listen_careful()
    except rospy.ROSInternalException:
        pass