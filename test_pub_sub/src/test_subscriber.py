#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def subscriber():
    sub = rospy.Subscriber('string_publish_2', String, callback_function)

    rospy.spin()

def callback_function(message):
    rospy.loginfo("I received: %s"%message.data)

if __name__ == "__main__":
    rospy.init_node("simple_subscriber")
    subscriber()
