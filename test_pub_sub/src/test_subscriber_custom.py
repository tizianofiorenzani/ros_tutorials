#!/usr/bin/env python

import rospy
from test_pub_sub.msg import test_custom_msg

def subscriber():
    sub = rospy.Subscriber('string_publish', test_custom_msg, callback_function)

    rospy.spin()

def callback_function(message):
    string_received = message.data
    counter_received = message.counter
    rospy.loginfo("I received: %d"%counter_received)

if __name__ == "__main__":
    rospy.init_node("simple_subscriber")
    subscriber()
