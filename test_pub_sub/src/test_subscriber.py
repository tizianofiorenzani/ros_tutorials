#!/usr/bin/env python

import rospy
from std_msgs.msg import String

#-- String has only one field: data

def subscriber():
    #--- queue_size argument limits the amount of queued messages if any
    #--- subscriber is not receiving them fast enough
    sub     = rospy.Subscriber('string_publish', String, callback_function)
    rospy.spin()

def callback_function(message):
    rospy.loginfo("I received: %s"%message.data)


if __name__ == "__main__":
    rospy.init_node("simple_subscriber")
    subscriber()
