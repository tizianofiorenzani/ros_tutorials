#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():

    pub     = rospy.Publisher('string_publish', String, queue_size=10)

    rate    = rospy.Rate(1)

    msg_to_publish  = String()

    counter = 0

    while not rospy.is_shutdown():
        string_to_publish = "Publishing %d"%counter
        counter += 1

        msg_to_publish.data = string_to_publish
        pub.publish(msg_to_publish)

        rospy.loginfo(string_to_publish)

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("simple_publisher")
    publisher()
