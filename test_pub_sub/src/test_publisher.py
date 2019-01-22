#!/usr/bin/env python

import rospy
from std_msgs.msg import String

#-- String has only one field: data

def publisher():

    #--- queue_size argument limits the amount of queued messages if any
    #--- subscriber is not receiving them fast enough
    pub     = rospy.Publisher('string_publish', String, queue_size=10)
    rate    = rospy.Rate(1) #-- publishing rate in Hz
    msg_to_publish = String()

    print "Initial object to be published"
    print msg_to_publish
    print "****************"
    counter = 0

    #--- While loop until interrupted
    while not rospy.is_shutdown():
        string_to_publish = "Publishing %d"%counter
        counter += 1

        #--- Assign the string to the data field
        msg_to_publish.data = string_to_publish
        rospy.loginfo(string_to_publish)
        pub.publish(msg_to_publish)

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("simple_publisher")
    publisher()
