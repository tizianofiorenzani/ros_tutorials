#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from tfmini_servo_scanner import *
import math

SERVO_GPIO          = 23
SRV_ANGLE_MIN       = math.radians(-85)
SRV_ANGLE_MAX       = math.radians(85)
SRV_DUTY_ANGLE_MIN  = 2250
SRV_DUTY_ANGLE_MAX  = 750
SRV_TIME_MIN_MAX    = 0.4
LASER_ANGLE_SAMPLES = 50


def tfmini_laserscan_publisher(frame_id):

    scan_pup= rospy.Publisher('tfmini_laser', LaserScan, queue_size=0)

    scan  = LaserScan()

    
    #-- Convention: counter clockwise is positive (left positive, right negative)
    tfminiscanner = TfminiServoScanner(SERVO_GPIO, SRV_ANGLE_MIN, SRV_ANGLE_MAX, 
                                SRV_DUTY_ANGLE_MIN, SRV_DUTY_ANGLE_MAX, LASER_ANGLE_SAMPLES, 
                                SRV_TIME_MIN_MAX)
                                
                                
    #-- Initialize the message
    scan.header.frame_id   = frame_id;
    scan.range_min         = tfminiscanner.laser.distance_min*0.01
    scan.range_max         = tfminiscanner.laser.distance_max*0.01
                                
    
    tfminiscanner.reset_servo()
    time.sleep(1)

    counter = 0

    while not rospy.is_shutdown():
        ini_angle, end_angle, time_increment, angle_increment, ranges = tfminiscanner.scan(scale_factor=0.01, reset=True)
        scan.angle_min = ini_angle
        scan.angle_max = end_angle
        scan.angle_increment = angle_increment
        scan.time_increment = time_increment
        scan.ranges = ranges

        scan_pup.publish(scan)



if __name__ == "__main__":
    rospy.init_node("tfmini_laserscan")
    tfmini_laserscan_publisher(frame_id="map")
