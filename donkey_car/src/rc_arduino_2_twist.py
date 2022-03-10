#!/usr/bin/python

"""
Class for arduino RC interpreter to teleom twist messages
"""
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
import time

class RcArduinoInput():
    def __init__(self):
        rospy.init_node('rc_arduino_proxy')
        #--- Create the Subscriber to RC Throttle commands
        self.ros_sub_rc_throttle  = rospy.Subscriber("/rc_throttle", UInt16, self.update_throttle)
        self.ros_sub_rc_steering  = rospy.Subscriber("/rc_steering", UInt16, self.update_steering)
        rospy.loginfo("> Subscriber corrrectly initialized")
        
        self._pwm_min = 900
        self._pwm_max = 2100
        
        self._sign_throttle = 1
        self._sign_steering = 1
        
        self._offset_throttle = 0
        self._offset_steering = 0
        
        self._values_received = 0
        
        self.ros_pub_twist   = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        self._timeout_s = 2;
        self._last_time_cmd_rcv = time.time() - self._timeout_s;
        
        self.ros_twist_msg = Twist();

    def update_throttle(self, message):
        self.ros_twist_msg.linear.x = self.pwm_to_adimensional(message.data + self._offset_throttle) * self._sign_throttle
        self._values_received += 1
        self._last_time_cmd_rcv = time.time()
        
    def update_steering(self, message):
        self.ros_twist_msg.angular.z = self.pwm_to_adimensional(message.data + self._offset_steering) * self._sign_steering
        self._values_received += 1
        self._last_time_cmd_rcv = time.time()
        
    def pwm_to_adimensional(self, pwm):
        pwm = max(pwm, self._pwm_min)
        pwm = min(pwm, self._pwm_max)
        pwm = pwm - ((self._pwm_max + self._pwm_min) * 0.5)
        pwm = pwm / ((self._pwm_max - self._pwm_min) * 0.5)
        return pwm
        
    @property
    def is_rc_connected(self):
        return(self._values_received >= 2)        
        
    def run(self):
        #--- Set the control rate
        #rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.is_rc_connected:
                self.ros_pub_twist.publish(self.ros_twist_msg)
                self._values_received = 0

            #rate.sleep()

if __name__ == "__main__":
    rc_proxy  = RcArduinoInput()
    rc_proxy.run()
