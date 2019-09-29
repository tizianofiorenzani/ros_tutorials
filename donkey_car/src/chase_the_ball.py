#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

Subscribes to 
    /blob/point_blob
    
Publishes commands to 
    /dkcar/control/cmd_vel    

"""
import math, time
import rospy
from geometry_msgs.msg import Twist

DIST_STEER_ENGAGE   = 1.2
DIST_BREAK          = 0.4

DIST_LAT_ENGAGE     = 0.4

K_FRONT_DIST_TO_SPEED   = 1.0
K_LAT_DIST_TO_STEER     = 2.0

TIME_KEEP_STEERING      = 1.5

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class ChaseBall():
    def __init__(self):
        
        self.blob_x         = 0.0
        self.blob_y         = 0.0
        self._time_detected = 0.0
        
        self.sub_center = rospy.Subscriber("/blob/point_blob", Point, self.update_ball)
        rospy.loginfo("Subscribers set")
        
        self.pub_twist = rospy.Publisher("/dkcar/control/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        
        self._message = Twist()
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0
        
    @property
    def is_detected(self): return(time.time() - self._time_detected > 1.0)
        
    def update_ball(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        
        # rospy.loginfo("Ball detected: %.1f  %.1f "%(self.blob_x, self.blob_y))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """
        steer_action   = 0.0
        
        if self.is_detected:
            #--- Apply steering, proportional to how close is the object
            steer_action   = K_LAT_DIST_TO_STEER*self.blob_x
            steer_action   = saturate(steer_action, -1.5, 1.5)
            rospy.loginfo("Steering command %.2f"%steer_action)
            
        return (steer_action)
        
    def run(self):
        
        #--- Set the control rate
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            #-- Get the control action
            throttle_action = 1.0 #-- Multiplicates the manual command
            steer_action    = self.get_control_action() #-- Adds to the manual command
            
            rospy.loginfo("Steering = %3.1f"%(steer_action))
            
            #-- update the message
            self._message.linear.x  = throttle_action
            self._message.angular.z = steer_action
            
            #-- publish it
            self.pub_twist.publish(self._message)

            rate.sleep()        
            
if __name__ == "__main__":

    rospy.init_node('obstacle_avoid')
    
    obst_avoid     = ObstAvoid()
    obst_avoid.run()            