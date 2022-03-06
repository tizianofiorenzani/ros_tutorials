
/*
 * Read a PWM input signal coming from a digital arduino PIN
 * This is a demonstration that can be later used for connecting
 * to ROS using ROSSerial library
 * 
 * @author: Tiziano Fiorenzani
 */


#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

/// Define input channel pins
static const int CH_THROTTLE_IN = 9;
static const int CH_STEERING_IN = 10;
static const int NUM_CHANNELS   = 2;
static const int PWM_DISCONNECT = 700;

/// Create the arrays
float channel[NUM_CHANNELS];

int chId[NUM_CHANNELS] = 
  { CH_THROTTLE_IN
  , CH_STEERING_IN
  };

/// Define the message and the publishers
std_msgs::UInt16 rc_msg;
ros::Publisher publisher_throttle("rc_throttle", &rc_msg);
ros::Publisher publisher_steering("rc_steering", &rc_msg);
 
void setup() {
  /// Define pins as INPUT
  for(int i = 0; i < NUM_CHANNELS; i++)
  {
    pinMode(chId[i], INPUT);
  }
  nh.initNode();
  nh.advertise(publisher_throttle);
  nh.advertise(publisher_steering);
}
 
void loop() {
  /// Use the function pulseIn and returns the timinng in us between two highs
  /// see: https://www.arduino.cc/reference/en/language/functions/advanced-io/pulsein/
  int tmpIsDisconnected = 0;
  for(int i = 0; i < NUM_CHANNELS; i++)
  {
    channel[i] = pulseIn(chId[i], HIGH);

    /// Check whether the transmitter is connected, otherwise don't send
    if(channel[i] < PWM_DISCONNECT)
    {
      tmpIsDisconnected = 1;
    }
  }

  /// If not disconnected, bublish
  if (tmpIsDisconnected < 1)
  {
    rc_msg.data = channel[0];
    publisher_throttle.publish(&rc_msg);

    rc_msg.data = channel[1];
    publisher_steering.publish(&rc_msg);

  }
  
  nh.spinOnce();
}
