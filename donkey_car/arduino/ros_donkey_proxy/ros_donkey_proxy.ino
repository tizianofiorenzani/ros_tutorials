
/*
 * Proxy to run on arduino board
 * > reads RC input and publishes them to pics as UInt16
 * > subscribes to commands and controls the servos
 * 
 * @author: Tiziano Fiorenzani
 */


/// Define the arduino
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

/// Include
#include <ros.h>
#include <Servo.h> 
#include <std_msgs/UInt16.h>

/// Create the node handle object
ros::NodeHandle  nh;

/// Define input channel pins
static const int CH_THROTTLE_IN  = 9; 
static const int CH_STEERING_IN  = 10;
static const int CH_THROTTLE_OUT = 11;
static const int CH_STEERING_OUT = 12;
static const int PWM_DISCONNECT  = 700;

Servo servo_throttle;
Servo servo_steering;
int channel[2];

/// Define the message and the publishers
std_msgs::UInt16 rc_msg;
ros::Publisher publisher_throttle("rc_throttle", &rc_msg);
ros::Publisher publisher_steering("rc_steering", &rc_msg);

void cb_servo_throttle( const std_msgs::UInt16& cmd_msg){
  servo_throttle.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

void cb_servo_steering( const std_msgs::UInt16& cmd_msg){
  servo_steering.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
ros::Subscriber<std_msgs::UInt16> subscriber_throttle("servo_throttle", cb_servo_throttle);
ros::Subscriber<std_msgs::UInt16> subscriber_steering("servo_steering", cb_servo_steering);

 
void setup() {
  /// Initialize the node
  nh.initNode();

  /// Define pins as INPUT
  pinMode(CH_THROTTLE_IN, INPUT);
  pinMode(CH_STEERING_IN, INPUT);

  /// Define the output pins
  servo_throttle.attach(CH_THROTTLE_OUT); 
  servo_steering.attach(CH_STEERING_OUT); 

  /// create the publisher
  nh.advertise(publisher_throttle);
  nh.advertise(publisher_steering);

  /// Subscribe to the topic
  nh.subscribe(subscriber_throttle);
  nh.subscribe(subscriber_steering);

  pinMode(LED_BUILTIN, OUTPUT);
}
 
void loop() {
  /// Use the function pulseIn and returns the timinng in us between two highs
  /// see: https://www.arduino.cc/reference/en/language/functions/advanced-io/pulsein/
  int tmpIsDisconnected = 0;

  channel[0] = pulseIn(CH_THROTTLE_IN, HIGH);
  channel[1] = pulseIn(CH_STEERING_IN, HIGH);

  /// Read the pulses and verify it is connected
  for(int i = 0; i < 2; i++)
  {
    /// Check whether the transmitter is connected, otherwise don't send
    if(channel[i] < PWM_DISCONNECT)
    {
      tmpIsDisconnected = 1;
    }
  }

  /// If RC is connected, publish
  if (tmpIsDisconnected < 1)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    
    rc_msg.data = channel[0];
    publisher_throttle.publish(&rc_msg);

    rc_msg.data = channel[1];
    publisher_steering.publish(&rc_msg);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  nh.spinOnce();
}
