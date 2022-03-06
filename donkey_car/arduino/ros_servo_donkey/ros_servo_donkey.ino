/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo1;
Servo servo2;

void servo_cb1( const std_msgs::UInt16& cmd_msg){
  servo1.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

void servo_cb2( const std_msgs::UInt16& cmd_msg){
  servo2.write(cmd_msg.data); //set servo angle, should be from 0-180  
}


ros::Subscriber<std_msgs::UInt16> sub1("servo1", servo_cb1);
ros::Subscriber<std_msgs::UInt16> sub2("servo2", servo_cb2);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  
  servo1.attach(9);  //attach it to pin 9
  servo2.attach(10); //attach it to pin 10
}

void loop(){
  nh.spinOnce();
  delay(1);
}
