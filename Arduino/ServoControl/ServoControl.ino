/*
 * rosserial Servo Control Example
 *
 * This sketch  control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo;

int s1_pin = 9;

void servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  int s1_angle = cmd_msg.data[0];
  servo.write(s1_angle); //set servo angle, should be from 0-180 

}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("/arduino_command", servo_cb);

void setup(){
  pinMode(s1_pin, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(s1_pin); //attach to servo 1
}

void loop(){
  nh.spinOnce();
  delay(1);
}
