/*
  Author: Joaquin Palacios
  Date:   2023-11-28
*/

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
//#include <Encoder.h>

// Motor Shield Initiation:
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// DC Motor:
Adafruit_DCMotor * myDCMotor = AFMS.getMotor(1);

// Encoder Pins
//#define ENCODER1 2
//#define ENCODER2 3

// Encoder Initiation:
//Encoder myEnc(ENCODER1, ENCODER2);

// For command
volatile int command;

// ROS:
#define USE_ROSSERIAL

#ifdef USE_ROSSERIAL

#include <ros.h>
#include <myhand_sci_msgs/Motor_cmd.h>
#include <std_msgs/UInt32.h>
#define CMD_TOPIC_NAME ("motor_cmds")

ros::NodeHandle nh;
std_msgs::UInt32 encoder_msg;

ros::Publisher pub_enc("encoder", &encoder_msg);

void cmd_sub_callback(const myhand_sci_msgs::Motor_cmd & cmd_msg)
{
  command = cmd_msg.cmd;
}

ros::Subscriber<myhand_sci_msgs::Motor_cmd> cmd_sub(CMD_TOPIC_NAME, cmd_sub_callback);

#endif

void setup() {
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myDCMotor->setSpeed(0);

  // Enable DC Motor
  myDCMotor->run(RELEASE);

  // Intitialize command:
  command = 0;

#ifdef USE_ROSSERIAL
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(pub_enc);
#else
  Serial.begin(9600);
#endif
  delay(100);
}

void loop() {
  // Run DC Motor Forwards
  myDCMotor->run(FORWARD);
  myDCMotor->setSpeed(command);
  encoder_msg.data = 10;
  
  #ifdef USE_ROSSERIAL
      pub_enc.publish( &encoder_msg );
      nh.spinOnce();
  #else
    Serial.println();
  #endif
  
}
