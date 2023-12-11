/*
  Author: Joaquin Palacios, Ava Chen
  Date:   2023-12-10
*/

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

// // Motor Shield Initiation:
// Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// // DC Motor:
// Adafruit_DCMotor * myDCMotor = AFMS.getMotor(1);

#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 IMU0; //address 0x4A  
BNO080 IMU1; //address 0x4B

const long BAUD_RATE = 9600;
const byte IMU_RATE = 20;

// ROS:
#define USE_ROSSERIAL

#ifdef USE_ROSSERIAL

#include <ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <myhand_sci_msgs/Motor_cmd.h>
#define CMD_TOPIC_NAME ("motor_cmds")
#define IMU_TOPIC_NAME ("IMUs")

ros::NodeHandle nh;

// For IMU publisher
geometry_msgs::QuaternionStamped imu_msg;
ros::Publisher pub_imu(IMU_TOPIC_NAME, &imu_msg);

// For command subscriber:
volatile int command;
void cmd_sub_callback(const myhand_sci_msgs::Motor_cmd & cmd_msg)
{
  command = cmd_msg.cmd;
}

ros::Subscriber<myhand_sci_msgs::Motor_cmd> sub_cmd(CMD_TOPIC_NAME, cmd_sub_callback);

#endif

void setup() {
  // //Motor Setup:
  // myDCMotor->setSpeed(0);
  // myDCMotor->run(RELEASE);

  // IMU Setup
  Wire.begin();
  Wire.setClock(400000); 

  IMU0.begin(0x4A);
  IMU1.begin(0x4B);

  IMU0.enableGameRotationVector(IMU_RATE); 
  IMU1.enableGameRotationVector(IMU_RATE); 

  command = 0;

#ifdef USE_ROSSERIAL
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.advertise(pub_imu);
#else
  Serial.begin(9600);
#endif
  delay(100);
}

void loop() {
  // // Run DC Motor Forwards
  // myDCMotor->run(FORWARD);
  // myDCMotor->setSpeed(command);

  if (IMU0.dataAvailable() == true)
  {
    imu_msg.header.frame_id = "0";
    imu_msg.header.stamp = nh.now();
    imu_msg.quaternion.x = IMU0.getQuatI();
    imu_msg.quaternion.y = IMU0.getQuatJ();
    imu_msg.quaternion.z = IMU0.getQuatK();
    imu_msg.quaternion.w = IMU0.getQuatReal();
    pub_imu.publish(&imu_msg);
    delay(1);
    // nh.spinOnce();
  }
  if (IMU1.dataAvailable() == true)
  {
    imu_msg.header.frame_id = "1";
    imu_msg.header.stamp = nh.now();
    imu_msg.quaternion.x = IMU1.getQuatI();
    imu_msg.quaternion.y = IMU1.getQuatJ();
    imu_msg.quaternion.z = IMU1.getQuatK();
    imu_msg.quaternion.w = IMU1.getQuatReal();
    pub_imu.publish(&imu_msg);
    delay(1);
    // nh.spinOnce();
  }
  nh.spinOnce();
}
