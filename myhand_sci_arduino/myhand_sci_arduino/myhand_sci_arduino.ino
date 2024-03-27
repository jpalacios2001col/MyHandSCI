/*
  Author: Joaquin Palacios, Ava Chen
  Date:   2023-12-10
  Last Edit: 2024-01-15
*/

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Encoder.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Int32.h>
#include <myhand_sci_msgs/Motor_cmd.h>
#include <std_msgs/Int64.h>  // Added include for the Int64 message type

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myDCMotor = AFMS.getMotor(1);
Encoder myEnc(2, 3);

long int enc_lb = 0;
long int enc_ub = 14000;
long int enc_counts = 0;

#define EMERGENCY_PIN 14
int emergencyState = 1;
int emergency_speed = 150;

BNO080 IMU0; //address 0x4A  
BNO080 IMU1; //address 0x4B

const long BAUD_RATE = 57600;
const byte IMU_RATE = 20;

#define USE_ROSSERIAL

#ifdef USE_ROSSERIAL
ros::NodeHandle nh;
geometry_msgs::QuaternionStamped imu_msg;
ros::Publisher pub_imu("IMUs", &imu_msg);

std_msgs::Int32 encoder_msg;
ros::Publisher pub_encoder("encoder_counts", &encoder_msg);

int command;
volatile bool update_needed = true;

void cmd_sub_callback(const myhand_sci_msgs::Motor_cmd &cmd_msg)
{
  command = cmd_msg.cmd;
}

void motor_upper_bound_callback(const std_msgs::Int64 &msg)
{
  enc_ub = msg.data;
}


ros::Subscriber<std_msgs::Int64> sub_motor_upper_bound("motor_upper_bound", motor_upper_bound_callback);  // Added subscriber for motor_upper_bound
ros::Subscriber<myhand_sci_msgs::Motor_cmd> sub_cmd("motor_cmds", cmd_sub_callback);

#endif

void setup()
{
#ifdef USE_ROSSERIAL
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.subscribe(sub_motor_upper_bound);  // Subscribe to motor_upper_bound topic
  nh.advertise(pub_imu);
  nh.advertise(pub_encoder);  // Advertise the encoder publisher
  delay(30);
#else
  Serial.begin(9600);
#endif

  pinMode(EMERGENCY_PIN, INPUT_PULLUP);

  AFMS.begin(1000);
  myDCMotor->setSpeed(0);
  myDCMotor->run(FORWARD);
  myDCMotor->run(RELEASE);

  Wire.begin();

  IMU0.begin(0x4A);
  IMU1.begin(0x4B);

  IMU0.enableGameRotationVector(IMU_RATE);
  IMU1.enableGameRotationVector(IMU_RATE);

  command = 0;
}

void loop()
{
  // Read encoder:
  enc_counts = myEnc.read();

  // Check emergency button:
  emergencyState = digitalRead(EMERGENCY_PIN);
  if (!emergencyState)
  {
    myDCMotor->run(BACKWARD);
    while (myEnc.read() > enc_lb)
    {
      myDCMotor->setSpeed(emergency_speed);
    }
    myDCMotor->setSpeed(0);
    while (1);
  }
  else 
  {
    if (command < 0 && enc_counts > enc_lb)
    {
      myDCMotor->run(BACKWARD);
      myDCMotor->setSpeed(abs(command));
    }
    else if (command > 0 && enc_counts < enc_ub)
    {
      myDCMotor->run(FORWARD);
      myDCMotor->setSpeed(abs(command));
    }
    else
    {
      myDCMotor->setSpeed(0);
    }
  }

  // Read IMUs:
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
    nh.spinOnce();
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
    nh.spinOnce();
  }

  // Publish encoder counts:
  encoder_msg.data = enc_counts;
  pub_encoder.publish(&encoder_msg);

  nh.spinOnce();
}


// //Initialize Timer5 for sampling frequency 500Hz
  // TIMSK5 |= (1 << TOIE5);    //enable timer overflow interrupt for Timer5
  // TCNT5 = 33535;            //set counter to 45535, 32000 clicks will be 2 ms
  // TCCR5B |= (1 << CS50);     //start timer5 with prescaler=1
  

// if(update_needed){
  //   myDCMotor->setSpeed(command);
  //   delay(10);
  //   update_needed = false;
  // }

// ISR(TIMER5_OVF_vect)
// {
//   // Reset the timer5 count for 2ms - 500Hz sampling rate
//   TCNT5 = 33535;
//   update_needed = true;
// }
