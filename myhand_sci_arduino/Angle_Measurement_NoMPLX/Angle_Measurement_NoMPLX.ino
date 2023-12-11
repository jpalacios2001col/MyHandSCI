/*
  By: Joaquin Palacios
*/

#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU1; //address 0x4A  
BNO080 myIMU2; //address 0x4B

const long BAUD_RATE = 9600;
const byte IMU_RATE = 20;

float x0, y0, z0, w0, x1, y1, z1, w1;

void setup()
{
  Serial.begin(BAUD_RATE);
  Serial.println();
  delay(1000);
  Serial.println("IMU measurements:");

  Wire.begin();
  Wire.setClock(400000); 

  if (myIMU1.begin(0x4A) == false)
  {
    Serial.println("IMU1 is not connected");
    while(1);
  }

  if (myIMU2.begin(0x4B) == false)
  {
    Serial.println("IMU1 is not connected");
    while(1);
  }

  myIMU1.enableGameRotationVector(IMU_RATE); 
  myIMU2.enableGameRotationVector(IMU_RATE); 

}

void loop()
{

  //Look for reports from the IMU
  if (myIMU1.dataAvailable() == true)
  {
    x0 = myIMU1.getQuatI();
    y0 = myIMU1.getQuatJ();
    z0 = myIMU1.getQuatK();
    w0 = myIMU1.getQuatReal();

    // //Serial.print("First:");
    // Serial.print(x1, 2);
    // Serial.print(F(","));
    // Serial.print(y1, 2);
    // Serial.print(F(","));
    // Serial.print(z1, 2);
    // Serial.print(F(","));
    // Serial.print(w1, 2);
    // Serial.print(F(","));

    // Serial.println();
  }

  if (myIMU2.dataAvailable() == true)
  {
    x1 = myIMU2.getQuatI();
    y1 = myIMU2.getQuatJ();
    z1 = myIMU2.getQuatK();
    w1 = myIMU2.getQuatReal();

    // //Serial.print("First:");
    // Serial.print(x0, 2);
    // Serial.print(F(","));
    // Serial.print(y0, 2);
    // Serial.print(F(","));
    // Serial.print(z0, 2);
    // Serial.print(F(","));
    // Serial.print(w0, 2);
    // Serial.print(F(","));

    // Serial.println();
  }

  // Serial.print("Y vector: ");

  // float local_y[3];
  // local_y[0] = 2*(y0*z0 - x0*w0);
  // local_y[1] = 2*(x0*x0 + z0*z0) - 1;
  // local_y[2] = 2*(z0*w0 + x0*y0);
  
  // Serial.print(local_y[0], 2);
  // Serial.print(F(","));
  // Serial.print(local_y[1], 2);
  // Serial.print(F(","));
  // Serial.print(local_y[2], 2);


  // Calculate the quaternion representing the rotation from the first to the second
  float qx, qy, qz, qw;
  qx = x1 * w0 - y1 * z0 + z1 * y0 + w1 * x0;
  qy = x1 * z0 + y1 * w0 - z1 * x0 + w1 * y0;
  qz = -x1 * y0 + y1 * x0 + z1 * w0 + w1 * z0;
  qw = -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0;

  // //Serial.print("First:");
  // Serial.print(qx, 2);
  // Serial.print(F(","));
  // Serial.print(qy, 2);
  // Serial.print(F(","));
  // Serial.print(qz, 2);
  // Serial.print(F(","));
  // Serial.println(qw, 2);
  
  // Serial.println();

  // // Calculate the angle from the quaternion
  float pitch = asin(-2.0*(qx*qz - qw*qy));
  Serial.print("Pitch: ");
  Serial.print(degrees(pitch));

  float angle = atan2(sqrt(qx * qx + qy * qy + qz * qz), qw);
  Serial.print("Axis Angle: ");
  if (pitch > 0) {
    Serial.println(degrees(angle), 2);
  } 
  if (pitch < 0) {
    Serial.println(-degrees(angle), 2);
  }


  Serial.println();

  delay(10);
}
