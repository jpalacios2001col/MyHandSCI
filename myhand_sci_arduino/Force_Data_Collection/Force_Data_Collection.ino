/*
  Futek Publisher
  50lb sensor
  Reads an analog input on pin 0, converts it to force, and publishes result.
  
 */
#define ROS

#ifdef ROS
  #include <ros.h>
  #include <ros/time.h>
  #include <myhand_sci_msgs/Futek.h>

  ros::NodeHandle nh;
  myhand_sci_msgs::Futek futek_msg;
  ros::Publisher pub("futek", &futek_msg);
#endif 

// the setup routine runs once when you press reset:
void setup() {
  pinMode(A0, INPUT);
  #ifdef ROS
    nh.getHardware()->setBaud(19200);
    nh.initNode();
    nh.advertise(pub);
  #else
    // initialize serial communication at 9600 bits per second:
    Serial.begin(19200);
  #endif
}

// the loop routine runs over and over again forever:
void loop() {
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a force reading
  // sensorValue * (sensorMax / ADCmax) * sensor_mv/V * calibration_scalar + calibration_intercept
  float force = sensorValue * 0.17; //50lb sensor into newtons
  //float force = sensorValue;

  #ifdef ROS
    futek_msg.load = force;
    futek_msg.header.stamp = nh.now();
    pub.publish(&futek_msg);
    nh.spinOnce();
    delay(10);
  #else
    Serial.println(force);
  #endif
}