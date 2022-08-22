#include <ros.h>
#include <Wire.h>
#include <std_msgs/Float64.h>
#include "MS5803.h"

//#define USE_USBCON
#define DELAY 1000 
#define RHO 1000
#define G_VALUE 9.81

MS5803 pressureSensor();
float basePressure, currentPressure;
//ros::NodeHandle nh;

//std_msgs::Float64 depthmsg;
//ros::Publisher depth("depth", &depthmsg);

void sensorInit() {
  //Wire.begin();
  //pressureSensor.reset();
}

float getDepth() {
  currentPressure = pressureSensor.getPressure();
  return currentPressure;
  //return((currentPressure-basePressure) / (G_VALUE*RHO));
}

void setup() {
  //sensorInit();
  //basePressure = pressureSensor.getPressure(ADC_4096);
  //nh.getHardware()->setBaud(57600);
  //nh.initNode();
  //nh.advertise(depth);  
  Serial.begin(9600);
}

void loop() {
  //depthmsg.data = getDepth();
  //depth.publish(&depthmsg);
  //nh.spinOnce();
  //delay(DELAY);
  float pressure = pressureSensor.getPressure();
  Serial.print("Pressure is ");
  Serial.println(pressure);
  delay(1000);
}
