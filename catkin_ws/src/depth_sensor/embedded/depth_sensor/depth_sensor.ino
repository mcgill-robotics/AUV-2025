#include <ros.h>
#include "MS5803.h"
#include <std_msgs/Float64.h>
#define DELAY 10
#define RHO 1000
#define G_VALUE 9.81

MS5803 pressureSensor(0x76);
int32_t basePressure, currentPressure;
byte sensor_address1;
ros::NodeHandle nh;
std_msgs::Float64 depthmsg;
ros::Publisher depth("depth", &depthmsg);

float getDepth() {
  currentPressure = pressureSensor.getPressure();
  return ((currentPressure-basePressure) / (G_VALUE*RHO));
}

void setup() {  
  nh.initNode();
  nh.advertise(depth);
  pressureSensor.readCoefficients();
  basePressure = pressureSensor.getPressure();
}

void loop() {

  depthmsg.data = (-1)*getDepth();
  depth.publish(&depthmsg);
  nh.spinOnce();
  delay(DELAY);
}
