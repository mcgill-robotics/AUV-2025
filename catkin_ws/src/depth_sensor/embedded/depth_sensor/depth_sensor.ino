#define USE_USBCON
#include <ros.h>
#include "MS5803.h"
#include "std_msgs/Float64.h"
#define DELAY 1000 
#define RHO 1000
#define G_VALUE 9.81


MS5803 pressureSensor(0x76);
int32_t basePressure, currentPressure;
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
  pressureSensor.sensorInit(); 
  basePressure = pressureSensor.getPressure();
  //Serial.begin(9600);
}

void loop() {
  float pressure = pressureSensor.getPressure();
  //Serial.print("Pressure is: ");
  //Serial.println(pressure);
  depthmsg.data = getDepth();
  depth.publish(&depthmsg);
  nh.spinOnce();
  //pressureSensor.readCoefficients();
  delay(DELAY);
}
