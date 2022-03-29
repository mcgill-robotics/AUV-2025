#include <ros.h>
#include <std_msgs/Float64.h>

#define DEPTH_PIN 5 
#define DELAY 1000 
ros::NodeHandle nh;

std_msgs::Float64 depthmsg;
ros::Publisher depth("depth", &depthmsg);

void setup() {
  pinMode(DEPTH_PIN, INPUT);
  nh.initNode();
  nh.advertise(depth);  
}

void loop() {
  depthmsg.data = analogRead(DEPTH_PIN);
  depth.publish(&depthmsg);
  nh.spinOnce();
  delay(DELAY);
}
