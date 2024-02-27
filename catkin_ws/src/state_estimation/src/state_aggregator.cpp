#include <ros/ros.h>
#include "sensor.h"
#include "dvl.h"
#include "imu.h"
#include "depth.h"
#include <geometry_msgs/Pose.h>
#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc,argv,"state_aggregator");
    ros::NodeHandle n;

    DepthSensor depth(0.0,n,std::string("depth"));
    return 0;
}