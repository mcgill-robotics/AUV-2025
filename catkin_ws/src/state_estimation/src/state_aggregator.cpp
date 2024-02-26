#include <ros/ros.h>
#include "dvl.h"
#include "imu.h"
#include "depth.h"
#include <geometry_msgs/Pose.h>
#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc,argv,"state_aggregator");
    return 0;
}