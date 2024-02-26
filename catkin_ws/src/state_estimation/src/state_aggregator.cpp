#include "dvl.h"
#include "imu.h"
#include "depth.h"
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
    ros::init(argc,argv,"state_aggregator");

    ros::NodeHandle n;
    return 0;
}