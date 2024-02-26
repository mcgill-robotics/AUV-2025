#include "state_estimation/dvl.h"
#include "state_estimation/imu.h"
#include "state_estimation/depth.h"
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
    ros::init(argc,argv,"state_aggregator");

    ros::NodeHandle n;
    return 0;
}