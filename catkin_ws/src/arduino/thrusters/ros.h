#ifndef _ROS_H_
#define _ROS_H_

#include "ArduinoHardware.h"
#include "ros/node_handle.h"

namespace ros {
typedef NodeHandle_<ArduinoHardware, 5, 5, 128, 128> NodeHandle;
}

#endif
