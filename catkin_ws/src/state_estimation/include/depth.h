#include "sensor.h"
#include <std_msgs/Float64.h>

class DepthSensor : Sensor {
    public:
        DepthSensor(double pos_auv_depth)
    private:
        void depth_cb(std_msgs::Float64 msg);
};