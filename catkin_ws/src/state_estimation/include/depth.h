#ifndef DEPTH
#define DEPTH

#include "sensor.h"
#include <std_msgs/Float64.h>
class DepthSensor : public Sensor {
    public:
        DepthSensor(double pos_auv_depth, ros::NodeHandle& n, std::string name);
    private:
        void depth_cb(std_msgs::Float64 msg);
        bool has_valid_data() override;
        bool has_different_data() override;
        void set_prev_state(void) override;
};

#endif