#ifndef DEPTH
#define DEPTH

#include "sensor.h"
#include <std_msgs/Float64.h>
class DepthSensor : public Sensor {
    public:
        DepthSensor(double pos_auv_depth, std::string name, bool update_on_clock);
        void depth_cb(const std_msgs::Float64::ConstPtr& msg);
    private:
        double prev_z;
        bool has_different_data();
        void set_prev_state(void) override;
        void update_last_state(void) override;
};

#endif