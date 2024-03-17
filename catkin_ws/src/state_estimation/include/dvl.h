#ifndef DVL
#define DVL

#include "sensor.h"
#include "imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <auv_msgs/DeadReckonReport.h>

class Dvl : public Sensor {
    public:
        Dvl(std::string name, bool u_o_cl);
        void dr_cb(const auv_msgs::DeadReckonReport::ConstPtr& msg);
    private:
        tf2::Vector3 pos_auv_dvl;
        tf2::Quaternion q_dvlref_nwu;
        void set_prev_state(void) override;
        double prev_x;
        double prev_y;
        double prev_z;
        geometry_msgs::Quaternion prev_q_nwu_auv;
};

#endif