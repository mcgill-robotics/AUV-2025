#ifndef DVL
#define DVL

#include "sensor.h"
#include "imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <auv_msgs/DeadReckonReport.h>

class Dvl : Sensor {
    public:
        Dvl(tf2::Quaternion q_auv_dvl, tf2::Vector3 pos_auv_dvl);
    private:
        void dr_cb(auv_msgs::DeadReckonReport msg);
        tf2::Vector3 pos_auv_dvl;
        Imu imu;
        tf2::Quaternion q_dvlref_nwu;
};

#endif