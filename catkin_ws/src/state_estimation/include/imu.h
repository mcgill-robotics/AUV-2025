#include "sensor.h"
#include <tf2/LinearMath/Quaternion.h>
#include <sbg_driver/SbgEkfQuat.h>

class Imu : Sensor {
    public:
        Imu(tf2::Quaternion q_auv_imu)
    private:
        void ang_vel_cb(void);
        void quat_cb(sbg_driver::SbgEkfQuat msg);
        tf2::Quaternion q_imunominal_imu;
};