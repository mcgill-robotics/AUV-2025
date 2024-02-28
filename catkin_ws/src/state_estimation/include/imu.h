#ifndef IMU
#define IMU

#include "sensor.h"
#include <tf2/LinearMath/Quaternion.h>
#include <sbg_driver/SbgEkfQuat.h>

class Imu : Sensor {
    public:
        Imu(tf2::Quaternion q_auv_imu);
    private:
        tf2::Quaternion prev_q_nwu_auv;
        void ang_vel_cb(void);
        void quat_cb(sbg_driver::SbgEkfQuat msg);
        tf2::Quaternion q_imunominal_imu;
        bool has_valid_data() override;
        bool has_different_data() override;
        void set_prev_state(void) override;
};

struct IMU_PARAMS {
    double q_imunominal_imu_w;
	double q_imunominal_imu_x;
	double q_imunominal_imu_y;
	double q_imunominal_imu_z;
}

#endif