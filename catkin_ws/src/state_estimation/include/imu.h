#ifndef IMU
#define IMU

#include "sensor.h"
#include <tf2/LinearMath/Quaternion.h>
#include <sbg_driver/SbgEkfQuat.h>
#include <sbg_driver/SbgImuData.h>

struct IMU_PARAMS {
    double q_imunominal_imu_w;
	double q_imunominal_imu_x;
	double q_imunominal_imu_y;
	double q_imunominal_imu_z;
};

class Imu : public Sensor {
    public:
        Imu(IMU_PARAMS q_imunominal_imu_s,std::string name,bool update_on_clock);
        bool is_active(void) override;
        void ang_vel_cb(const sbg_driver::SbgImuData::ConstPtr& msg);
        void quat_cb(const sbg_driver::SbgEkfQuat::ConstPtr& msg);
    private:
        geometry_msgs::Quaternion prev_q_nwu_auv;
        bool seen_quat;
        bool seen_ang_vel;
        tf2::Quaternion q_imunominal_imu;
        void set_prev_state(void) override;
};

#endif