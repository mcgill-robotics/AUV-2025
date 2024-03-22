#ifndef DVL
#define DVL

#include "sensor.h"
#include "imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <auv_msgs/DeadReckonReport.h>

struct DVL_PARAMS {
    double q_dvlnominal_dvl_w;
	double q_dvlnominal_dvl_x;
	double q_dvlnominal_dvl_y;
	double q_dvlnominal_dvl_z;
    double pos_auv_dvl_x;
    double pos_auv_dvl_y;
    double pos_auv_dvl_z;
};

class Dvl : public Sensor {
    public:
        Dvl(DVL_PARAMS params, std::string name, bool u_o_cl, const Imu& _imu);
        void dr_cb(const auv_msgs::DeadReckonReport::ConstPtr& msg);
    private:
        tf2::Vector3 pos_auv_dvl;
        tf2::Quaternion q_dvl_auv;
        const Imu& imu;
        tf2::Quaternion q_nwu_dvlref;
        bool valid_q_nwu_dvlref;
        virtual bool is_active(void) override;
};

#endif