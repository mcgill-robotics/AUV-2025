#include <ros/ros.h>
#include "sensor.h"
#include "dvl.h"
#include "imu.h"
#include "depth.h"
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <iostream>

bool update_state_on_clock;
ros::Time last_clock_msg;
Sensor* z_estimators[1];
Sensor* quat_estimators[1];
ros::Publisher pub_pose;
ros::Publisher pub_x;
ros::Publisher pub_y;
ros::Publisher pub_z;
ros::Publisher pub_av;
ros::Publisher pub_imu_status;
ros::Publisher pub_dvl_status;
ros::Publisher pub_depth_status;
DepthSensor* depth;
Imu* imu;
ros::NodeHandle n;

void update_state(const ros::TimerEvent& event) {
    if(update_state_on_clock) {
        if(event.current_expected == last_clock_msg) {
            return;
        }
        last_clock_msg.sec = event.current_expected.sec;
        last_clock_msg.nsec = event.current_expected.nsec;
    }
    std_msgs::Bool depth_status_msg;
    depth_status_msg.data = depth->is_active();
    pub_depth_status.publish(depth_status_msg);

    std_msgs::Float64 z;
    std_msgs::Float64 x;
    std_msgs::Float64 y;
    geometry_msgs::Quaternion q_nwu_auv;
    bool found_x = false;
    bool found_y = false;
    bool found_z = false;
    bool found_quat = false;
    for(Sensor* sensor : z_estimators) {
        if(sensor->is_active()) {
            z.data = sensor->z;
            found_z = true;
            break;
        }
    }

    for(Sensor* sensor : quat_estimators) {
        if(sensor->is_active()) {
            q_nwu_auv.w = sensor->q_nwu_auv.w;
            q_nwu_auv.x = sensor->q_nwu_auv.x;
            q_nwu_auv.y = sensor->q_nwu_auv.y;
            q_nwu_auv.z = sensor->q_nwu_auv.z;
            found_quat = true;
            break;
        }
    }

    if(found_x && found_y && found_z && found_quat) {
        geometry_msgs::Pose pose;
        pose.position.x = x.data;
        pose.position.y = y.data;
        pose.position.z = z.data;
        pose.orientation = q_nwu_auv;
        pub_x.publish(x);
        pub_y.publish(y);
        pub_z.publish(z);
        pub_pose.publish(pose);
    }

}



void set_imu_params(IMU_PARAMS& params) {

    if(!n.getParam("q_imunominal_imu_w",params.q_imunominal_imu_w)) {
        ROS_ERROR("Failed to get param 'q_imunominal_imu_w'");
    }

    if(!n.getParam("q_imunominal_imu_x",params.q_imunominal_imu_x)) {
        ROS_ERROR("Failed to get param 'q_imunominal_imu_x'");
    }
    if(!n.getParam("q_imunominal_imu_y",params.q_imunominal_imu_y)) {
        ROS_ERROR("Failed to get param 'q_imunominal_imu_y'");
    }
    if(!n.getParam("q_imunominal_imu_z",params.q_imunominal_imu_z)) {
        ROS_ERROR("Failed to get param 'q_imunominal_imu_z'");
    }

}

int main(int argc, char **argv) {
    ros::init(argc,argv,"state_aggregator");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }


    depth = new DepthSensor(0.0,std::string("depth"));
    ros::Subscriber sub_depth = n.subscribe("/sensors/depth/z",100,&DepthSensor::depth_cb, depth);
    z_estimators[0] = depth;

    IMU_PARAMS q_imunominal_imu_s;
    set_imu_params(q_imunominal_imu_s);
    imu = new Imu(q_imunominal_imu_s,std::string("imu"));
    ros::Subscriber sub_quat = n.subscribe("/sensors/imu/quaternion",100,&Imu::quat_cb, imu);
    ros::Subscriber sub_ang_vel = n.subscribe("/sensors/imu/angular_velocity",100,&Imu::ang_vel_cb, imu);
    quat_estimators[0] = imu;

    pub_pose = n.advertise<geometry_msgs::Pose>("/state/pose",1);
    pub_x = n.advertise<std_msgs::Float64>("/state/x",1);
    pub_y = n.advertise<std_msgs::Float64>("/state/y",1);
    pub_z = n.advertise<std_msgs::Float64>("/state/z",1);
    pub_av = n.advertise<geometry_msgs::Vector3>("/state/angular_velocity",1);
    pub_imu_status = n.advertise<std_msgs::Bool>("/sensors/imu/status",1);
    pub_dvl_status = n.advertise<std_msgs::Bool>("/sensors/dvl/status",1);
    pub_depth_status = n.advertise<std_msgs::Bool>("/sensors/depth/status",1);


    if(!n.getParam("update_state_on_clock",update_state_on_clock)) {
        ROS_ERROR("Failed to get param 'update_state_on_clock'");
    }

    int update_rate;
    if(!n.getParam("update_rate",update_rate)) {
        ROS_ERROR("Failed to get param 'update_rate'");
    }
    double freq = 1.0/update_rate;
    ros::spinOnce();
    ros::Timer timer = n.createTimer(ros::Duration(freq), update_state);

    ros::spin();
    delete depth;
    return 0;
}