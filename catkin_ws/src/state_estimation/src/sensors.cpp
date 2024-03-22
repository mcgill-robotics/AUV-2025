#include "sensor.h"
#include "ros/ros.h"
#include "dvl.h"
#include "imu.h"
#include "depth.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <auv_msgs/DeadReckonReport.h>
#include <sbg_driver/SbgEkfQuat.h>
#include <sbg_driver/SbgImuData.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <ros/console.h>

tf2::Quaternion Q_NWU_NED(1,0,0,0);
tf2::Quaternion Q_DVLNOMINAL_AUV(1,0,0,0);

Sensor::~Sensor() {
    delete tfListener;
}

Sensor::Sensor(std::string name, bool u_o_c) {
    sensor_name = name;
    last_unique_state_time = ros::Time(0,0);
    last_error_message_time = ros::Time(0,0);
    time_before_considered_inactive = ros::Duration(1.0);
    update_on_clock = u_o_c;
    last_clock_msg = ros::Time::now();
    tfListener = new tf2_ros::TransformListener(tfBuffer);
}

void Sensor::update_last_state() {
    if(ros::Time::now() - last_unique_state_time > time_before_considered_inactive) {
            ROS_INFO("%s has become active",sensor_name.c_str());
        }
    last_unique_state_time = ros::Time::now();
}

bool Sensor::is_active() {
    if(ros::Time::now() == ros::Time(0,0)) return false; //this equality might be sus but idk
    ros::Time now = ros::Time::now();
    if ((now - last_unique_state_time) > time_before_considered_inactive) {
        if(now - last_error_message_time > ros::Duration(1.0)) {
            last_error_message_time = ros::Time::now();
            ROS_WARN("%s has been inactive for %s seconds",sensor_name.c_str(),std::to_string(time_before_considered_inactive.sec).c_str());
        } 
        return false;
    }
    return true;
}

Dvl::Dvl(DVL_PARAMS params, std::string name, bool u_o_c, const Imu& _imu) : Sensor(name, u_o_c) {
    tf2::Quaternion q_dvlnominal_dvl(params.q_dvlnominal_dvl_x, params.q_dvlnominal_dvl_y, params.q_dvlnominal_dvl_z, params.q_dvlnominal_dvl_w);
    q_dvl_auv = q_dvlnominal_dvl * Q_DVLNOMINAL_AUV;
    pos_auv_dvl = tf2::Vector3(params.pos_auv_dvl_x, params.pos_auv_dvl_y, params.pos_auv_dvl_z);
    imu = _imu;
    valid_q_nwu_dvlref = false;
}

void Dvl::dr_cb(const auv_msgs::DeadReckonReport::ConstPtr& msg) {
    if(update_on_clock && last_clock_msg == ros::Time::now()) return;
    last_clock_msg = ros::Time::now();

    tf2::Quaternion q_dvlref_dvl;
    q_dvlref_dvl.setEuler(msg->yaw,msg->pitch,msg->roll);
    tf2::Vector3 pos_dvlref_dvl(msg->position.x,msg->position.y,msg->position.z);
    tf2::Quaternion q_dvlref_auv = q_dvlref_dvl * q_dvl_auv;

    if(imu->is_active()) {
        // this should be made into a moving average or something at some point.
        // not even gonna bother trying until a pool test is possible
        q_nwu_dvlref = imu->q_nwu_auv * q_dvlref_auv.conjugate();
        valid_q_nwu_dvlref = true;
    }

    if(valid_q_nwu_dvlref) {
        tf2::Quaterion quat_pos_dvlref_dvl(msg->position.x,msg->position.y,msg->position.z,0);
        tf2::Quaternion quat_post_nwu_dvl = q_nwu_dvlref * quat_pos_dvlref_dvl * q_nwu_dvlref.conjugate();
        pos_nwu_auv.x = quat_post_nwu_dvl.getX();
        pos_nwu_auv.y = quat_post_nwu_dvl.getY();
        pos_nwu_auv.z = quat_post_nwu_dvl.getZ();       
    }

    update_last_state();
}

bool Dvl::is_active() {
    return valid_q_nwu_dvlref && Sensor::is_active();
}

Imu::Imu(IMU_PARAMS params, std::string name, bool u_o_c) : Sensor(name,u_o_c){
    seen_ang_vel = false;
    seen_quat = false;
    tf2::Quaternion q_imunominal_imu(q_imunominal_imu_w, q_imunominal_imu_x, q_imunominal_imu_y, q_imunominal_imu_z);
    tf2::Quaternion q_imunominal_auv(1,0,0,0);
    q_imu_auv = q_imunominal_imu.conjugate() * q_imunominal_auv;

}

void Imu::quat_cb(const sbg_driver::SbgEkfQuat::ConstPtr& msg) {
    if(update_on_clock && last_clock_msg == ros::Time::now()) return;
    last_clock_msg = ros::Time::now();

    tf2::Quaternion q_ned_imu(msg->quaternion.x,msg->quaternion.y,msg->quaternion.z,msg->quaternion.w);
    q_nwu_auv = Q_NWU_NED * q_ned_imu * q_imu_auv;
    seen_quat = true;

    update_last_state();
}

bool Imu::is_active() {
    return Sensor::is_active() && seen_ang_vel && seen_quat;
}

void Imu::ang_vel_cb(const sbg_driver::SbgImuData::ConstPtr& msg) {
    tf2::Quaternion quat_ang_vel_imu(msg->gyro.x, msg->gyro.y, msg->gyro.z,0);
    tf2::Quaternion quat_ang_vel_auv = q_imu_auv * quat_ang_vel_imu * q_imu_auv.conjugate();
    ang_vel_auv.x = quat_ang_vel_auv.getX();
    ang_vel_auv.y = quat_ang_vel_auv.getY();
    ang_vel_auv.z = quat_ang_vel_auv.getZ();

    
    seen_ang_vel = true;
    update_last_state();
}


DepthSensor::DepthSensor(double pos_auv_depth, std::string name, bool u_o_c, const Imu& _imu) : Sensor(name,u_o_c) {
    prev_z = 0;
    z = 0;
    pos_auv_depth = pos_auv_depth;
    imu = _imu;
    pos_auv_depth_nwu = pos_auv_depth;
}

void DepthSensor::depth_cb(const std_msgs::Float64::ConstPtr& msg) {
    if(update_on_clock && last_clock_msg == ros::Time::now()) return;
    last_clock_msg = ros::Time::now();

    // tf2::Quaternion quat_pos_auv_depth(0,0,pos_auv_depth,0);
    // if(imu->is_active()) {
    //     tf2::Quaternion offset = imu->q_nwu_auv * quat_pos_auv_depth * imu->q_nwu_auv.conjugate();
    //     pos_auv_depth_nwu = offset.getZ();
        
    // }
    // z = msg->data - pos_auv_depth_nwu.getZ();
    z = msg->data;
    update_last_state();

}

bool DepthSensor::has_different_data() {
    return prev_z != z;
}

void DepthSensor::set_prev_state() {
    prev_z = z;
}

void DepthSensor::update_last_state() {
    if(update_on_clock || has_different_data()) Sensor::update_last_state();
    set_prev_state();
}
