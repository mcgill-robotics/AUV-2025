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
    set_prev_state();
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

Dvl::Dvl(std::string name, bool u_o_c) : Sensor(name, u_o_c) {
}

void Dvl::dr_cb(const auv_msgs::DeadReckonReport::ConstPtr& msg) {
    if(update_on_clock && last_clock_msg == ros::Time::now()) return;
    tf2::Quaternion q_dvlref_dvl;
    q_dvlref_dvl.setEuler(msg->yaw,msg->pitch,msg->roll);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "DVL";
    transformStamped.child_frame_id = "DVLREF";
    transformStamped.transform.translation.x = -msg->x;
    transformStamped.transform.translation.y = -msg->y;
    transformStamped.transform.translation.z = -msg->z;
    transformStamped.transform.rotation.w = q_dvlref_dvl.getW();
    transformStamped.transform.rotation.x = -q_dvlref_dvl.getX();
    transformStamped.transform.rotation.y = -q_dvlref_dvl.getY();
    transformStamped.transform.rotation.z = -q_dvlref_dvl.getZ();

    br.sendTransform(transformStamped);

    geometry_msgs::TransformStamped t_nwu_auv;
    try{
        t_nwu_auv = tfBuffer.lookupTransform("DVLREF", "NWU",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }
    

    ROS_DEBUG("x: %lf, y: %lf, z: %lf\n",x,y,z);

    update_last_state();

}


void Dvl::set_prev_state() {
    prev_q_nwu_auv = q_nwu_auv;
}

Imu::Imu(IMU_PARAMS params, std::string name, bool u_o_c) : Sensor(name,u_o_c){
    seen_ang_vel = false;
    seen_quat = false; 
}

void Imu::quat_cb(const sbg_driver::SbgEkfQuat::ConstPtr& msg) {
    if(update_on_clock && last_clock_msg == ros::Time::now()) return;
    last_clock_msg = ros::Time::now();
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "IMU";
    transformStamped.child_frame_id = "NED";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;

    transformStamped.transform.rotation.x = -msg->quaternion.x;
    transformStamped.transform.rotation.y = -msg->quaternion.y;
    transformStamped.transform.rotation.z = -msg->quaternion.z;
    transformStamped.transform.rotation.w = msg->quaternion.w;

    br.sendTransform(transformStamped);

    geometry_msgs::TransformStamped q_nwu_auv_ts;
    try{
        q_nwu_auv_ts = tfBuffer.lookupTransform("NWU", "AUV",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }
    q_nwu_auv.w = q_nwu_auv_ts.transform.rotation.w;
    q_nwu_auv.x = q_nwu_auv_ts.transform.rotation.x;
    q_nwu_auv.y = q_nwu_auv_ts.transform.rotation.y;
    q_nwu_auv.z = q_nwu_auv_ts.transform.rotation.z;

    seen_quat = true;

    update_last_state();
}

bool Imu::is_active() {
    return Sensor::is_active() && seen_ang_vel && seen_quat;
}

void Imu::ang_vel_cb(const sbg_driver::SbgImuData::ConstPtr& msg) {
    geometry_msgs::Vector3Stamped ang_vel_imu;
    ang_vel_imu.vector.x = msg->gyro.x;
    ang_vel_imu.vector.y = msg->gyro.y;
    ang_vel_imu.vector.z = msg->gyro.z;
    ang_vel_imu.header.stamp.sec = msg->header.stamp.sec;
    ang_vel_imu.header.stamp.nsec = msg->header.stamp.nsec;
    ang_vel_imu.header.frame_id = "IMU";
    try {
        tfBuffer.transform(ang_vel_imu,ang_vel_auv,"AUV");
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }
    seen_ang_vel = true;
    update_last_state();
}

void Imu::set_prev_state() {
    prev_q_nwu_auv = q_nwu_auv;
}


DepthSensor::DepthSensor(double pos_auv_depth, std::string name, bool u_o_c) : Sensor(name,u_o_c) {
    prev_z = 0;
    z = 0;
}

void DepthSensor::depth_cb(const std_msgs::Float64::ConstPtr& msg) {
    // geometry_msgs::PointStamped in;
    // in.header.stamp = ros::Time::now();
    // in.header.frame_id = "DEPTH_SENSOR";
    // in.point.x = 0;
    // in.point.y = 0;
    // in.point.z = msg.data;
    // geometry_msgs::PointStamped out;
    // tfBuffer.transform<geometry_msgs::PointStamped>(&in,&out,"AUV",ros::Duration(0.0));
    // if(pos_nwu_auv == NULL) {
    //     pos_nwu_auv = new tf2::Vector3(0,0,out.point.z);
    // }
    // pose_nwu_auv.position.z = out.z
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
}
