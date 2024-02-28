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
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <ros/console.h>

Sensor::~Sensor() {
    delete tfBuffer;
    delete tfListener;
}

Sensor::Sensor(std::string name) {
    tfBuffer = new tf2_ros::Buffer;
    tfListener = new tf2_ros::TransformListener(*tfBuffer);
    sensor_name = name;
    last_unique_state_time = ros::Time(0,0);
    last_error_message_time = ros::Time(0,0);
    time_before_considered_inactive = ros::Duration(1.0);
}

void Sensor::update_last_state() {
    if(has_different_data()) {
        if(ros::Time::now() - last_unique_state_time > time_before_considered_inactive) {
                ROS_INFO("%s has become active",sensor_name.c_str());
            }
        last_unique_state_time = ros::Time::now();
        set_prev_state();
    }
}

bool Sensor::is_active() {
    if(ros::Time::now() == ros::Time(0,0)) return false;
    if(!has_valid_data()) return false;
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

// Dvl::Dvl(tf2::Quaternion q_auv_dvl, tf2::Vector3 pos_auv_dvl, Imu i, ros::NodeHandle n) : Sensor(){
//     imu = i;

//     static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//     geometry_msgs::TransformStamped static_transformStamped;
    
//     static_transformStamped.header.stamp = ros::Time::now();
//     static_transformStamped.header.frame_id = "AUV";
//     static_transformStamped.child_frame_id = "DVL";
//     static_transformStamped.transform.translation.x = pos_auv_dvl.getX();
//     static_transformStamped.transform.translation.y = pos_auv_dvl.getY();
//     static_transformStamped.transform.translation.z = pos_auv_dvl.getZ();

//     static_transformStamped.transform.rotation.x = q_auv_dvl.x();
//     static_transformStamped.transform.rotation.y = q_auv_dvl.y();
//     static_transformStamped.transform.rotation.z = q_auv_dvl.z();
//     static_transformStamped.transform.rotation.w = q_auv_dvl.w();
//     static_broadcaster.sendTransform(static_transformStamped);

//     ros::Subscriber sub = n.subscribe("/sensors/dvl/pose",1000,dr_cb);

//     sensor_name = "dvl";
// }

// Dvl::dr_cb(auv_msgs::DeadReckonReport) {
    
// }

Imu::Imu(IMU_PARAMS q_imunominal_imu_s, ros::NodeHandle n, std::string name) : Sensor(name){

    tf2::Quaternion q_imunominal_imu(q_imunominal_imu_s.x,q_imunominal_imu_s.y,q_imunominal_imu_s.z,q_imunominal_imu_s.w);
    tf2::Quaternion q_imunominal_auv(1.0, 0.0, 0.0, 0.0);

    tf2::Quaternion q_imu_auv = q_imunominal_imu.inverse() * q_imunominal_auv;


    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "IMU";
    static_transformStamped.child_frame_id = "AUV";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;

    static_transformStamped.transform.rotation.x = q_imu_auv.x();
    static_transformStamped.transform.rotation.y = q_imu_auv.y();
    static_transformStamped.transform.rotation.z = q_imu_auv.z();
    static_transformStamped.transform.rotation.w = q_imu_auv.w();
    static_broadcaster.sendTransform(static_transformStamped);    

    q_nwu_auv = Quaternion(0.0, 0.0, 0.0, 1.0)
}

void Imu::quat_cb(const sbg_driver::SbgEkfQuat::ConstPtr& msg) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp.secs = msg.head.stamp.secs;
    transformStamped.header.stamp.nsecs = msg.head.stamp.nsecs;
    transformStamped.header.frame_id = "NED";
    transformStamped.child_frame_id = "IMU";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;

    transformStamped.transform.rotation.x = msg.quaternion.x;
    transformStamped.transform.rotation.y = msg.quaternion.y;
    transformStamped.transform.rotation.z = msg.quaternion.z;
    transformStamped.transform.rotation.w = msg.quaternion.w;

    br.sendTransform(transformStamped);

    geometry_msgs::TransformStamped q_nwu_auv_ts;
    try{
      q_nwu_auv_ts = tfBuffer.lookupTransform("NWU", "AUV",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.25).sleep();
      return;
    }
    q_nwu_auv.w = q_nwu_auv_ts.rotation.w
    q_nwu_auv.x = q_nwu_auv_ts.rotation.x
    q_nwu_auv.y = q_nwu_auv_ts.rotation.y
    q_nwu_auv.z = q_nwu_auv_ts.rotation.z

    update_last_state();
}

void Imu::ang_vel_cb(const sbg_driver::ImuData::ConstPts& msg) {
    tf2::Vector3 = 
}


bool Imu::has_valid_data() {
    return true;
}

bool Imu::has_different_data() {
    return prev_q_nwu_auv != q_nwu_auv;
}

void Imu::set_prev_state() {
    prev_q_nwu_auv = q_nwu_auv;
}


DepthSensor::DepthSensor(double pos_auv_depth, ros::NodeHandle& n, std::string name) : Sensor(name) {
    geometry_msgs::TransformStamped static_transformStamped;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "AUV";
    static_transformStamped.child_frame_id = "DEPTH_SENSOR";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = pos_auv_depth;

    static_transformStamped.transform.rotation.x = 0.0;
    static_transformStamped.transform.rotation.y = 0.0;
    static_transformStamped.transform.rotation.z = 0.0;
    static_transformStamped.transform.rotation.w = 1.0;
    static_broadcaster.sendTransform(static_transformStamped);
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

bool DepthSensor::has_valid_data() {
    return true;
}

bool DepthSensor::has_different_data() {
    return prev_z != z;
}

void DepthSensor::set_prev_state() {
    prev_z = z;
}
