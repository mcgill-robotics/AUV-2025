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

// Imu::Imu(tf2::Quaternion q_auv_imu, ros::NodeHandle n) : Sensor(){

//     static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//     geometry_msgs::TransformStamped static_transformStamped;
    
//     static_transformStamped.header.stamp = ros::Time::now();
//     static_transformStamped.header.frame_id = "AUV";
//     static_transformStamped.child_frame_id = "IMU";
//     static_transformStamped.transform.translation.x = 0;
//     static_transformStamped.transform.translation.y = 0;
//     static_transformStamped.transform.translation.z = 0;

//     static_transformStamped.transform.rotation.x = q_auv_imu.x();
//     static_transformStamped.transform.rotation.y = q_auv_imu.y();
//     static_transformStamped.transform.rotation.z = q_auv_imu.z();
//     static_transformStamped.transform.rotation.w = q_auv_imu.w();
//     static_broadcaster.sendTransform(static_transformStamped);

//     ros::Subscriber sub = n.subscribe("/sensors/imu/quaternion",1000,quat_cb);

//     sensor_name = "imu";
    
// }
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
    prev_depth = 0;
    depth = 0;
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
    depth = msg->data;
    update_last_state();

}

bool DepthSensor::has_valid_data() {
    return true;
}

bool DepthSensor::has_different_data() {
    return prev_depth != depth;
}

void DepthSensor::set_prev_state() {
    prev_depth = depth;
}
