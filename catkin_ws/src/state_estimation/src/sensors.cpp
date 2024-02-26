#include "state_estimation/sensor.h"
#include "ros/ros.h"
#include "state_estimation/dvl.h"
#include "state_estimation/imu.h"
#include "state_estimation/depth.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <auv_msgs/DeadReckonReport.h>
#include <sbg_driver/SbgEkfQuat.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>

Sensor::Sensor() {
    tfBuffer = new tf2_ros::Buffer;
    
    tfListener = new tf2_ros::TransformListener(*tfBuffer);
}

Sensor::update_last_state() {

}

Sensor::is_active() {
    
}

Dvl::Dvl(tf2::Quaternion q_auv_dvl, tf2::Vector3 pos_auv_dvl, Imu i, ros::NodeHandle n) : Sensor(){
    imu = i;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "AUV";
    static_transformStamped.child_frame_id = "DVL";
    static_transformStamped.transform.translation.x = pos_auv_dvl.getX();
    static_transformStamped.transform.translation.y = pos_auv_dvl.getY();
    static_transformStamped.transform.translation.z = pos_auv_dvl.getZ();

    static_transformStamped.transform.rotation.x = q_auv_dvl.x();
    static_transformStamped.transform.rotation.y = q_auv_dvl.y();
    static_transformStamped.transform.rotation.z = q_auv_dvl.z();
    static_transformStamped.transform.rotation.w = q_auv_dvl.w();
    static_broadcaster.sendTransform(static_transformStamped);

    ros::Subscriber sub = n.subscribe("/sensors/dvl/pose",1000,dr_cb);
}

Dvl::dr_cb(auv_msgs::DeadReckonReport) {
    
}

Imu::Imu(tf2::Quaternion q_auv_imu, ros::NodeHandle n) : Sensor(){

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "AUV";
    static_transformStamped.child_frame_id = "IMU";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;

    static_transformStamped.transform.rotation.x = q_auv_imu.x();
    static_transformStamped.transform.rotation.y = q_auv_imu.y();
    static_transformStamped.transform.rotation.z = q_auv_imu.z();
    static_transformStamped.transform.rotation.w = q_auv_imu.w();
    static_broadcaster.sendTransform(static_transformStamped);

    ros::Subscriber sub = n.subscribe("/sensors/imu/quaternion",1000,quat_cb);
    
}

DepthSensor::DepthSensor(double pos_auv_depth, ros::NodeHandle n) : Sensor() {
    geometry_msgs::TransformStamped static_transformStamped;
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

    ros::Subscriber sub = n.subscribe("/sensors/depth/z",1000,);
    pos_nwu_auv = NULL;
    prev_pos_nwu_auv
}

void DepthSensor::depth_cb(std_msgs::Float64 msg) {

    double depth_z = msg.data;
    geometry_msgs::PointStamped in;
    in.header.stamp = ros::Time::now();
    in.header.frame_id = "DEPTH_SENSOR";
    in.point.x = 0;
    in.point.y = 0;
    in.point.z = msg.data;
    geometry_msgs::PointStamped out;
    tfBuffer.transform<geometry_msgs::PointStamped>(&in,&out,"AUV",ros::Duration(0.0));
    if(pos_nwu_auv == NULL) {
        pos_nwu_auv = new tf2::Vector3(0,0,out.point.z);
    }
    pos_nwu_auv.z = out.z
    update_last_state();

    // try {
    //     transformStamped = tfBuffer.lookupTransform("DEPTH_SENSOR", "AUV",ros::Time(0));
    // }
    // catch {
    //     std::cout << "Error";
    // }

    // 
    // tf2_ros::Point3 transformedPoint;
    // tfListener
}

bool DepthSensor::has_valid_data() {
    return pos_nwu_auv != NULL;
}