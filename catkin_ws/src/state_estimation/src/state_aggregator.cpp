#include <ros/ros.h>
#include "sensor.h"
#include "dvl.h"
#include "imu.h"
#include "depth.h"
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>

bool update_state_on_clock;
ros::Time last_clock_msg;
Sensor* z_estimators[1];
Sensor* quat_estimators[1];
Sensor* pos_estimators[1];
ros::Publisher pub_pose;
ros::Publisher pub_x;
ros::Publisher pub_y;
ros::Publisher pub_z;
ros::Publisher pub_theta_x;
ros::Publisher pub_theta_y;
ros::Publisher pub_theta_z;
ros::Publisher pub_av;
ros::Publisher pub_imu_status;
ros::Publisher pub_dvl_status;
ros::Publisher pub_depth_status;
DepthSensor* depth;
Imu* imu;
Dvl* dvl;
void broad_cast_pose(const geometry_msgs::Pose& msg);

double RAD_TO_DEG = 180.0 / 3.14159265;

void update_state(const ros::TimerEvent& event) {
    std_msgs::Int32 depth_status_msg;
    depth_status_msg.data = depth->is_active();
    pub_depth_status.publish(depth_status_msg);

    std_msgs::Int32 imu_status_msg;
    imu_status_msg.data = imu->is_active();
    pub_imu_status.publish(imu_status_msg);

    std_msgs::Int32 dvl_status_msg;
    dvl_status_msg.data = dvl->is_active();
    pub_dvl_status.publish(dvl_status_msg);


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
            q_nwu_auv.w = sensor->q_nwu_auv.getW();
            q_nwu_auv.x = sensor->q_nwu_auv.getX();
            q_nwu_auv.y = sensor->q_nwu_auv.getY();
            q_nwu_auv.z = sensor->q_nwu_auv.getZ();
            found_quat = true;
            break;
        }
    }

    for(Sensor* sensor: pos_estimators) {
        if(sensor->is_active()) {
            x.data = sensor->x;
            y.data = sensor->y;
            found_x = true;
            found_y = true;
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

        tf2::Quaternion quat_tf2_format(q_nwu_auv.x,q_nwu_auv.y,q_nwu_auv.z,q_nwu_auv.w);
        tf2::Matrix3x3 mat(quat_tf2_format);
        double yaw;
        double pitch;
        double roll;
        mat.getEulerYPR(yaw,pitch,roll);

        std_msgs::Float64 yaw_msg;
        std_msgs::Float64 pitch_msg;
        std_msgs::Float64 roll_msg;

        yaw *= RAD_TO_DEG;
        pitch *= RAD_TO_DEG;
        roll *= RAD_TO_DEG;

        yaw_msg.data = yaw;
        pitch_msg.data = pitch;
        roll_msg.data = roll;

        // ROS_DEBUG("roll: %lf, pitch: %lf, yaw: %lf",roll, pitch, yaw);

        pub_theta_x.publish(roll_msg);
        pub_theta_y.publish(pitch_msg);
        pub_theta_z.publish(yaw_msg);

        broad_cast_pose(pose);
    }

}

void broad_cast_pose(const geometry_msgs::Pose& msg) {
    if(update_state_on_clock) {
        ros::Time now = ros::Time::now();
        if(now == last_clock_msg) {
            return;
        }
        last_clock_msg.sec = now.sec;
        last_clock_msg.nsec = now.nsec;
    }

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped1;
    transformStamped1.header.stamp = ros::Time::now();
    transformStamped1.header.frame_id = "world";
    transformStamped1.child_frame_id = "auv_base";
    transformStamped1.transform.translation.x = msg.position.x;
    transformStamped1.transform.translation.y = msg.position.y;
    transformStamped1.transform.translation.z = msg.position.z;
    transformStamped1.transform.rotation = msg.orientation;
    br.sendTransform(transformStamped1);

    geometry_msgs::TransformStamped transformStamped2;
    transformStamped2.header.stamp = ros::Time::now();
    transformStamped2.header.frame_id = "world_rotation";
    transformStamped2.child_frame_id = "auv_rotation";
    transformStamped2.transform.translation.x = 0;
    transformStamped2.transform.translation.y = 0;
    transformStamped2.transform.translation.z = 0;
    transformStamped2.transform.rotation = msg.orientation;
    br.sendTransform(transformStamped2);
}

void set_imu_params(IMU_PARAMS& params, ros::NodeHandle& n) {

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

void set_dvl_params(DVL_PARAMS& params, ros::NodeHandle& n) {
    if(!n.getParam("q_dvlnominal_dvl_w",params.q_dvlnominal_dvl_w)) {
        ROS_ERROR("Failed to get param 'q_dvlnominal_dvl_w'");
    }
    if(!n.getParam("q_dvlnominal_dvl_x",params.q_dvlnominal_dvl_x)) {
        ROS_ERROR("Failed to get param 'q_dvlnominal_dvl_x'");
    }
    if(!n.getParam("q_dvlnominal_dvl_y",params.q_dvlnominal_dvl_y)) {
        ROS_ERROR("Failed to get param 'q_dvlnominal_dvl_y'");
    }
    if(!n.getParam("q_dvlnominal_dvl_z",params.q_dvlnominal_dvl_z)) {
        ROS_ERROR("Failed to get param 'q_dvlnominal_dvl_z'");
    }
    if(!n.getParam("auv_dvl_offset_x",params.pos_auv_dvl_x)) {
        ROS_ERROR("Failed to get param 'pos_auv_dvl_x'");
    }
    if(!n.getParam("auv_dvl_offset_y",params.pos_auv_dvl_y)) {
        ROS_ERROR("Failed to get param 'pos_auv_dvl_y'");
    }
    if(!n.getParam("auv_dvl_offset_z",params.pos_auv_dvl_z)) {
        ROS_ERROR("Failed to get param 'pos_auv_dvl_z'");
    }
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"state_aggregator");
    ros::NodeHandle n;
    // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }
    ros::Duration(0.5).sleep();

    if(!n.getParam("update_state_on_clock",update_state_on_clock)) {
        ROS_ERROR("Failed to get param 'update_state_on_clock'");
    }

    IMU_PARAMS q_imunominal_imu_s;
    set_imu_params(q_imunominal_imu_s,n);
    imu = new Imu(q_imunominal_imu_s,std::string("imu"),update_state_on_clock);
    ros::Subscriber sub_quat = n.subscribe("/sensors/imu/quaternion",100,&Imu::quat_cb, imu);
    ros::Subscriber sub_ang_vel = n.subscribe("/sensors/imu/angular_velocity",100,&Imu::ang_vel_cb, imu);
    quat_estimators[0] = imu;

    DVL_PARAMS dvl_s;
    set_dvl_params(dvl_s,n);
    dvl = new Dvl(dvl_s,std::string("dvl"),update_state_on_clock, imu);
    ros::Subscriber sub_dvl = n.subscribe("sensors/dvl/pose",100,&Dvl::dr_cb, dvl);
    pos_estimators[0] = dvl;


    // double pos_auv_depth = 0;
    // if(!n.getParam("pos_auv_dvl_z",pos_auv_depth)) {
    //     ROS_ERROR("Failed to get param 'pos_auv_dvl_z'");
    // }
    depth = new DepthSensor(std::string("depth"),update_state_on_clock);
    ros::Subscriber sub_depth = n.subscribe("/sensors/depth/z",100,&DepthSensor::depth_cb, depth);
    z_estimators[0] = depth;

    pub_pose = n.advertise<geometry_msgs::Pose>("/state/pose",1);
    pub_x = n.advertise<std_msgs::Float64>("/state/x",1);
    pub_y = n.advertise<std_msgs::Float64>("/state/y",1);
    pub_z = n.advertise<std_msgs::Float64>("/state/z",1);
    pub_theta_x = n.advertise<std_msgs::Float64>("/state/theta/x",1);
    pub_theta_y = n.advertise<std_msgs::Float64>("/state/theta/y",1);
    pub_theta_z = n.advertise<std_msgs::Float64>("/state/theta/z",1);
    pub_av = n.advertise<geometry_msgs::Vector3>("/state/angular_velocity",1);
    pub_imu_status = n.advertise<std_msgs::Int32>("/sensors/imu/status",1);
    pub_dvl_status = n.advertise<std_msgs::Int32>("/sensors/dvl/status",1);
    pub_depth_status = n.advertise<std_msgs::Int32>("/sensors/depth/status",1);


    

    int update_rate;
    if(!n.getParam("update_rate",update_rate)) {
        ROS_ERROR("Failed to get param 'update_rate'");
    }
    double freq = 1.0/update_rate;
    ros::spinOnce();

    // broad_cast_NWU_NED();
    ros::Timer timer = n.createTimer(ros::Duration(freq), update_state);

    ros::spin();
    delete depth;
    delete imu;
    return 0;
}