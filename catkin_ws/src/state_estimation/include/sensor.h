#ifndef SENSOR
#define SENSOR

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include "ros/ros.h"
#include "tf2/LinearMath/Transform.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>

class Sensor {
    public:
        Sensor(std::string name, bool update_on_clock);
        virtual ~Sensor();
        std::string sensor_name;
        virtual bool is_active(void);
        double z;
        geometry_msgs::Quaternion q_nwu_auv;
        geometry_msgs::Vector3Stamped ang_vel_auv;
    protected:
        ros::Time last_clock_msg;
        bool update_on_clock;
        tf2_ros::TransformBroadcaster br;
        ros::Time last_unique_state_time;
        ros::Time last_error_message_time;
        ros::Duration time_before_considered_inactive;
        void update_last_state(void);
        virtual void set_prev_state(void) = 0;
        virtual bool has_different_data(void) = 0;
};

#endif