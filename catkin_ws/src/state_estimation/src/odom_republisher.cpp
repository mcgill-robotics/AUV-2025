#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

bool update_state_on_clock;
ros::Time last_clock_msg;

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

double depth;

double RAD_TO_DEG = 180.0 / 3.14159265;

void broad_cast_pose(const geometry_msgs::Pose &msg);

void depth_cb(const std_msgs::Float64::ConstPtr &msg)
{
    depth = msg->data * -1;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    std_msgs::Float64 x, y, z;
    x.data = msg->pose.pose.position.x;
    y.data = msg->pose.pose.position.y;
    z.data = depth; // use the depth from the depth callback

    // get orientation and angular velocoty
    geometry_msgs::Quaternion q_nwu_auv = msg->pose.pose.orientation;
    geometry_msgs::Vector3 av = msg->twist.twist.angular;

    geometry_msgs::Pose pose = msg->pose.pose;
    pose.position.z = depth;
    pub_x.publish(x);
    pub_y.publish(y);
    pub_z.publish(z);
    pub_pose.publish(pose);

    // publish angular velocity
    pub_av.publish(av);

    // convert quaternion to Euler angles using tf2
    tf2::Quaternion quat_tf2(q_nwu_auv.x, q_nwu_auv.y, q_nwu_auv.z, q_nwu_auv.w);
    tf2::Matrix3x3 mat(quat_tf2);
    double yaw, pitch, roll;
    // Returns Euler angles in the order: yaw, pitch, roll (in radians)
    mat.getEulerYPR(yaw, pitch, roll);

    // If you need degrees instead, uncomment the following:
    // yaw   *= RAD_TO_DEG;
    // pitch *= RAD_TO_DEG;
    // roll  *= RAD_TO_DEG;

    // Prepare and publish Euler angle messages
    std_msgs::Float64 yaw_msg, pitch_msg, roll_msg;
    yaw_msg.data = yaw;
    pitch_msg.data = pitch;
    roll_msg.data = roll;
    pub_theta_x.publish(roll_msg);  // roll on state/theta/x
    pub_theta_y.publish(pitch_msg); // pitch on state/theta/y
    pub_theta_z.publish(yaw_msg);   // yaw on state/theta/z

    // publish additional status messages
    std_msgs::Int32 imu_status, dvl_status, depth_status;
    imu_status.data = 1; // for example: 1 means "OK"
    dvl_status.data = 1;
    depth_status.data = 1;
    pub_imu_status.publish(imu_status);
    pub_dvl_status.publish(dvl_status);
    pub_depth_status.publish(depth_status);

    broad_cast_pose(pose);
}

void broad_cast_pose(const geometry_msgs::Pose &msg)
{
    if (update_state_on_clock)
    {
        ros::Time now = ros::Time::now();
        if (now == last_clock_msg)
        {
            return;
        }
        last_clock_msg = now;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_republish");
    ros::NodeHandle n;

    // Subscribers
    ros::Subscriber odom_sub = n.subscribe("/odometry/filtered", 100, odom_cb);
    ros::Subscriber depth_sub = n.subscribe("/sensors/depth/z", 100, depth_cb);

    // Publishers for state information
    pub_pose = n.advertise<geometry_msgs::Pose>("/state/pose", 1);
    pub_x = n.advertise<std_msgs::Float64>("/state/x", 1);
    pub_y = n.advertise<std_msgs::Float64>("/state/y", 1);
    pub_z = n.advertise<std_msgs::Float64>("/state/z", 1);
    pub_theta_x = n.advertise<std_msgs::Float64>("/state/theta/x", 1);
    pub_theta_y = n.advertise<std_msgs::Float64>("/state/theta/y", 1);
    pub_theta_z = n.advertise<std_msgs::Float64>("/state/theta/z", 1);
    pub_av = n.advertise<geometry_msgs::Vector3>("/state/angular_velocity", 1);
    pub_imu_status = n.advertise<std_msgs::Int32>("/state/imu/status", 1);
    pub_dvl_status = n.advertise<std_msgs::Int32>("/state/dvl/status", 1);
    pub_depth_status = n.advertise<std_msgs::Int32>("/state/depth/status", 1);

    ros::param::get("/update_state_on_clock", update_state_on_clock);

    ros::spin();
    return 0;
}
