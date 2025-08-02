#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Node that republishes a depth (Float64) reading as a PoseWithCovarianceStamped on /sensors/depth/pose

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_republisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // private

    // Load variance parameter (default to 0.01 if not set)
    double variance;
    pnh.param("variance", variance, 0.01);

    // Publisher for the depth as a PoseWithCovarianceStamped
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/sensors/depth/pose", 10);

    // Callback: convert Float64 depth to PoseWithCovarianceStamped
    auto depth_cb = [&](const std_msgs::Float64::ConstPtr &msg)
    {
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        // Frame must match the frame of the depth value. i.e "odom"
        pose_msg.header.frame_id = "odom";

        // Position: only Z changes (invert sign if your sensor convention requires)
        pose_msg.pose.pose.position.x = 0.0;
        pose_msg.pose.pose.position.y = 0.0;
        pose_msg.pose.pose.position.z = -msg->data;

        // Orientation: identity quaternion (no rotation)
        pose_msg.pose.pose.orientation.x = 0.0;
        pose_msg.pose.pose.orientation.y = 0.0;
        pose_msg.pose.pose.orientation.z = 0.0;
        pose_msg.pose.pose.orientation.w = 1.0;

        // Initialize all covariances to zero
        for (size_t i = 0; i < 36; ++i)
        {
            pose_msg.pose.covariance[i] = 0.0;
        }
        // Set variance on Z-axis (index 2*6 + 3 - 1 = 14)
        pose_msg.pose.covariance[14] = variance;

        pub_pose.publish(pose_msg);
    };

    // Subscriber for the raw depth value
    ros::Subscriber sub_depth = nh.subscribe<std_msgs::Float64>(
        "/sensors/depth/z", 10, depth_cb);

    ros::spin();
    return 0;
}
