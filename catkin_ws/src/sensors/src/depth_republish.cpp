#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_republisher");
    ros::NodeHandle nh, pnh("~");

    double z_variance;
    pnh.param("z_variance", z_variance, 0.01); // basically just saying really trust z, but everything else dont bother
    double big_variance;
    pnh.param("big_variance", big_variance, 1e6);
    double z_sign;
    pnh.param("z_sign", z_sign, -1.0);
    std::string frame_id;
    pnh.param<std::string>("frame_id", frame_id, "odom");

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/sensors/depth/pose", 10);

    auto cb = [&](const std_msgs::Float64::ConstPtr &msg)
    {
        geometry_msgs::PoseWithCovarianceStamped out;
        out.header.stamp = ros::Time::now();
        out.header.frame_id = frame_id;

        out.pose.pose.position.x = 0.0;
        out.pose.pose.position.y = 0.0;
        out.pose.pose.position.z = z_sign * msg->data;

        out.pose.pose.orientation.x = 0.0;
        out.pose.pose.orientation.y = 0.0;
        out.pose.pose.orientation.z = 0.0;
        out.pose.pose.orientation.w = 1.0;

        for (int i = 0; i < 36; ++i)
            out.pose.pose.covariance[i] = 0.0;
        out.pose.pose.covariance[0] = big_variance;
        out.pose.pose.covariance[7] = big_variance;
        out.pose.pose.covariance[14] = z_variance;
        out.pose.pose.covariance[21] = big_variance;
        out.pose.pose.covariance[28] = big_variance;
        out.pose.pose.covariance[35] = big_variance;
        pub.publish(out);
    };

    auto sub = nh.subscribe<std_msgs::Float64>("/sensors/depth/z", 10, cb);
    ros::spin();
    return 0;
}