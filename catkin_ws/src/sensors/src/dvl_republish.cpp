#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

ros::Publisher pub_dvl;
double variance;

void rawDvlCb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    geometry_msgs::TwistWithCovarianceStamped out;
    out.header = msg->header;
    out.header.frame_id = "dvl"; // check auv if not work, should mathc your EKF base_link_frame
    out.twist.twist = msg->twist;

    // Zero all covariances…
    out.twist.covariance.assign(36, 0.0);
    // then set x, y, z variances
    out.twist.covariance[0] = variance;  // var(linear.x)
    out.twist.covariance[7] = variance;  // var(linear.y)
    out.twist.covariance[14] = variance; // var(linear.z)

    pub_dvl.publish(out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dvl_republish");
    ros::NodeHandle nh("~");

    nh.param("variance");

    // Advertise the _with_covariance message
    pub_dvl = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/sensors/dvl/twist", 10);

    // Subscribe to your raw DVL velocities
    ros::Subscriber sub = nh.subscribe<geometry_msgs::TwistStamped>(
        "/sensors/dvl/raw", 10, rawDvlCb);

    ros::spin();
    return 0;
}
