#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class MovellaRepublisher
{
public:
    MovellaRepublisher(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pub_imu_ = nh.advertise<sensor_msgs::Imu>("sensors/imu/data", 20);

        sub_imu_.subscribe(nh, "imu/data", 100, ros::TransportHints().tcpNoDelay());
        sub_free_acc_.subscribe(nh, "filter/free_acceleration", 100, ros::TransportHints().tcpNoDelay());

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Imu, geometry_msgs::Vector3Stamped>
            SyncPolicy;

        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), sub_imu_, sub_free_acc_));
        sync_->setMaxIntervalDuration(ros::Duration(0.05));
        sync_->setInterMessageLowerBound(ros::Duration(0.010)); // applies to all inputs
        sync_->registerCallback(boost::bind(&MovellaRepublisher::callback, this, _1, _2));
        pnh.param<std::string>("frame_id", frame_override_, "imu");
    }

private:
    void callback(const sensor_msgs::ImuConstPtr &imu_msg, const geometry_msgs::Vector3StampedConstPtr &free_acc_msg)
    {
        sensor_msgs::Imu new_imu;
        new_imu.header.stamp = imu_msg->header.stamp;

        if (!frame_override_.empty())
            new_imu.header.frame_id = frame_override_;
        else
            new_imu.header.frame_id = imu_msg->header.frame_id;

        new_imu.orientation = imu_msg->orientation;
        new_imu.orientation_covariance = imu_msg->orientation_covariance;
        new_imu.angular_velocity = imu_msg->angular_velocity;
        new_imu.angular_velocity_covariance = imu_msg->angular_velocity_covariance;
        new_imu.linear_acceleration = free_acc_msg->vector;
        new_imu.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;

        double dt = fabs((imu_msg->header.stamp - free_acc_msg->header.stamp).toSec());
        if (dt > 0.015)
            ROS_WARN_THROTTLE(5.0, "IMU vs FreeAccel timestamp diff: %.1f ms", dt * 1000.0);
        pub_imu_.publish(new_imu);
    }

    ros::Publisher pub_imu_;
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu_;
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub_free_acc_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::Vector3Stamped>>> sync_;
    std::string frame_override_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "movella_republisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    MovellaRepublisher node(nh, pnh);

    ros::spin();
    return 0;
}
