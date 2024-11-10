#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>


class DVLRepublishNode : public rclcpp::Node {
public:
    DVLRepublishNode() : Node("dvl_republish"){
        this->declare_parameter<float>("variance", 0.0);
        this->get_parameter("variance", variance);

        public_dvl = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/sensors/dvl/pose", 1);
        odom_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensors/dvl/raw", 100, std::bind(&DVLRepublishNode::odom_cb, this, std::placeholders:_1)
            );
    }
private:
    void odom_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr& msg) {
        auto new_msg = *msg;
        new_msg.header.frame_id = "dvl";
        new_msg.pose.covariance[18] = variance;
        new_msg.pose.covariance[24] = variance;
        new_msg.pose.covariance[30] = variance;
        pub_dvl->publish(new_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_dvl;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_sub;
    float variance;

}

int main(int argc, chae **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DVLRepublishNode>());
    return 0;
}