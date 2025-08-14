// zed_pose_to_auv_pose.cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <string>

class ZedToAuvCompose
{
public:
    ZedToAuvCompose(ros::NodeHandle &nh, ros::NodeHandle &pnh) : buffer_(ros::Duration(30.0)), listener_(buffer_)
    {
        pnh.param<std::string>("input_topic", in_topic_, std::string("/zed2i/zed_node/pose_with_covariance"));
        pnh.param<std::string>("output_topic", out_topic_, std::string("sensors/zed2i/pose"));
        pnh.param<std::string>("map_frame", map_frame_, std::string("map"));
        pnh.param<std::string>("zed_frame", zed_frame_, std::string("zed"));
        pnh.param<std::string>("auv_frame", auv_frame_, std::string("auv"));
        pnh.param<double>("tf_timeout", tf_timeout_sec_, 0.2);
        pnh.param<bool>("use_adjoint_covariance", use_adjoint_covariance_, true);

        pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(out_topic_, 100);
        sub_ = nh.subscribe(in_topic_, 10, &ZedToAuvCompose::cb, this);

        ROS_INFO_STREAM("zed_pose_to_auv_pose_tf2_compose: input=" << in_topic_ << " output=" << out_topic_ << " map=" << map_frame_ << " zed=" << zed_frame_ << " auv=" << auv_frame_ << " adjoint_cov=" << (use_adjoint_covariance_ ? "true" : "false"));
    }

private:
    static Eigen::Matrix4d Rt(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;
        return T;
    }

    static Eigen::Matrix3d quat_to_R(const geometry_msgs::Quaternion &q)
    {
        tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 tfR(tfq);
        Eigen::Matrix3d R;
        for (int i = 0; i < 3; ++i)
        {
            tf2::Vector3 row = tfR.getRow(i);
            R(i, 0) = row.x();
            R(i, 1) = row.y();
            R(i, 2) = row.z();
        }
        return R;
    }

    static tf2::Quaternion R_to_quat(const Eigen::Matrix3d &R)
    {
        tf2::Matrix3x3 tfR(
            R(0, 0), R(0, 1), R(0, 2),
            R(1, 0), R(1, 1), R(1, 2),
            R(2, 0), R(2, 1), R(2, 2));
        tf2::Quaternion q;
        tfR.getRotation(q);
        return q;
    }

    static Eigen::Matrix3d skew(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d S;
        S << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return S;
    }

    // Python-equivalent simple block-diagonal rotation:
    static void rotate_pose_cov_block(const geometry_msgs::PoseWithCovariance::_covariance_type &cov_in, const Eigen::Matrix3d &R_za, geometry_msgs::PoseWithCovariance::_covariance_type &cov_out)
    {
        Eigen::Matrix<double, 6, 6> C;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                C(i, j) = cov_in[i * 6 + j];

        Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
        J.block<3, 3>(0, 0) = R_za; // position
        J.block<3, 3>(3, 3) = R_za; // orientation

        Eigen::Matrix<double, 6, 6> Crot = J * C * J.transpose();
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                cov_out[i * 6 + j] = Crot(i, j);
    }
    static void rotate_pose_cov_adjoint(const geometry_msgs::PoseWithCovariance::_covariance_type &cov_in, const Eigen::Matrix3d &R_pose, const Eigen::Matrix3d &R_za, const Eigen::Vector3d &t_za, geometry_msgs::PoseWithCovariance::_covariance_type &cov_out)
    {
        Eigen::Matrix<double, 6, 6> C;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                C(i, j) = cov_in[i * 6 + j];

        Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
        J.block<3, 3>(0, 0) = R_za;
        J.block<3, 3>(0, 3) = -R_pose * skew(t_za);
        J.block<3, 3>(3, 3) = R_za;

        Eigen::Matrix<double, 6, 6> Crot = J * C * J.transpose();
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                cov_out[i * 6 + j] = Crot(i, j);
    }

    void cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;

        // T_map_zed from measurement
        Eigen::Matrix3d R_map_zed = quat_to_R(q);
        Eigen::Vector3d t_map_zed(p.x, p.y, p.z);
        Eigen::Matrix4d T_map_zed = Rt(R_map_zed, t_map_zed);

        if (!buffer_.canTransform(zed_frame_, auv_frame_, msg->header.stamp, ros::Duration(tf_timeout_sec_)))
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "No TF " << zed_frame_ << "->" << auv_frame_ << " at t=" << msg->header.stamp.toSec());
            return;
        }

        geometry_msgs::TransformStamped tf_msg;
        try
        {
            tf_msg = buffer_.lookupTransform(zed_frame_, auv_frame_, msg->header.stamp, ros::Duration(tf_timeout_sec_));
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "lookupTransform failed: " << ex.what());
            return;
        }

        // T_zed_auv from TF
        Eigen::Matrix3d R_za = quat_to_R(geometry_msgs::Quaternion{tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w});
        Eigen::Vector3d t_za(tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z);
        Eigen::Matrix4d T_zed_auv = Rt(R_za, t_za);

        // Compose: T_map_auv = T_map_zed * T_zed_auv
        Eigen::Matrix4d T_map_auv = T_map_zed * T_zed_auv;
        Eigen::Matrix3d R_out = T_map_auv.block<3, 3>(0, 0);
        Eigen::Vector3d t_out = T_map_auv.block<3, 1>(0, 3);

        tf2::Quaternion q_out = R_to_quat(R_out);

        // Publish
        geometry_msgs::PoseWithCovarianceStamped out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = map_frame_;
        out.pose.pose.position.x = t_out.x();
        out.pose.pose.position.y = t_out.y();
        out.pose.pose.position.z = t_out.z();
        out.pose.pose.orientation.x = q_out.x();
        out.pose.pose.orientation.y = q_out.y();
        out.pose.pose.orientation.z = q_out.z();
        out.pose.pose.orientation.w = q_out.w();

        // Covariance rotation
        if (use_adjoint_covariance_)
        {
            rotate_pose_cov_adjoint(msg->pose.covariance, R_map_zed, R_za, t_za, out.pose.covariance);
        }
        else
        {
            rotate_pose_cov_block(msg->pose.covariance, R_za, out.pose.covariance);
        }

        pub_.publish(out);
    }

    // Params / state
    std::string in_topic_, out_topic_, map_frame_, zed_frame_, auv_frame_;
    double tf_timeout_sec_{0.3};
    bool use_adjoint_covariance_{true};

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_pose_republisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ZedToAuvCompose node(nh, pnh);
    ros::spin();
    return 0;
}