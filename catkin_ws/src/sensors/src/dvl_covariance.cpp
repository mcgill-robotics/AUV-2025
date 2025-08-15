#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <string>
#include <sstream>

class WaterLinkedDriver
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_pose_;
    
    serial::Serial conn_;
    
    // Parameters
    std::string port_;
    int baudrate_;
    double fom_threshold_;
    std::string frame_id_;
    double big_var_ang_;
    double big_var_ori_;
    bool publish_dr_pose_;
    
    static constexpr double RAD_PER_DEG = M_PI / 180.0;

public:
    WaterLinkedDriver() : nh_(), pnh_("~")
    {
        // Initialize publishers
        pub_twist_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/sensors/dvl/twist", 10);
        pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/sensors/dvl/pose", 10);
        
        // Get parameters
        pnh_.param<std::string>("port", port_, "");
        pnh_.param<int>("baudrate", baudrate_, 115200);
        pnh_.param<double>("fom_threshold", fom_threshold_, 25.0);
        pnh_.param<std::string>("frame_id", frame_id_, "dvl");
        pnh_.param<double>("big_var_ang", big_var_ang_, 1e6);
        pnh_.param<double>("big_var_ori", big_var_ori_, 1e6);
        pnh_.param<bool>("publish_dr_pose", publish_dr_pose_, false);
    }
    
    bool initialize()
    {
        if (port_.empty()) {
            ROS_ERROR("Port parameter not specified");
            return false;
        }
        
        try {
            conn_.setPort(port_);
            conn_.setBaudrate(baudrate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            conn_.setTimeout(timeout);
            conn_.open();
        } catch (const serial::IOException& e) {
            ROS_ERROR("ERR: cannot open serial port %s: %s", port_.c_str(), e.what());
            return false;
        }
        
        // Reset DR
        conn_.sendBreak(250);
        conn_.write("wcr\r\n");
        conn_.flush();
        
        // Wait for response
        while (ros::ok()) {
            std::string line = conn_.readline();
            boost::trim(line);
            if (boost::starts_with(line, "wra") || boost::starts_with(line, "wrn")) {
              break;
            }
        }
        
        ROS_INFO("Started DVL driver on %s @ %d baud", port_.c_str(), baudrate_);
        return true;
    }
    
    void run()
    {
        while (ros::ok()) { //TODO: add timeout to prevent infinite loop
            std::string raw = conn_.readline();
            boost::trim(raw);
            
            if (raw.empty()) {
                continue;
            }
            
            ros::Time stamp = ros::Time::now();
            
            if (boost::starts_with(raw, "wrz")) {
                processVelocityMessage(raw, stamp);
            } else if (publish_dr_pose_ && boost::starts_with(raw, "wrp")) {
                processPoseMessage(raw, stamp);
            }
        }
        
        conn_.close();
    }

private:
    void processVelocityMessage(const std::string& raw, const ros::Time& stamp)
    {
        // wrz,[vx],[vy],[vz],[valid],[altitude],[fom],[cov],...
        std::vector<std::string> parts;
        std::string clean_raw = raw;
        boost::replace_all(clean_raw, "*", "");
        boost::split(parts, clean_raw, boost::is_any_of(","));
        
        if (parts.size() < 8) {
            ROS_WARN("Malformed wrz: %s", raw.c_str());
            return;
        }
        
        try {
            double vx = boost::lexical_cast<double>(parts[1]);
            double vy = boost::lexical_cast<double>(parts[2]);
            double vz = boost::lexical_cast<double>(parts[3]);
            bool valid = (boost::to_lower_copy(parts[4]) == "y");
            double fom = boost::lexical_cast<double>(parts[6]);
            
            // Parse covariance matrix (3x3 row-major, semicolon separated)
            std::vector<std::string> cov_parts;
            boost::split(cov_parts, parts[7], boost::is_any_of(";"));
            
            if (cov_parts.size() != 9) {
                ROS_WARN("Invalid covariance format in wrz: %s", raw.c_str());
                return;
            }
            
            std::vector<double> cov3(9);
            for (size_t i = 0; i < 9; ++i) {
                cov3[i] = boost::lexical_cast<double>(cov_parts[i]);
            }
            
            if (!valid || fom > fom_threshold_) {
                return;
            }
            
            // Create 6x6 covariance matrix
            boost::array<double, 36> cov6;
            std::fill(cov6.begin(), cov6.end(), 0.0);
            
            // Linear velocity covariance (3x3 block at top-left)
            cov6[0] = cov3[0];   cov6[1] = cov3[1];   cov6[2] = cov3[2];
            cov6[6] = cov3[3];   cov6[7] = cov3[4];   cov6[8] = cov3[5];
            cov6[12] = cov3[6];  cov6[13] = cov3[7];  cov6[14] = cov3[8];
            
            // Angular velocity unknown → huge variances (diagonal of bottom-right 3x3 block)
            cov6[21] = big_var_ang_;
            cov6[28] = big_var_ang_;
            cov6[35] = big_var_ang_;
            
            geometry_msgs::TwistWithCovarianceStamped msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = frame_id_;
            
            // FRD -> ENU (x: fwd->east, y: right->north, z: down->up)
            msg.twist.twist.linear.x = vx;
            msg.twist.twist.linear.y = -vy;
            msg.twist.twist.linear.z = -vz;
            msg.twist.covariance = cov6;
            
            pub_twist_.publish(msg);
            
        } catch (const boost::bad_lexical_cast& e) {
            ROS_WARN("Malformed wrz: %s", raw.c_str());
        }
    }
    
    void processPoseMessage(const std::string& raw, const ros::Time& stamp)
    {
        // wrp,[tstamp],[x],[y],[z],[pos_std],[roll],[pitch],[yaw],...
        std::vector<std::string> parts;
        std::string clean_raw = raw;
        boost::replace_all(clean_raw, "*", "");
        boost::split(parts, clean_raw, boost::is_any_of(","));
        
        if (parts.size() < 9) {
            ROS_WARN("Malformed wrp: %s", raw.c_str());
            return;
        }
        
        try {
            double x = boost::lexical_cast<double>(parts[2]);
            double y = boost::lexical_cast<double>(parts[3]);
            double z = boost::lexical_cast<double>(parts[4]);
            double pos_std = boost::lexical_cast<double>(parts[5]);
            double roll = boost::lexical_cast<double>(parts[6]);
            double pitch = boost::lexical_cast<double>(parts[7]);
            double yaw = boost::lexical_cast<double>(parts[8]);
            
            geometry_msgs::PoseWithCovarianceStamped pose;
            pose.header.stamp = stamp;
            pose.header.frame_id = frame_id_;
            
            pose.pose.pose.position.x = x;
            pose.pose.pose.position.y = -y;
            pose.pose.pose.position.z = -z;
            
            // Convert Euler angles to quaternion
            tf::Quaternion q = tf::createQuaternionFromRPY(
                roll * RAD_PER_DEG,
                -pitch * RAD_PER_DEG,
                -yaw * RAD_PER_DEG
            );
            
            pose.pose.pose.orientation.x = q.x();
            pose.pose.pose.orientation.y = q.y();
            pose.pose.pose.orientation.z = q.z();
            pose.pose.pose.orientation.w = q.w();
            
            // Set covariance
            double pos_var = pos_std * pos_std;
            boost::array<double, 36> cov;
            std::fill(cov.begin(), cov.end(), 0.0);
            
            cov[0] = pos_var;      // x variance
            cov[7] = pos_var;      // y variance
            cov[14] = pos_var;     // z variance
            cov[21] = big_var_ori_; // roll variance
            cov[28] = big_var_ori_; // pitch variance
            cov[35] = big_var_ori_; // yaw variance
            
            pose.pose.covariance = cov;
            
            pub_pose_.publish(pose);
            
        } catch (const boost::bad_lexical_cast& e) {
            ROS_WARN("Malformed wrp: %s", raw.c_str());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waterlinked_driver");
    
    WaterLinkedDriver driver;
    
    if (!driver.initialize()) {
        return 1;
    }
    
    driver.run();
    
    return 0;
}