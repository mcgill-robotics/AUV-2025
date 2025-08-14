// waterlinked_driver.cpp
#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/asio.hpp>
#include <termios.h> // tcsendbreak
#include <algorithm>
#include <array>
#include <cctype>
#include <string>
#include <vector>
#include <sstream>

namespace wl
{
    using boost::asio::buffer;
    using boost::asio::io_service;
    using boost::asio::read_until;
    using boost::asio::serial_port;
    using boost::asio::serial_port_base;
    using boost::asio::write;

    static inline bool starts_with(const std::string &s, const char *prefix)
    {
        const size_t n = std::char_traits<char>::length(prefix);
        return s.size() >= n && std::equal(prefix, prefix + n, s.begin());
    }

    static inline void trim(std::string &s)
    {
        auto not_space = [](int ch)
        { return !std::isspace(ch); };
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
        s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    }

    static inline void erase_char(std::string &s, char c)
    {
        s.erase(std::remove(s.begin(), s.end(), c), s.end());
    }

    static std::vector<std::string> split(const std::string &s, char delim)
    {
        std::vector<std::string> out;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim))
            out.push_back(item);
        return out;
    }

    static bool to_double(const std::string &s, double &v)
    {
        char *end = nullptr;
        v = std::strtod(s.c_str(), &end);
        return end && *end == '\0';
    }

    static bool parse_doubles(const std::vector<std::string> &toks, size_t i0, size_t n, std::vector<double> &out)
    {
        out.clear();
        out.reserve(n);
        for (size_t i = 0; i < n; ++i)
        {
            double v;
            if (!to_double(toks[i0 + i], v))
                return false;
            out.push_back(v);
        }
        return true;
    }

    class Driver
    {
    public:
        Driver()
            : nh_(), pnh_("~"),
              io_(), port_(io_)
        {
            pnh_.param<std::string>("port", port_name_, std::string(""));
            pnh_.param<int>("baudrate", baudrate_, 115200);
            pnh_.param<double>("fom_threshold", fom_threshold_, 25.0);
            pnh_.param<std::string>("frame_id", frame_id_, std::string("dvl"));

            pnh_.param<double>("big_var_ang", big_var_ang_, 1e6); // rad^2/s^2
            pnh_.param<double>("big_var_ori", big_var_ori_, 1e6); // rad^2
            pnh_.param<bool>("publish_dr_pose", publish_dr_pose_, false);

            pub_twist_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/sensors/dvl/twist", 10);
            pub_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/sensors/dvl/pose", 10);
        }

        bool open()
        {
            if (port_name_.empty())
            {
                ROS_ERROR("~port is required");
                return false;
            }
            boost::system::error_code ec;
            port_.open(port_name_, ec);
            if (ec)
            {
                ROS_ERROR_STREAM("ERR: cannot open serial port " << port_name_ << ": " << ec.message());
                return false;
            }
            port_.set_option(serial_port_base::baud_rate(baudrate_));
            port_.set_option(serial_port_base::character_size(8));
            port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
            port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

            // Send break + reset DR: "wcr\r\n"
            ::tcsendbreak(port_.native_handle(), 0);
            const std::string reset_cmd = "wcr\r\n";
            write(port_, buffer(reset_cmd.data(), reset_cmd.size()));

            // Wait for "wra" or "wrn"
            boost::asio::streambuf buf;
            while (ros::ok())
            {
                boost::system::error_code ecr;
                std::size_t n = read_until(port_, buf, '\n', ecr);
                if (ecr)
                {
                    ROS_WARN_STREAM("Serial read error during init: " << ecr.message());
                    continue;
                }
                std::istream is(&buf);
                std::string line;
                std::getline(is, line);
                trim(line);
                if (line.empty())
                    continue;
                if (starts_with(line, "wra") || starts_with(line, "wrn"))
                    break;
            }

            ROS_INFO_STREAM("Started DVL driver on " << port_name_ << " @ " << baudrate_ << " baud");
            return true;
        }

        void spin()
        {
            boost::asio::streambuf buf;
            const double RAD_PER_DEG = M_PI / 180.0;

            while (ros::ok())
            {
                // Read one line
                boost::system::error_code ec;
                std::size_t n = read_until(port_, buf, '\n', ec);
                if (ec)
                {
                    if (ec == boost::asio::error::operation_aborted)
                        break;
                    // If no data, keep trying
                    continue;
                }

                std::istream is(&buf);
                std::string raw;
                std::getline(is, raw);
                trim(raw);
                if (raw.empty())
                {
                    ros::spinOnce();
                    continue;
                }

                // One timestamp per record
                ros::Time stamp = ros::Time::now();

                // Remove '*' chars
                erase_char(raw, '*');

                if (starts_with(raw, "wrz"))
                {
                    // wrz,[vx],[vy],[vz],[valid],[altitude],[fom],[cov],...
                    auto parts = split(raw, ',');
                    if (parts.size() < 8)
                    {
                        ROS_WARN_STREAM("Malformed wrz (too few fields): " << raw);
                        ros::spinOnce();
                        continue;
                    }

                    std::vector<double> v3;
                    if (!parse_doubles(parts, 1, 3, v3))
                    {
                        ROS_WARN_STREAM("Malformed wrz velocities: " << raw);
                        ros::spinOnce();
                        continue;
                    }
                    double vx = v3[0], vy = v3[1], vz = v3[2];

                    bool valid = false;
                    if (!parts[4].empty())
                    {
                        char c = std::tolower(parts[4][0]);
                        valid = (c == 'y');
                    }

                    double fom = 1e9;
                    if (!to_double(parts[6], fom))
                    {
                        ROS_WARN_STREAM("Malformed wrz fom: " << raw);
                        ros::spinOnce();
                        continue;
                    }

                    if (!valid || fom > fom_threshold_)
                    {
                        ros::spinOnce();
                        continue;
                    }

                    // cov3 as "a;b;c;..."
                    auto cov_tokens = split(parts[7], ';');
                    if (cov_tokens.size() < 9)
                    {
                        ROS_WARN_STREAM("Malformed wrz cov(3x3): " << raw);
                        ros::spinOnce();
                        continue;
                    }
                    std::vector<double> cov3(9, 0.0);
                    for (int i = 0; i < 9; ++i)
                    {
                        if (!to_double(cov_tokens[i], cov3[i]))
                        {
                            ROS_WARN_STREAM("Malformed wrz cov number: " << raw);
                            cov3.clear();
                            break;
                        }
                    }
                    if (cov3.empty())
                    {
                        ros::spinOnce();
                        continue;
                    }

                    // Build 6x6 covariance (row-major)
                    std::array<double, 36> cov6{};
                    cov6.fill(0.0);
                    // rows 0..2, cols 0..2
                    cov6[0] = cov3[0];
                    cov6[1] = cov3[1];
                    cov6[2] = cov3[2];
                    cov6[6] = cov3[3];
                    cov6[7] = cov3[4];
                    cov6[8] = cov3[5];
                    cov6[12] = cov3[6];
                    cov6[13] = cov3[7];
                    cov6[14] = cov3[8];
                    // angular velocity variance unknown -> huge on diagonals
                    cov6[21] = big_var_ang_; // (3,3)
                    cov6[28] = big_var_ang_; // (4,4)
                    cov6[35] = big_var_ang_; // (5,5)

                    geometry_msgs::TwistWithCovarianceStamped msg;
                    msg.header.stamp = stamp;
                    msg.header.frame_id = frame_id_;
                    // FRD -> ENU
                    msg.twist.twist.linear.x = vx;
                    msg.twist.twist.linear.y = -vy;
                    msg.twist.twist.linear.z = -vz;
                    // Copy cov
                    for (size_t i = 0; i < 36; ++i)
                        msg.twist.covariance[i] = cov6[i];

                    pub_twist_.publish(msg);
                    ros::spinOnce();
                    continue;
                }

                if (publish_dr_pose_ && starts_with(raw, "wrp"))
                {
                    // wrp,[tstamp],[x],[y],[z],[pos_std],[roll],[pitch],[yaw],...
                    auto parts = split(raw, ',');
                    if (parts.size() < 9)
                    {
                        ROS_WARN_STREAM("Malformed wrp (too few fields): " << raw);
                        ros::spinOnce();
                        continue;
                    }

                    double x = 0, y = 0, z = 0, pos_std = 0, roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
                    if (!to_double(parts[2], x) || !to_double(parts[3], y) || !to_double(parts[4], z) ||
                        !to_double(parts[5], pos_std) ||
                        !to_double(parts[6], roll_deg) || !to_double(parts[7], pitch_deg) || !to_double(parts[8], yaw_deg))
                    {
                        ROS_WARN_STREAM("Malformed wrp fields: " << raw);
                        ros::spinOnce();
                        continue;
                    }

                    geometry_msgs::PoseWithCovarianceStamped pose;
                    pose.header.stamp = stamp;
                    pose.header.frame_id = frame_id_;

                    // FRD (x,y,z with z down) -> ENU (z up)
                    pose.pose.pose.position.x = x;
                    pose.pose.pose.position.y = -y;
                    pose.pose.pose.position.z = -z;

                    // Orientation: roll, -pitch, -yaw (deg -> rad)
                    double rr = roll_deg * RAD_PER_DEG;
                    double pr = -pitch_deg * RAD_PER_DEG;
                    double yr = -yaw_deg * RAD_PER_DEG;

                    tf2::Quaternion q;
                    q.setRPY(rr, pr, yr);
                    pose.pose.pose.orientation.x = q.x();
                    pose.pose.pose.orientation.y = q.y();
                    pose.pose.pose.orientation.z = q.z();
                    pose.pose.pose.orientation.w = q.w();

                    double pos_var = pos_std * pos_std;
                    for (double &c : pose.pose.covariance)
                        c = 0.0;
                    pose.pose.covariance[0] = pos_var;  // xx
                    pose.pose.covariance[7] = pos_var;  // yy
                    pose.pose.covariance[14] = pos_var; // zz
                    pose.pose.covariance[21] = big_var_ori_;
                    pose.pose.covariance[28] = big_var_ori_;
                    pose.pose.covariance[35] = big_var_ori_;

                    pub_pose_.publish(pose);
                    ros::spinOnce();
                    continue;
                }

                // Unhandled line type -> ignore
                ros::spinOnce();
            }
        }

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher pub_twist_, pub_pose_;

        // Params
        std::string port_name_;
        int baudrate_{115200};
        double fom_threshold_{25.0};
        std::string frame_id_{"dvl"};
        double big_var_ang_{1e6};
        double big_var_ori_{1e6};
        bool publish_dr_pose_{false};

        // Serial
        io_service io_;
        serial_port port_;
    };

} // namespace wl

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waterlinked_driver");
    wl::Driver drv;
    if (!drv.open())
        return 1;
    drv.spin();
    return 0;
}
