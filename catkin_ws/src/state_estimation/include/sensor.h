#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>

class Sensor {
    public:
        tf2::Pose* pose_nwu_auv;
        tf2::Vector3* angular_velocity;
        char[] sensor_name;
        bool is_active(void);
    private:
        ros::Time last_unique_state_time;
        ros::Time last_error_message_time;
        ros::Time time_before_considered_inactive;
        tf2::Vector3 prev_pos_nwu_auv;
        tf2::Quaternion prev_q_nwu_auv;
        tf2::Vector3 prev_angular_velocity;
        void update_last_state(void);
        virtual has_different_data(void);
        virtual bool has_valid_data(void);
        tf2_ros::Buffer* tfBuffer;
        tf2_ros::TransformListener* tfListener;
};