#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <iostream>

#include "XimuReceiver.h"
#include "XimuPublisher.h"

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <cmath>

using namespace boost;

static unsigned int sequence = 0;
ros::Publisher pub, pub2, pub3, pub4, rawPub;
geometry_msgs::Pose pos;
geometry_msgs::PoseStamped posStamped;
double* q = new double[4];
asio::io_service serial_io;
asio::serial_port port(serial_io);
XimuReceiver receiver;

void multiplyQuaternions(double q[],double p[])
{
    double temp[4];
    temp[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    temp[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    temp[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
    temp[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
    q[0] = temp[0];
    q[1] = temp[1];
    q[2] = temp[2];
    q[3] = temp[3];
}

void spin() {
	try{
		port.open("/dev/imu");
		port.set_option(asio::serial_port_base::baud_rate(115200));
	}
	catch(exception& e) {
		std::cerr << "The IMU is not connected or doesn't have permission to read the serial port\n";
	}

    char c;

    while (true) {
        // Read 1 character into c, this will block
        // forever if no character arrives.
        asio::read(port, asio::buffer(&c,1));
        receiver.processNewChar(c);

        if (receiver.isInertialAndMagGetReady()) {
            InertialAndMagStruct ims = receiver.getInertialAndMag();
            geometry_msgs::Vector3 acc = geometry_msgs::Vector3();
            acc.x = ims.accX;
            acc.y = ims.accY;
            acc.z = ims.accZ;
            pub2.publish(acc);

            geometry_msgs::Vector3 gyro = geometry_msgs::Vector3();
            gyro.x = ims.gyrX;
            gyro.y = ims.gyrY;
            gyro.z = ims.gyrZ;
            pub3.publish(gyro);

            geometry_msgs::Vector3 mag = geometry_msgs::Vector3();
            mag.x = ims.magX;
            mag.y = ims.magY;
            mag.z = ims.magZ;
            pub4.publish(mag);

            // IMU raw data
            sensor_msgs::Imu imu;
            imu.angular_velocity.x = ims.gyrX;
            imu.angular_velocity.y = ims.gyrY;
            imu.angular_velocity.z = ims.gyrZ;
            imu.linear_acceleration.x = ims.accX;
            imu.linear_acceleration.y = ims.accY;
            imu.linear_acceleration.z = ims.accZ;
            imu.header.stamp = ros::Time::now();
            rawPub.publish(imu);
        }

        if (receiver.isQuaternionGetReady()) {
            QuaternionStruct quaternionStruct = receiver.getQuaternion();

            q[0] = quaternionStruct.w;
            q[1] = quaternionStruct.x;
            q[2] = quaternionStruct.y;
            q[3] = quaternionStruct.z;

            pos.orientation.w = q[0];
            pos.orientation.x = q[1];
            pos.orientation.y = q[2];
            pos.orientation.z = q[3];

            posStamped = geometry_msgs::PoseStamped();
            posStamped.pose = pos;
            posStamped.header.seq = sequence++;
            posStamped.header.stamp = ros::Time::now();
            posStamped.header.frame_id = "base_footprint";

            pub.publish(posStamped);
        }
    }
}

int main(int argc, char** argv) {
    printf("Starting XimuPublisher\n");
    ros::init(argc, argv, "x_imu_pose");
    ros::NodeHandle node;

    pub = node.advertise<geometry_msgs::PoseStamped>("state_estimation/pose", 100);
    pub2 = node.advertise<geometry_msgs::Vector3>("state_estimation/acc", 100);
    pub3 = node.advertise<geometry_msgs::Vector3>("state_estimation/gyro", 100);
    pub4 = node.advertise<geometry_msgs::Vector3>("state_estimation/mag", 100);
    // Publish raw data
    rawPub = node.advertise<sensor_msgs::Imu>("state_estimation/raw", 100);

    spin();

    return 0;
}
