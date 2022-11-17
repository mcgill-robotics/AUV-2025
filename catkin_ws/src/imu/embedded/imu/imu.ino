/*
    x_IMU_Arduino_Example.ino
    Author: Seb Madgwick

    Example usage of x-IMU C++ library.  Also uses the quaternion library to
    convert the received quaternion to Euler angles.

    Requires two hardware serial modules: one to receive from the x-IMU and one
    to transmit the decoded data to be displayed on computer.

    x-IMU settings:
    Auxiliary Port > Auxiliary Port Mode: "UART"
    Auxiliary Port > UART > Baud Rate: 115200
    Auxiliary Port > UART > Hardware Flow Control: Off

    Hardware connections:
    x-IMU GND -> Arduino MEGA GND
    x-IMU EXT -> Arduino MEGA 5V
    x-IMU AX2 -> Arduino MEGA RX1

    Tested with "arduino-1.0.3" and "Arduino MEGA".
*/

//------------------------------------------------------------------------------
// Includes

#include "Quaternion.h"
#include "XimuReceiver.h"
#include <ros.h>
#include "std_msgs/Float64.h"


/*ros::NodeHandle x_nh;
ros::NodeHandle y_nh;
ros::NodeHandle z_nh;*/
ros::NodeHandle theta_x_nh;
ros::NodeHandle theta_y_nh;
ros::NodeHandle theta_z_nh;

/*std_msgs::Float64 x_msg;
std_msgs::Float64 y_msg;
std_msgs::Float64 z_msg;*/
std_msgs::Float64 theta_x_msg;
std_msgs::Float64 theta_y_msg;
std_msgs::Float64 theta_z_msg;

/*
ros::Publisher x_pub("state_x", &x_msg);
ros::Publisher y_pub("state_y", &y_msg);
ros::Publisher z_pub("state_z", &z_msg);*/
ros::Publisher theta_x_pub("imu_theta_x", &theta_x_msg);
ros::Publisher theta_y_pub("imu_theta_y", &theta_y_msg);
ros::Publisher theta_z_pub("imu_theta_z", &theta_z_msg);


//------------------------------------------------------------------------------
// Variables

XimuReceiver ximuReceiver;

//------------------------------------------------------------------------------
// Functions



void setup() {

    theta_x_nh.initNode();
    theta_y_nh.initNode();
    theta_z_nh.initNode();

    theta_x_nh.advertise(theta_x_pub);
    theta_y_nh.advertise(theta_y_pub);
    theta_z_nh.advertise(theta_z_pub);

    Serial.begin(115200);   // for sending data to computer
    Serial1.begin(115200);  // for receiving data from x-IMU
}

void loop() {
    ErrorCode e = ERR_NO_ERROR;

    // Process recieved data
    while(Serial1.available() > 0) {
        e = ximuReceiver.processNewChar(Serial1.read());
    }

    // Print error code (receive error)
    if(e != ERR_NO_ERROR) {
        Serial.print("ERROR: ");
        Serial.print(e);
        Serial.print("\r");
    }

    // Print battery and thermometer data
    if(ximuReceiver.isBattAndThermGetReady()) {
        BattAndThermStruct battAndThermStruct = ximuReceiver.getBattAndTherm();
        Serial.print("battery = ");
        Serial.print(battAndThermStruct.battery);
        Serial.print(", thermometer = ");
        Serial.print(battAndThermStruct.thermometer);
        Serial.print("\r");
    }

    // Print sensor data
    if(ximuReceiver.isInertialAndMagGetReady()) {
        InertialAndMagStruct inertialAndMagStruct = ximuReceiver.getInertialAndMag();
        Serial.print("gyrX = ");
        Serial.print(inertialAndMagStruct.gyrX);
        Serial.print(", gyrY = ");
        Serial.print(inertialAndMagStruct.gyrY);
        Serial.print(", gyrZ = ");
        Serial.print(inertialAndMagStruct.gyrZ);
        Serial.print(", accX = ");
        Serial.print(inertialAndMagStruct.accX);
        Serial.print(", accY = ");
        Serial.print(inertialAndMagStruct.accY);
        Serial.print(", accZ = ");
        Serial.print(inertialAndMagStruct.accZ);
        Serial.print(", magX = ");
        Serial.print(inertialAndMagStruct.magX);
        Serial.print(", magY = ");
        Serial.print(inertialAndMagStruct.magY);
        Serial.print(", magZ = ");
        Serial.print(inertialAndMagStruct.magZ);
        Serial.print("\r");
    }

    theta_x_msg.data = inertialAndMagStruct.gyrX;
    theta_y_msg.data = inertialAndMagStruct.gyrY;
    theta_z_msg.data = inertialAndMagStruct.gyrZ;

    theta_x_pub.publish(&theta_x_msg);
    theta_y_pub.publish(&theta_y_msg);
    theta_z_pub.publish(&theta_z_msg);


    // Print quaternion data as Euler angles
    if(ximuReceiver.isQuaternionGetReady()) {
        QuaternionStruct quaternionStruct = ximuReceiver.getQuaternion();
        Quaternion quaternion = Quaternion(quaternionStruct.w, quaternionStruct.x, quaternionStruct.y, quaternionStruct.z);
        EulerAnglesStruct eulerAnglesStruct = quaternion.getEulerAngles();
        Serial.print("roll = ");
        Serial.print(eulerAnglesStruct.roll);
        Serial.print(", pitch = ");
        Serial.print(eulerAnglesStruct.pitch);
        Serial.print(", yaw = ");
        Serial.print(eulerAnglesStruct.yaw);
        Serial.print("\r");
    }
}

//------------------------------------------------------------------------------
// End of file
