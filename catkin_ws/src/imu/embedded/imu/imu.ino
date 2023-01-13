#include "Quaternion.h"
#include "XimuReceiver.h"
#include <ros.h>
#include <auv_msgs/ImuData.h>


ros::NodeHandle nh;

auv_msgs::ImuData data_msg;


XimuReceiver ximuReceiver;

ros::Publisher pub("imu_data", &data_msg);

void setup() {
	nh.initNode();
	nh.advertise(pub);
	nh.spinOnce();
	Serial1.begin(115200);
}

void loop() {
	delay(1000);
	ErrorCode e = ERR_NO_ERROR;
	/*data_msg.ROLL = 5;
	data_msg.PITCH = 10;
	data_msg.YAW = 15;
	pub.publish(&data_msg);*/
	if(ximuReceiver.isQuaternionGetReady()) {
		QuaternionStruct quaternionStruct = ximuReceiver.getQuaternion();
		Quaternion quaternion = Quaternion(quaternionStruct.w, quaternionStruct.x, quaternionStruct.y, quaternionStruct.z);
		EulerAnglesStruct eulerAnglesStruct = quaternion.getEulerAngles();
		data_msg.ROLL = eulerAnglesStruct.roll;
		data_msg.PITCH = eulerAnglesStruct.pitch;
		data_msg.YAW = eulerAnglesStruct.yaw;
		pub.publish(&data_msg);
	}
	nh.spinOnce();
}
