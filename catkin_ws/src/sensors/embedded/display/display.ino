#include <ros.h>
#include <auv_msgs/DisplayScreen.h>

ros::NodeHandle nh;

String mission;
float dvl;
float imu;
float depth;



void displayScreenCallback(const auv_msgs::DisplayScreen& msg) {
    // Do something with the message
    mission = msg.mission;
    dvl = msg.dvl;
    imu = msg.imu;
    depth = msg.depth;
}

ros::Subscriber<auv_msgs::DisplayScreen> sub_display_screen("/display_screen", &displayScreenCallback);
ros::Publisher pub("display_screen", &displayMsg);


void setup() {
    nh.initNode();
    nh.subscribe(sub_display_screen);
	nh.advertise(pub);
	nh.spinOnce();
	Serial1.begin(115200);
    
}


void loop() {

}

