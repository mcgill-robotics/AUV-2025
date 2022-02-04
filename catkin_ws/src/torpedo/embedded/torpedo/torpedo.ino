#include <ros.h>
#include <std_msgs/Bool.h>

#define ENABLE_SWITCH 3

void setup(){
	pinMode(ENABLE_SWITCH, INPUT);
	digitalWrite(ENABLE_SWITCH, LOW);
}

void commandCb(const std_msgs::Bool& enable){
	if(enable.data){
		digitalWrite(ENABLE_SWITCH, HIGH);
	}
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> sub("torpedo_enable", &commandCb);

void loop(){
	// listen for commands
	nh.spinOnce(); 
}
