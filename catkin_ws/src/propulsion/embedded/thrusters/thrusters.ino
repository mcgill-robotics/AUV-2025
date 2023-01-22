#include <ros.h>
#include <Servo.h>
#include <auv_msgs/ThrusterMicroseconds.h>
/* 
NOTE: pins 2-9 were chosen instead of 0-7, since 
pins 0 and 1 function differently.

*/
#define SRG_P_PIN 	2
#define SRG_S_PIN	3
#define SWY_BW_PIN 	4
#define SWY_ST_PIN 	5
#define HVE_BW_P_PIN 	6
#define HVE_BW_S_PIN 	7
#define HVE_ST_S_PIN 	8
#define HVE_ST_P_PIN 	9

/* less verbose identifiers
	Pin numbers [0-7] from ThusterCommand.msg
 */
const uint8_t SRG_P 	= auv_msgs::ThrusterMicroseconds::SURGE_PORT;
const uint8_t SRG_S 	= auv_msgs::ThrusterMicroseconds::SURGE_STAR;
const uint8_t SWY_BW 	= auv_msgs::ThrusterMicroseconds::SWAY_BOW;
const uint8_t SWY_ST 	= auv_msgs::ThrusterMicroseconds::SWAY_STERN;
const uint8_t HVE_BW_P 	= auv_msgs::ThrusterMicroseconds::HEAVE_BOW_PORT;
const uint8_t HVE_BW_S 	= auv_msgs::ThrusterMicroseconds::HEAVE_BOW_STAR;
const uint8_t HVE_ST_S 	= auv_msgs::ThrusterMicroseconds::HEAVE_STERN_STAR;
const uint8_t HVE_ST_P 	= auv_msgs::ThrusterMicroseconds::HEAVE_STERN_PORT;

Servo thrusters[8];
const uint16_t offCommand[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; 

unsigned long lastCommandTime = millis();
unsigned long currentTime = millis();

void updateThrusters(const uint16_t microseconds[8]){
    unsigned long lastCommandTime = millis();
	thrusters[SRG_P].writeMicroseconds(microseconds[SRG_P]);
	thrusters[SRG_S].writeMicroseconds(microseconds[SRG_S]);
	thrusters[SWY_BW].writeMicroseconds(microseconds[SWY_BW]);
	thrusters[SWY_ST].writeMicroseconds(microseconds[SWY_ST]);
	thrusters[HVE_BW_P].writeMicroseconds(microseconds[HVE_BW_P]);
	thrusters[HVE_BW_S].writeMicroseconds(microseconds[HVE_BW_S]);
	thrusters[HVE_ST_P].writeMicroseconds(microseconds[HVE_ST_P]);
	thrusters[HVE_ST_S].writeMicroseconds(microseconds[HVE_ST_S]);
}

void thrustersOff(){
	updateThrusters(offCommand);
}

void commandCb(const auv_msgs::ThrusterMicroseconds& tc){
	const uint16_t* microseconds = tc.microseconds;
	updateThrusters(microseconds);
}

void initThrusters(){
	thrusters[SRG_P].attach(SRG_P_PIN);
	thrusters[SRG_S].attach(SRG_S_PIN);
	thrusters[SWY_BW].attach(SWY_BW_PIN);
	thrusters[SWY_ST].attach(SWY_ST_PIN);
	thrusters[HVE_BW_P].attach(HVE_BW_P_PIN);
	thrusters[HVE_BW_S].attach(HVE_BW_S_PIN);
	thrusters[HVE_ST_S].attach(HVE_ST_S_PIN);
	thrusters[HVE_ST_P].attach(HVE_ST_P_PIN);

	//set initial thruster effort (OFF)
	thrustersOff();
}


ros::NodeHandle nh;
ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("propulsion/thruster_microseconds", &commandCb);

void setup() {
	initThrusters();
	nh.subscribe(sub);
	nh.initNode();
	
}

void loop() {
    // turn of thrusters if no command in 2s
    if(currentTime - lastCommandTime > 1000)
        thrustersOff();

	// listen for commands
	nh.spinOnce(); 
}
