#include <ros.h>
#include <Servo.h>
#include <auv_msgs/ThrusterIntensities.h>
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

/* thrusters operate in range 1500 +/- 500 */
#define STOP 1500
#define MAX_DELTA 500

/* less verbose identifiers
	Pin numbers [0-7] from ThusterCommand.msg
 */
const uint8_t SRG_P 	= auv_msgs::ThrusterIntensities::SURGE_PORT;
const uint8_t SRG_S 	= auv_msgs::ThrusterIntensities::SURGE_STAR;
const uint8_t SWY_BW 	= auv_msgs::ThrusterIntensities::SWAY_BOW;
const uint8_t SWY_ST 	= auv_msgs::ThrusterIntensities::SWAY_STERN;
const uint8_t HVE_BW_P 	= auv_msgs::ThrusterIntensities::HEAVE_BOW_PORT;
const uint8_t HVE_BW_S 	= auv_msgs::ThrusterIntensities::HEAVE_BOW_STAR;
const uint8_t HVE_ST_S 	= auv_msgs::ThrusterIntensities::HEAVE_STERN_STAR;
const uint8_t HVE_ST_P 	= auv_msgs::ThrusterIntensities::HEAVE_STERN_PORT;

Servo thrusters[8];
const float offCommand[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

uint16_t* convertToMicroseconds(const float intensities[8]){
	double intensity; 
	static uint16_t res[8];

	for(uint8_t i = 0; i < 8; i++){
		intensity = intensities[i];
		if(intensity < -1){
			intensity = -1;
		} else if(intensity > 1){
			intensity = 1;	
		}
		res[i] = static_cast<int>(STOP + intensity*MAX_DELTA);
	}
	return res;
}

void updateThrusters(const float intensities[8]){
	uint16_t* command = convertToMicroseconds(intensities);

	thrusters[SRG_P].writeMicroseconds(command[SRG_P]);
	thrusters[SRG_S].writeMicroseconds(command[SRG_S]);
	thrusters[SWY_BW].writeMicroseconds(command[SWY_BW]);
	thrusters[SWY_ST].writeMicroseconds(command[SWY_ST]);
	thrusters[HVE_BW_P].writeMicroseconds(command[HVE_BW_P]);
	thrusters[HVE_BW_S].writeMicroseconds(command[HVE_BW_S]);
	thrusters[HVE_ST_P].writeMicroseconds(command[HVE_ST_P]);
	thrusters[HVE_ST_S].writeMicroseconds(command[HVE_ST_S]);
}

void thrustersOff(){
	updateThrusters(offCommand);
}

void commandCb(const auv_msgs::ThrusterIntensities& tc){
	const float* intensities = tc.intensities;
	updateThrusters(intensities);
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
	//thrustersOff();
}


ros::NodeHandle nh;
ros::Subscriber<auv_msgs::ThrusterIntensities> sub("propulsion/thruster_intensities", &commandCb);

void setup() {
	initThrusters();
	nh.subscribe(sub);
	nh.initNode();
	
}

void loop() {
	// listen for commands
	nh.spinOnce(); 
}
