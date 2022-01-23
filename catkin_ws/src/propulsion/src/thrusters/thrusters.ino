#include <ros.h>
#include <Servo.h>
#include <propulsion/ThrusterCommand.h>

#define SRG_P_PIN 	4
#define SRG_S_PIN	5
#define SWY_BW_PIN 	6
#define SWY_ST_PIN 	7
#define HVE_BW_P_PIN 	8
#define HVE_BW_S_PIN 	9
#define HVE_ST_S_PIN 	10
#define HVE_ST_P_PIN 	11

/* less verbose identifiers */
const uint8_t SRG_P 	= propulsion::ThrusterCommand::SURGE_PORT
const uint8_t SRG_S 	= propulsion::ThrusterCommand::SURGE_STAR
const uint8_t SWY_BW 	= propulsion::ThrusterCommand::SWAY_BOW
const uint8_t SWY_ST 	= propulsion::ThrusterCommand::SWAY_STERN
const uint8_t HVE_BW_P 	= propulsion::ThrusterCommand::HEAVE_BOW_PORT
const uint8_t HVE_BW_S 	= propulsion::ThrusterCommand::HEAVE_BOW_STAR
const uint8_t HVE_ST_S 	= propulsion::ThrusterCommand::HEAVE_STERN_STAR
const uint8_t HVE_ST_P 	= propulsion::ThrusterCommand::HEAVE_STERN_PORT

Servo thrusters[] = new Servo[8];

ros::NodeHandle nh;
ros::Subscriber<propulsion::ThrusterCommand> 
	sub("propulsion/thruster_cmd", &commandCallback);


void commandCb(propulsion::ThrusterCommand command){
	thrusters[SRG_P].writeMicroSeconds(command[SRG_P_PIN]);
	thrusters[SRG_S].writeMicroSeconds(command[SRG_S_PIN]);
	thrusters[SWY_BW].writeMicroSeconds(command[SWY_BW_PIN]);
	thrusters[SWY_ST].writeMicroSeconds(command[SWY_ST_PIN]);
	thrusters[HVE_BW_P].writeMicroSeconds(command[HVE_BW_P_PIN]);
	thrusters[HVE_BW_S].writeMicroSeconds(command[HVE_BW_S_PIN]);
	thrusters[HVE_ST_P].writeMicroSeconds(command[HVE_ST_P_PIN]);
	thrusters[HVE_ST_S].writeMicroSeconds(command[HVE_ST_S_PIN]);
}


void initThrusters(){
	thrusters[SRG_P].attach(SRG_P_PIN)
	thrusters[SRG_S].attach(SRG_S_PIN)
	thrusters[SWY_BW].attach(SWY_BW_PIN)
	thrusters[SWY_ST].attach(SWY_ST_PIN)
	thrusters[HVE_BW_P].attach(HVE_BW_P_PIN)
	thrusters[HVE_BW_S].attach(HVE_BW_S_PIN)
	thrusters[HVE_ST_S].attach(HVE_ST_S_PIN)
	thrusters[HVE_ST_P].attach(HVE_ST_P_PIN)
	//TODO - set initial thruster effort (OFF)
}

void setup() {
	initThrusters();
	nh.initNode();
}

void loop() {
	// listen for commands
	nh.spinOnce(); 
}
