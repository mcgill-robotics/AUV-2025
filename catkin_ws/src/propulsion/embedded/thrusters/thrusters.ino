#include <Servo.h>
#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>

// defines all MCU pins
#define SRG_P_PIN 	2
#define SRG_S_PIN	3
#define SWY_BW_PIN 	4
#define SWY_ST_PIN 	5
#define HVE_BW_P_PIN 	6
#define HVE_BW_S_PIN 	7
#define HVE_ST_S_PIN 	8
#define HVE_ST_P_PIN 	9

#define MCU_KS 10
#define WATER_DETECTED 12

#define TEENSY_LED 13

#define TC_1 14
#define TC_2 15
#define TC_3 16
#define TC_4 17
#define TC_5 18
#define TC_6 19
#define TC_7 20
#define TC_8 21

#define VBAT1_SENSE 22
#define VBAT2_SENSE 23

// defines 8 thursters from ROS for initialization in an array
const uint8_t SRG_P 	= auv_msgs::ThrusterMicroseconds::SURGE_PORT;
const uint8_t SRG_S 	= auv_msgs::ThrusterMicroseconds::SURGE_STAR;
const uint8_t SWY_BW 	= auv_msgs::ThrusterMicroseconds::SWAY_BOW;
const uint8_t SWY_ST 	= auv_msgs::ThrusterMicroseconds::SWAY_STERN;
const uint8_t HVE_BW_P 	= auv_msgs::ThrusterMicroseconds::HEAVE_BOW_PORT;
const uint8_t HVE_BW_S 	= auv_msgs::ThrusterMicroseconds::HEAVE_BOW_STAR;
const uint8_t HVE_ST_S 	= auv_msgs::ThrusterMicroseconds::HEAVE_STERN_STAR;
const uint8_t HVE_ST_P 	= auv_msgs::ThrusterMicroseconds::HEAVE_STERN_PORT;

// creates array of 8 thrusters
Servo thrusters[8];

// signals to push to thrusters
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
const uint16_t offCommand[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// creates array for 8 thruster current sensing
double Tcurrents[8];

// creates array for 2 battery voltage sensing
float Bvoltages[2];

// updates thrusters' pwm signals from array
void updateThrusters(const uint16_t microseconds[8]) {
	thrusters[SRG_P].writeMicroseconds(microseconds[SRG_P]);
	thrusters[SRG_S].writeMicroseconds(microseconds[SRG_S]);
	thrusters[SWY_BW].writeMicroseconds(microseconds[SWY_BW]);
	thrusters[SWY_ST].writeMicroseconds(microseconds[SWY_ST]);
	thrusters[HVE_BW_P].writeMicroseconds(microseconds[HVE_BW_P]);
	thrusters[HVE_BW_S].writeMicroseconds(microseconds[HVE_BW_S]);
	thrusters[HVE_ST_P].writeMicroseconds(microseconds[HVE_ST_P]);
	thrusters[HVE_ST_S].writeMicroseconds(microseconds[HVE_ST_S]);
}

void commandCb(const auv_msgs::ThrusterMicroseconds& tc){
	memcpy(microseconds, tc.microseconds, 8*sizeof(uint16_t));
}

// attaches and arms thrusters
void initThrusters() {
	thrusters[SRG_P].attach(SRG_P_PIN);
	thrusters[SRG_S].attach(SRG_S_PIN);
	thrusters[SWY_BW].attach(SWY_BW_PIN);
	thrusters[SWY_ST].attach(SWY_ST_PIN);
	thrusters[HVE_BW_P].attach(HVE_BW_P_PIN);
	thrusters[HVE_BW_S].attach(HVE_BW_S_PIN);
	thrusters[HVE_ST_S].attach(HVE_ST_S_PIN);
	thrusters[HVE_ST_P].attach(HVE_ST_P_PIN);

	updateThrusters(offCommand);
}

ros::NodeHandle nh;
ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);

void killSystem() {
	digitalWrite(MCU_KS, HIGH);
	delay(100);
}

void powerSystem() {
	digitalWrite(MCU_KS, LOW);
	delay(100);
}

void waterInterrupt() {
	killSystem();
	while (true) {
		digitalWrite(TEENSY_LED, HIGH);
		delay(500);
		digitalWrite(TEENSY_LED, LOW);
		delay(500);
	}
}

void senseCurrent(double Tcurrents[]) {
	Tcurrents[0] = ((analogRead(TC_1) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[1] = ((analogRead(TC_2) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[2] = ((analogRead(TC_3) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[3] = ((analogRead(TC_4) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[4] = ((analogRead(TC_5) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[5] = ((analogRead(TC_6) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[6] = ((analogRead(TC_7) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[7] = ((analogRead(TC_8) / 1024.0) * 3.3) / (0.005 * 50);
}

void senseVoltage(float Bvoltages[]) {
	Bvoltages[0] = analogRead(VBAT1_SENSE) * (3.3 / 1024) * 1.6625 + 12.5;
	Bvoltages[1] = analogRead(VBAT2_SENSE) * (3.3 / 1024) * 1.6625 + 12.5;
}

void setup() {
	initThrusters();
	
	//pinMode(WATER_DETECTED, INPUT_PULLUP);
	//pinMode(MCU_KS, OUTPUT);
	pinMode(TC_1, INPUT);
	pinMode(TC_2, INPUT);
	pinMode(TC_3, INPUT);
	pinMode(TC_4, INPUT);
	pinMode(TC_5, INPUT);
	pinMode(TC_6, INPUT);
	pinMode(TC_7, INPUT);
	pinMode(TC_8, INPUT);
	pinMode(VBAT1_SENSE, INPUT);
	pinMode(VBAT2_SENSE, INPUT);
	pinMode(TEENSY_LED, OUTPUT);

	//attachInterrupt(digitalPinToInterrupt(WATER_DETECTED), waterInterrupt, RISING);

	nh.subscribe(sub);
	nh.initNode();
}

void loop() {
	updateThrusters(microseconds);
	nh.spinOnce(); 
}