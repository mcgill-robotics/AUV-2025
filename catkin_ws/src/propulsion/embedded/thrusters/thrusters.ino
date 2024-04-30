#include <Servo.h>
#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>
#include <std_msgs/Float32.h>

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

#define VBAT2_SENSE 22
#define VBAT1_SENSE 23

// defines 8 thursters for ROS subscribing
const uint8_t SRG_P 	= auv_msgs::ThrusterMicroseconds::SURGE_PORT;
const uint8_t SRG_S 	= auv_msgs::ThrusterMicroseconds::SURGE_STAR;
const uint8_t SWY_BW 	= auv_msgs::ThrusterMicroseconds::SWAY_BOW;
const uint8_t SWY_ST 	= auv_msgs::ThrusterMicroseconds::SWAY_STERN;
const uint8_t HVE_BW_P 	= auv_msgs::ThrusterMicroseconds::HEAVE_BOW_PORT;
const uint8_t HVE_BW_S 	= auv_msgs::ThrusterMicroseconds::HEAVE_BOW_STAR;
const uint8_t HVE_ST_S 	= auv_msgs::ThrusterMicroseconds::HEAVE_STERN_STAR;
const uint8_t HVE_ST_P 	= auv_msgs::ThrusterMicroseconds::HEAVE_STERN_PORT;

// defines 2 battery voltage sensing and 8 thruster current sensing messages for ROS advertising
std_msgs::Float32 batt1_voltage_msg;
std_msgs::Float32 batt2_voltage_msg;
std_msgs::Float32 thrust1_current_msg;
std_msgs::Float32 thrust2_current_msg;
std_msgs::Float32 thrust3_current_msg;
std_msgs::Float32 thrust4_current_msg;
std_msgs::Float32 thrust5_current_msg;
std_msgs::Float32 thrust6_current_msg;
std_msgs::Float32 thrust7_current_msg;
std_msgs::Float32 thrust8_current_msg;

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

// updates microseconds array with values from ros
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

// sets up ros publisher and subscriber nodes
ros::NodeHandle nh;
ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);
ros::Publisher batt1_voltage("batt1_voltage", &batt1_voltage_msg);
ros::Publisher batt2_voltage("batt2_voltage", &batt2_voltage_msg);
ros::Publisher thrust1_current("thrust1_current", &thrust1_current_msg);
ros::Publisher thrust2_current("thrust2_current", &thrust2_current_msg);
ros::Publisher thrust3_current("thrust3_current", &thrust3_current_msg);
ros::Publisher thrust4_current("thrust4_current", &thrust4_current_msg);
ros::Publisher thrust5_current("thrust5_current", &thrust5_current_msg);
ros::Publisher thrust6_current("thrust6_current", &thrust6_current_msg);
ros::Publisher thrust7_current("thrust7_current", &thrust7_current_msg);
ros::Publisher thrust8_current("thrust8_current", &thrust8_current_msg);

// kills system by writing high to kill switch transistor
void killSystem() {
	digitalWrite(MCU_KS, HIGH);
	delay(100);
}

// powers on system by writing low to kill switch transistor
void powerSystem() {
	digitalWrite(MCU_KS, LOW);
	delay(100);
}

// permanently kills system by writing high to kill switch transistor and flashes led light
void waterInterrupt() {
	killSystem();
	while (true) {
		digitalWrite(TEENSY_LED, HIGH);
		delay(500);
		digitalWrite(TEENSY_LED, LOW);
		delay(500);
	}
}

// senses currents of the 8 thrusters
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

// senses the voltages of the 2 batteries
void senseVoltage(float Bvoltages[]) {
	Bvoltages[0] = analogRead(VBAT1_SENSE) * (3.3 / 1024) * 1.6625 + 12.5;
	Bvoltages[1] = analogRead(VBAT2_SENSE) * (3.3 / 1024) * 1.6625 + 12.5;
}

// updates values sensed onto the ros nodes and publishes them
void publishVoltagesAndCurrents() {
	senseCurrent(Tcurrents);
	senseVoltage(Bvoltages);

	batt1_voltage_msg.data = Bvoltages[0];
	batt2_voltage_msg.data = Bvoltages[1];
	thrust1_current_msg.data = Tcurrents[0];
	thrust2_current_msg.data = Tcurrents[1];
	thrust3_current_msg.data = Tcurrents[2];
	thrust4_current_msg.data = Tcurrents[3];
	thrust5_current_msg.data = Tcurrents[4];
	thrust6_current_msg.data = Tcurrents[5];
	thrust7_current_msg.data = Tcurrents[6];
	thrust8_current_msg.data = Tcurrents[7];
	
	batt1_voltage.publish( &batt1_voltage_msg );
	batt2_voltage.publish( &batt2_voltage_msg );
	thrust1_current.publish( &thrust1_current_msg );
	thrust2_current.publish( &thrust2_current_msg );
	thrust3_current.publish( &thrust3_current_msg );
	thrust4_current.publish( &thrust4_current_msg );
	thrust5_current.publish( &thrust5_current_msg );
	thrust6_current.publish( &thrust6_current_msg );
	thrust7_current.publish( &thrust7_current_msg );
	thrust8_current.publish( &thrust8_current_msg );
}

void setup() {
	initThrusters();

	pinMode(MCU_KS, OUTPUT);
	pinMode(TEENSY_LED, OUTPUT);
	// pinMode(WATER_DETECTED, INPUT_PULLUP);
	// attachInterrupt(digitalPinToInterrupt(WATER_DETECTED), waterInterrupt, RISING);
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

	nh.initNode();
	nh.subscribe(sub);
	nh.advertise(batt1_voltage);
	nh.advertise(batt2_voltage);
	nh.advertise(thrust1_current);
	nh.advertise(thrust2_current);
	nh.advertise(thrust3_current);
	nh.advertise(thrust4_current);
	nh.advertise(thrust5_current);
	nh.advertise(thrust6_current);
	nh.advertise(thrust7_current);
	nh.advertise(thrust8_current);
}

void loop() {
	updateThrusters(microseconds);

	publishVoltagesAndCurrents();
	
	nh.spinOnce();
	
	delay(10);
}
