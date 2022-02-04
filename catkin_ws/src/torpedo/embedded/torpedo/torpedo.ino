#include <Arduino.h>
#define ENABLE 3
#define THRUSTER 9
#define PWM_EN 255
#define PWM_OFF 0

void setup(){
	pinMode(ENABLE, INPUT);
	pinMode(THRUSTER, OUTPUT);
	digitalWrite(THRUSTER, LOW);
}

void loop(){
	if (digitalRead(ENABLE)){
		analogWrite(THRUSTER, PWM_EN);
	}
	else{
		analogWrite(THRUSTER, PWM_OFF);
	}
}
