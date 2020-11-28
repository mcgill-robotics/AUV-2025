#include <Arduino.h>
#include <ros.h>
#include <Servo.h> 

//#define THRUSTER_COUNT 4
#define SERVO_COUNT 2

/*[STAR, STERN, PORT, BOW]*/
// const uint8_t thruster_esc_pins[THRUSTER_COUNT] = {}
const uint8_t servo_pins[SERVO_COUNT] = {2, 4};

/*
 * Servo library is used to send PWM signal to ESC/Servo.
 * An array of Servo objects corresponding to 
 * [STAR, STERN, PORT, BOW] thrusters/servos respectively
 */
// Servo thruster_esc[THRUSTER_COUNT]; //T200 motors (thrusters)
Servo servos[SERVO_COUNT]; //HS-311 servo motors

/**
 * Bring all servos back to unrotated position
 */
void resetServos(){
    for(uint8_t i = 0; i < SERVO_COUNT; i++){
        servos[i].writeMicroseconds(1500);
    }
}

/**
 * Attaches corresponding pin (and sets pinMode OUTPUT) defined in servo_pins 
 * to each Servo object in servos
 */
void servosInit(){
    for(uint8_t i = 0; i < SERVO_COUNT; i++){
        servos[i].attach(servo_pins[i]);
    }

}

void setup(){
    servosInit();
}

void loop(){
    servos[0].writeMicroseconds(1000);
    delay(5000);
    servos[1].writeMicroseconds(1900);
    delay(5000);
    resetServos();
    delay(5000);
}
