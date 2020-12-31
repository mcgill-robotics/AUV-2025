#include <Arduino.h>
#include <auv_msgs/ThrusterCommands.h>
#include <ros.h>
#include <Servo.h>


#define SERVO_RESET_VALUE 1500
#define THRUSTER_RESET_VALUE 1500

#define THRUSTER_COUNT 4
#define SERVO_COUNT 4

/*[STAR, STERN, PORT, BOW]*/
const uint8_t thruster_esc_pins[THRUSTER_COUNT] = {1, 3, 5, 7}
const uint8_t servo_pins[SERVO_COUNT] = {2, 4, 6, 8};

/*
 * Servo library is used to send PWM signal to ESC/Servo.
 * An array of Servo objects corresponding to
 * [STAR, STERN, PORT, BOW] thrusters/servos respectively
 */
Servo thruster_esc[THRUSTER_COUNT]; //T200 motors (thrusters)
Servo servos[SERVO_COUNT]; //HS-311 servo motors


/**
 * Bring all servos back to unrotated position
 */
void resetServos(){
    for(uint8_t i = 0; i < SERVO_COUNT; i++){
        servos[i].writeMicroseconds(SERVO_RESET_VALUE);
    }
}
/*
 *  Bring all thrusters to a stop
 */
void resetThrusters(){
  for(uint8_t i = 0; i< THRUSTER_COUNT; i++){
    thrusters_esc[i].writeMicroseconds(THRUSTER_RESET_VALUE);
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
/**
 * Attaches corresponding pin (and sets pinMode OUTPUT) defined in thruster_esc_pins
 * to each Servo object in thruster_esc
 */


void thrustersInit(){
   for(uint8_t i = 0; i < THRUSTER_COUNT; i++){
        thruster_esc[i].attach(thruster_esc_pins[i]);
    }
}


void thrusterCommandsMsgCallback( const auv_msgs::ThrusterCommands& msg){
    for(uint8_t i = 0; i < SERVO_COUNT; i++){
        int16_t servo_command = msg.servo_commands[i];
        servos[i].writeMicroseconds(SERVO_RESET_VALUE + servo_command);
    }
    for(uint8_t i = 0; i < THRUSTER_COUNT; i++){
        int16_t thruster_command = msg.motor_commands[i];
        thruster_esc[i].writeMicroseconds(THRUSTER_RESET_VALUE + thruster_command);
    }

}


ros::NodeHandle nodeHandle;
ros::Subscriber<auv_msgs::ThrusterCommands> thrusterCommandsSub("servo_pos", &thrusterCommandsMsgCallback );

void setup(){
    servosInit();
    nodeHandle.initNode();
    nodeHandle.subscribe(thrusterCommandsSub);
}

void loop(){
    nodeHandle.spinOnce();
    delay(1);
}

/* create custom message to pass PWM to arduino -> [1444, 1300, ...]

topic: servo_pos

# in the .ino we want to read the published messages from the topic, get the PWM
# value and set the servo control pin to output a signal with that pulse width
*/
