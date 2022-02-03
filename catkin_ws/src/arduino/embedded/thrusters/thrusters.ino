#include <Arduino.h>
#include <ros.h>
#include <auv_msgs/ThrusterCommands.h>
#include <Servo.h>


#define SERVO_RESET_VALUE 1500
#define THRUSTER_RESET_VALUE 1500

#define THRUSTER_COUNT 4
#define SERVO_COUNT 4

/*[STAR, STERN, PORT, BOW]*/
const uint8_t thruster_esc_pins[THRUSTER_COUNT] = {1, 3, 5, 7};
const uint8_t servo_pins[SERVO_COUNT] = {2, 4, 6, 8};

/*
 * Servo library is used to send PWM signal to ESC/Servo.
 * An array of Servo objects corresponding to
 * [STAR, STERN, PORT, BOW] thrusters/servos respectively
 */
Servo thruster_esc[THRUSTER_COUNT]; //T200 motors (thrusters)
Servo servos[SERVO_COUNT]; //HS-311 servo motors


/*
 * Attaches corresponding pin (and sets pinMode OUTPUT) defined in servo_pins and thruster_esc_pins
 * to each Servo object in servos and thruster_esc respectively
 * Brings all servos back to unrotated position
 * Brings all thrusters to stop
 */
 void initialize(){
    for(uint8_t i = 0; i < SERVO_COUNT; i++){
        servos[i].attach(servo_pins[i]);
        servos[i].writeMicroseconds(SERVO_RESET_VALUE);
    }
    for(uint8_t i = 0; i < THRUSTER_COUNT; i++){
        thruster_esc[i].attach(thruster_esc_pins[i]);
        thruster_esc[i].writeMicroseconds(THRUSTER_RESET_VALUE);
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

/*
 * Sweeps servos/thrusters to test code is correctly uploaded
 * this is not mission critical code, it should not be called during 
 * mission operation
 */
void smoketest(){
    while(true){

         // forwards
        for(uint8_t i = 0; i < THRUSTER_COUNT; i++){
            thruster_esc[i].write(0);
        }
        for(uint8_t i = 0; i < SERVO_COUNT; i++){
            servos[i].write(0);
        }
        delay(5000);

        // backwards
        for(uint8_t i = 0; i < THRUSTER_COUNT; i++){
            thruster_esc[i].write(180);
        }
        for(uint8_t i = 0; i < SERVO_COUNT; i++){
            servos[i].write(180);
        }
        delay(5000);
    }
   
}


ros::NodeHandle nodeHandle;
ros::Subscriber<auv_msgs::ThrusterCommands> thrusterCommandsSub("servo_pos", &thrusterCommandsMsgCallback );

void setup(){
    initialize();
    nodeHandle.initNode();
    nodeHandle.subscribe(thrusterCommandsSub);
}

void loop(){
    // smoketest();
    nodeHandle.spinOnce();
    delay(1);
}