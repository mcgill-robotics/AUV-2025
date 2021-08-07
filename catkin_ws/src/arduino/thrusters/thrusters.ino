#include "thrusters.h"

int thrusterBoundCheck(int input) {
  if (abs(input) > THRUSTER_OUTPUT_BOUND) {
    nh.logerror("Thrusters command out of bound!");
    return max(-THRUSTER_OUTPUT_BOUND, min(THRUSTER_OUTPUT_BOUND, input));
  }
  return input;
}

int thrusterSlopeCheck(int last_input, int new_input) {
  int output = max(last_input - THRUSTER_MAX_INCREMENT, new_input);
  output = min(last_input + THRUSTER_MAX_INCREMENT, new_input);
  return output;
}

void thrustersCallback(const auv_msgs::ThrusterCommands& msg) {
  if (mission_enabled) {
    thrusters_reset_schedule = millis() + THRUSTER_TIMEOUT;

    for (uint8_t i = 0; i < THRUSTER_COUNT; i++) {
      int16_t value = thrusterBoundCheck(msg.thruster_commands[i]);
      value = thrusterSlopeCheck(last_thruster_commands[i], value);
      thrusters[i].writeMicroseconds(THRUSTER_RST_VALUE + value);

      last_thruster_commands[i] = value;
    }
  } else {
    nh.logwarn("Mission off, thrusters command IGNORED!!");
  }
}

void vacuumCallback(const std_msgs::Int16& msg) {
  if (mission_enabled) {
    vacuum_reset_schedule = millis() + THRUSTER_TIMEOUT;

    int16_t value = thrusterBoundCheck(msg.data);
    value = thrusterSlopeCheck(last_vacuum_command, value);
    vacuum.writeMicroseconds(THRUSTER_RST_VALUE + value);
    last_vacuum_command = value;
  } else {
    nh.logwarn("Mission off, vacuum command IGNORED!!");
  }
}

void resetThrusters() {
  for (uint8_t i = 0; i < THRUSTER_COUNT; i++) {
    last_thruster_commands[i] = 0;
    thrusters[i].writeMicroseconds(THRUSTER_RST_VALUE);
  }
  nh.loginfo("Thrusters reset!!");
}

void thrustersInit() {
  for (uint8_t i = 0; i < THRUSTER_COUNT; i++) {
    thrusters[i].attach(thruster_pins[i]);
  }
  resetThrusters();
}

void resetVacuum() {
  last_vacuum_command = 0;
  vacuum.writeMicroseconds(THRUSTER_RST_VALUE);
  nh.loginfo("Vacuum reset!!");
}

void vacuumInit() {
  vacuum.attach(vacuum_pin);
  resetVacuum();
}

void gpioInit() {
//  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(MISSION_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
}

void rosInit() {
  nh.initNode();

  nh.advertise(voltage_pub);
  nh.advertise(mission_pub);
  nh.subscribe(vacuum_sub);
  nh.subscribe(thrusters_sub);
}

void setup() {
  thrustersInit();
  vacuumInit();
  gpioInit();
  rosInit();
  if (MCUSR & MCUSR_WDRF) {
    MCUSR ^= MCUSR_WDRF;
    nh.logerror("Watchdog reset occurred!");
    nh.spinOnce();
  }
  wdt_enable(WDTO_1S);
}

void voltageReportTask(unsigned long time_now) {
  if (voltage_report_schedule < time_now) {
    voltage_m.data = analogRead(VOLTAGE_PIN) * VOLT_SLOPE + VOLT_OFFSET;
    voltage_pub.publish(&voltage_m);
    voltage_report_schedule += VOLTAGE_REPORT_INTERVAL;
    toggleLed();
  }
}

void thurstersResetTask(unsigned long time_now) {
  if (thrusters_reset_schedule < time_now) {
    if (mission_enabled) {
      nh.logwarn("Thrusters command timeout!");
    }
    resetThrusters();
    thrusters_reset_schedule = time_now + TRRUSTER_RESET_INTERVAL;
    toggleLed();
  }
}

void vacuumResetTask(unsigned long time_now) {
  if (vacuum_reset_schedule < time_now) {
    if (mission_enabled) {
      nh.logwarn("Vacuum command timeout!");
    }
    resetVacuum();
    vacuum_reset_schedule = time_now + TRRUSTER_RESET_INTERVAL;
    toggleLed();
  }
}

void missionReportTask(unsigned long time_now) {
  if (misssion_report_schedule < time_now) {
    mission_m.data = mission_enabled;
    mission_pub.publish(&mission_m);
    misssion_report_schedule += MISSION_REPORT_INTERVAL;
    toggleLed();
  }
}
void loop() {
  wdt_reset();
  unsigned long time_now = millis();
  mission_enabled = digitalRead(MISSION_PIN);

  thurstersResetTask(time_now);
  vacuumResetTask(time_now);
  missionReportTask(time_now);
  voltageReportTask(time_now);

  nh.spinOnce();
}
