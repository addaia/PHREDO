// imports (provided in folder)
#include "LineSensors.h"
#include "Magnetometer.h"
#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "PID.h"
#include <math.h>

Motors_c motors;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
Kinematics_c pose;
PID_c left_pid;



void testAdvance(float targetDistance, unsigned long updateInterval) {
  unsigned long lastUpdateTime = millis();
  motors.setPWM(40, 40);


  while (sqrt(pow(pose.x, 2) + pow(pose.y, 2)) < targetDistance) {
    if (millis() - lastUpdateTime >= updateInterval) {
      pose.update();
      
      
      // Update the PID controller with demand and current measurement of total counts
      float l_pwm = left_pid.update( pose.last_e0, pose.last_e1 );
    
      // Adjust speed of left motor
      motors.setPWM( l_pwm + 40, 40 );
      
    
      // Update the time for the next pose update.
      lastUpdateTime = millis();
    }
      
  }



  motors.setPWM(0, 0);
  Serial.println("Test complete. Target distance reached.");
}

void setup() {
  Serial.begin(9600);
  // Initialise components as before.
  motors.initialise();
  left_pid.initialise( -1, -0.01, 0.0 ); // Negative because counts are negative
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);
  testAdvance(200.0, 0.5);
}

void loop() {
}
