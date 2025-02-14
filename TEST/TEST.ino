// imports (provided in folder)
#include "LineSensors.h"
#include "Magnetometer.h"
#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include <math.h>

Motors_c motors;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
Kinematics_c pose;


void testAdvance(float targetDistance, unsigned long updateInterval) {
  unsigned long lastUpdateTime = millis();
  motors.setPWM(40, 40);


  while (sqrt(pow(pose.x, 2) + pow(pose.y, 2)) < targetDistance) {
    if (millis() - lastUpdateTime >= updateInterval) {
      pose.update(); 
      

      
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
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);
  testAdvance(200.0, 0.5);
}

void loop() {
}
