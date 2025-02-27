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

# define MAX_ENTRIES 220
unsigned long times[MAX_ENTRIES];
float distances[MAX_ENTRIES];
int entryCount = 0;

void testAdvance(float targetDistance, unsigned long updateInterval) {
  unsigned long startTime = millis();
  unsigned long lastUpdateTime = millis();
  motors.setPWM(200,200);


  while (sqrt(pow(pose.x, 2) + pow(pose.y, 2)) < targetDistance) {
    if (millis() - lastUpdateTime >= updateInterval) {
      pose.update(); 
      
      if (entryCount < MAX_ENTRIES) {
        times[entryCount] = millis() - startTime;
        distances[entryCount] = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
        entryCount++;
      }
      
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
  testAdvance(800.0, 40);
}

void loop() {
  for (int i = 0; i < entryCount; i++) {
    Serial.print("Time: ");
    Serial.print(times[i]);
    Serial.print(" ms, Distance: ");
    Serial.println(distances[i]);
  }
  delay(5000);
}
