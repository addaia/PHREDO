// imports (provided in folder)
#include "LineSensors.h"
#include "Magnetometer.h"
#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "PID.h"
# include <Wire.h>
# include <LSM6.h>
#include <math.h>
#define BUZZER_PIN 6

Motors_c motors;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
Kinematics_c pose;
PID_c left_pid;
LSM6 imu;


# define MAX_ENTRIES 220
float times[MAX_ENTRIES];
float distances[MAX_ENTRIES];
int entryCount = 0;

void testAdvance(float targetDistance, unsigned long updateInterval) {
  unsigned long lastUpdateTime = millis();
  unsigned long startTime = millis();
  const unsigned long engineModInterval = 80;
  unsigned long lastEngineModTime = millis();
  bool engine = true;
  motors.setPWM(200, 200);


  while (sqrt(pow(pose.x, 2) + pow(pose.y, 2)) < targetDistance) {
    if (millis() - lastUpdateTime >= updateInterval) {
      pose.update();

      if (entryCount < MAX_ENTRIES) {
        imu.read();

        times[entryCount] = millis() - startTime;
        distances[entryCount] = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
        entryCount++;
      }

      // Update the time for the next pose update.
      lastUpdateTime = millis();
    }
    if (millis() - lastEngineModTime >= engineModInterval) {
        if (engine) {
          float l_pwm = left_pid.update(pose.last_e0, pose.last_e1);
          motors.setPWM(-l_pwm + 200, 200);
          engine = false;
        } else {
          motors.setPWM(0, 0);
          engine = true;
        }
        lastEngineModTime = millis();
      }
  }
  motors.setPWM(0, 0);
  for (int i = 0; i < 5; i++) {
    delay(updateInterval);

    if (entryCount < MAX_ENTRIES) {
      pose.update();

      times[entryCount] = millis() - startTime;
      distances[entryCount] = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
      entryCount++;
    }
  }



  //analogWrite( BUZZER_PIN, 120 );
  Serial.println("Test complete. Target distance reached.");
}

void setup() {

  // IMU
  Wire.begin();
  if (!imu.init() ) {  // no..? :(

    // Since we failed to communicate with the
    // IMU, we put the robot into an infinite
    // while loop and report the error.
    while (1) {
      Serial.println("Failed to detect and initialize IMU!");
      delay(1000);
    }
  }
  imu.enableDefault();

  // plotter
  Serial.begin(9600);

  // initialise standard comps
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  left_pid.initialise( -1, -0.01, 0.0 );
  pose.initialise(0, 0, 0);
  testAdvance(800.0, 40);
}

void loop() {




  for (int i = 0; i < entryCount; i++) {
    Serial.print(times[i]);
    Serial.print(" ");
    Serial.println(distances[i]);
  }
  delay(5000);

}
