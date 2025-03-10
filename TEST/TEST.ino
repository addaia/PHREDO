#include "LineSensors.h"
#include "Magnetometer.h"
#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "PID.h"
#include <Wire.h>
#include <LSM6.h>
#include <math.h>
#define BUZZER_PIN 6

Motors_c motors;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
Kinematics_c pose;
PID_c left_pid;
LSM6 imu;

#define MAX_ENTRIES 200
float speeds[MAX_ENTRIES];
unsigned long times[MAX_ENTRIES];
//float distances[MAX_ENTRIES];
int entryCount = 0;

// acceleartor bias
float ax_bias = 0.0;

// normalisation factor, needs to be checked based on units
const float NORMALIZATION_FACTOR = 1000.0;

// parameters for a simple filter
const int filterSize = 10;
float filterBuffer[filterSize] = {0};
int filterIndex = 0;
float filterSum = 0.0;

// moving average filter
float filterAcceleration(float new_sample) {
  filterSum -= filterBuffer[filterIndex];   // remove oldest sample
  filterBuffer[filterIndex] = new_sample;       // insert new sample
  filterSum += new_sample;                      // update sum
  filterIndex = (filterIndex + 1) % filterSize; // wrap index
  return filterSum / filterSize;                // return average
  }

// function to calibrate

float calibrateAccelerometer() {
  float bias = 0;
  const int calibrationSamples = 1000;
  
  for (int i = 0; i < calibrationSamples; i++) {
    imu.read();
    bias += imu.a.x;
    delay(50);  // Adjust delay if needed for a stable reading
  }
  bias /= calibrationSamples;
  return bias;
}




void testAdvance(float targetDistance, unsigned long updateInterval) {
  unsigned long startTime = millis();
  unsigned long lastUpdateTime = millis();
  motors.setPWM(200, 200);
  
  while (sqrt(pow(pose.x, 2) + pow(pose.y, 2)) < targetDistance) {
    if (millis() - lastUpdateTime >= updateInterval) {
      pose.update();
  
      float l_pwm = left_pid.update(pose.last_e0, pose.last_e1);
      motors.setPWM(-l_pwm + 200, 200);
  
      if (entryCount < MAX_ENTRIES) {
        imu.read();
        // sub bias
        float raw_ax = imu.a.x;
        float corrected_ax = raw_ax - ax_bias;
        // normalise
        float normalized_ax = corrected_ax / NORMALIZATION_FACTOR;
        // filter
        float filtered_ax = filterAcceleration(normalized_ax);
  
        speeds[entryCount] = filtered_ax;
        //distances[entryCount] = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
        times[entryCount] = millis() - startTime;
        entryCount++;
      }
  
      lastUpdateTime = millis();
    }
  }
  motors.setPWM(0, 0);
  
  // take a few extra readings after stopping
  for (int i = 0; i < 5; i++) {
    delay(updateInterval);
    if (entryCount < MAX_ENTRIES) {
      imu.read();
      pose.update();
  
      float raw_ax = imu.a.x;
      float corrected_ax = raw_ax - ax_bias;
      float normalized_ax = corrected_ax / NORMALIZATION_FACTOR;
      float filtered_ax = filterAcceleration(normalized_ax);
  
      speeds[entryCount] = filtered_ax;
      //distances[entryCount] = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
      times[entryCount] = millis() - startTime;
      entryCount++;
    }
  }
  
  Serial.println("Test complete. Target distance reached.");
}

void setup() {
  
  Serial.begin(9600);
  
  // initialise the IMU
  Wire.begin();
  if (!imu.init()) {
    while (1) {
      Serial.println("Failed to detect and initialize IMU!");
      delay(1000);
    }
  }
  imu.enableDefault();
  
  // rest
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  left_pid.initialise(-1, -0.01, 0.0);
  pose.initialise(0, 0, 0);
  
  // calibrate
  ax_bias = calibrateAccelerometer();
  Serial.print("Calibrated accelerometer bias (X-axis): ");
  Serial.println(ax_bias);
  
  // start
  testAdvance(800.0, 40);
}

void loop() {
  // print stored results
  for (int i = 0; i < entryCount; i++) {
    Serial.print(times[i] );
    Serial.print(" ");
    Serial.println(speeds[i]);
  }
  
  delay(5000);
}
