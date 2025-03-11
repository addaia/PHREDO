// imports
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

// funcs
Motors_c motors;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
Kinematics_c pose;
PID_c left_pid;
LSM6 imu;

//define some constants
#define MAX_ENTRIES 150
float speeds[MAX_ENTRIES];
unsigned long times[MAX_ENTRIES];
//float distances[MAX_ENTRIES];
int entryCount = 0;
unsigned long pos_update_time = 0;


// calibration IMU
float ax_bias = 0.0;
float calibrationMin = 0.0;
float calibrationMax = 0.0;

// normalistaion IMU
const float NORMALIZATION_FACTOR = 1000.0;
#define ACC_CONVERSION_FACTOR 0.061

// params for moving average filter (MAF
const int filterSize = 3;
float filterBuffer[filterSize] = {0};
int filterIndex = 0;
float filterSum = 0.0;
float overshoot;

// MAF
float filterAcceleration(float new_sample) {
  filterSum -= filterBuffer[filterIndex];   // remove oldest sample
  filterBuffer[filterIndex] = new_sample;       // insert new sample
  filterSum += new_sample;                      // update sum
  filterIndex = (filterIndex + 1) % filterSize; // wrap index
  return filterSum / filterSize;                // return average
}

// calibrate IMU
float calibrateAccelerometer(float &minVal, float &maxVal) {
  const int calibrationSamples = 100;
  float sum = 0;
  // init range
  minVal = 1e6;
  maxVal = -1e6;

  for (int i = 0; i < calibrationSamples; i++) {
    imu.read();
    float current = imu.a.x;
    // track range
    if (current < minVal) {
      minVal = current;
    }
    if (current > maxVal) {
      maxVal = current;
    }
    sum += current;
    delay(100);  // delay for a more comprehensive set of values
  }
  float bias = sum / calibrationSamples;
  return bias;
}

//compute overshoot
float computeOvershoot(const unsigned long times[], const float acc_g[])
{

  // init values
  float velocityPrev = 0.0f;
  float positionPrev = 0.0f;

  for (int i = 1; i < 25; i++)
  {
    // convert time difference from ms to s:
    float dt = (times[i] - times[i - 1]) / 1000.0f;

    // convert acceleration from g to m/s^2:
    float a_i = acc_g[i] * 9.81f;
    float a_im1 = acc_g[i - 1] * 9.81f;

    //average acc
    float aAvg = 0.5f * (a_i + a_im1);

    // integrate acc to update velocity
    float velocityCurr = velocityPrev + aAvg * dt;

    // integrate v to update displacemen
    float positionCurr = positionPrev + 0.5f * (velocityPrev + velocityCurr) * dt;

    velocityPrev = velocityCurr;
    positionPrev = positionCurr;
  }
  //return last value
  positionPrev = positionPrev / ACC_CONVERSION_FACTOR;
  return positionPrev;
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
        float raw_ax = imu.a.x;
        float filtered_ax = 0;
        // 0 if in range
        if (raw_ax >= calibrationMin && raw_ax <= calibrationMax) {
          filtered_ax = 0;
        } else {
          // standard procedure
          float corrected_ax = raw_ax - ax_bias;
          float normalized_ax = corrected_ax * ACC_CONVERSION_FACTOR  / NORMALIZATION_FACTOR;
          filtered_ax = filterAcceleration(normalized_ax);
        }

        speeds[entryCount] = filtered_ax;
        times[entryCount] = millis() - startTime;
        entryCount++;
      }

      lastUpdateTime = millis();
    }
  }
  //stopp them motors
  motors.setPWM(0, 0);

  // few extra readings after stop
  for (int i = 0; i < 25; i++) {
    delay(updateInterval);
    if (entryCount < MAX_ENTRIES) {
      imu.read();
      pose.update();
      float raw_ax = imu.a.x;
      float filtered_ax = 0;
      if (raw_ax >= calibrationMin && raw_ax <= calibrationMax) {
        filtered_ax = 0;
      } else {
        float corrected_ax = raw_ax - ax_bias;
        float normalized_ax = corrected_ax * ACC_CONVERSION_FACTOR  / NORMALIZATION_FACTOR;
        filtered_ax = filterAcceleration(normalized_ax);
      }

      speeds[entryCount] = filtered_ax;
      times[entryCount] = millis() - startTime;
      entryCount++;
    }
  }
}


// non blocking func
void update_pose(unsigned long duration_ms) {
  if (millis() >= pos_update_time) {
    pose.update();
    pos_update_time = millis() + duration_ms;
  }
}

// slip correct
void errorFix(float targetDistance, unsigned long updateInterval) {
  // units factor
  const float conversionFactor = 0.1;

  // update
  pose.update();
  float init_x = pose.x;
  float init_y = pose.y;

  // set speed
  motors.setPWM(-20, -20);

  float dx = (pose.x - init_x) * conversionFactor;
  float dy = (pose.y - init_y) * conversionFactor;
  float traveledDistance = sqrt(pow(dx, 2) + pow(dy, 2));

  // loop
  while (traveledDistance < targetDistance) {
    update_pose(updateInterval);

    dx = (pose.x - init_x) * conversionFactor;
    dy = (pose.y - init_y) * conversionFactor;
    traveledDistance = sqrt(pow(dx, 2) + pow(dy, 2));
  }

  // stop motors
  motors.setPWM(0, 0);
}


/*
// slip correct
void errorFix(float targetDistance, unsigned long updateInterval) {
  // units factor
  const float conversionFactor = 0.1;
  unsigned long lastUpdateTime = millis();

  // update
  pose.update();
  float init_x = pose.x;
  float init_y = pose.y;

  // set speed
  motors.setPWM(-20, -20);


  // loop
  while (sqrt(pow((pose.x - init_x) * conversionFactor, 2) + pow((pose.y - init_y) * conversionFactor, 2)) < targetDistance) {
    if (millis() - lastUpdateTime >= updateInterval) {
      pose.update();
  }

  // stop motors
  motors.setPWM(0, 0);
}

}

*/
void setup() {
  Serial.begin(9600);

  // init the IMU
  Wire.begin();
  if (!imu.init()) {
    while (1) {
      Serial.println("Failed to detect and initialize IMU!");
      delay(1000);
    }
  }
  imu.enableDefault();

  // other init
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  left_pid.initialise(-1, -0.01, 0.0);
  pose.initialise(0, 0, 0);

  // calibrate IMU
  ax_bias = calibrateAccelerometer(calibrationMin, calibrationMax);

  // begin the movement test
  testAdvance(800.0, 40);

  // get overshoot
  if (entryCount >= 25)
  {
    overshoot = computeOvershoot(
                  &times[entryCount - 25],
                  &speeds[entryCount - 25]);

  }
  delay(1000);

  //correct overshoot
  if (fabs(overshoot) > 1) {
    errorFix(fabs(overshoot), 40);
  }

}

void loop() {

  // print
  for (int i = 0; i < entryCount; i++) {
    Serial.print(times[i]);
    Serial.print(" ");
    Serial.println(speeds[i]);
  }

  Serial.print("overshoot: ");
  Serial.println(fabs(overshoot)); ;

  delay(5000);
}
