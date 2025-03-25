## The kinematics.h file may require small adjustments so that encoders are calibrated.

#include "LineSensors.h"
#include "Magnetometer.h"
#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "PID.h"
#include <math.h>

// These two includes are necessary to read
// the LSM6DS33 (accelerometer and gyro)
#include <Wire.h>
#include <LSM6.h>

// IMU comes in mm/s^2
LSM6 imu;

// States
#define STATE_CALLIBRATE 0
#define STATE_MISSION 1
#define STATE_FINISH 2
#define BUZZER_PIN 6


#define MAX_ENTRIES 97
// Define alpha (smoothing factor) globally
#define ALPHA 0.2
float accelerationSum = 0.0;
int accelerationCount = 0;

Motors_c motors;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
Kinematics_c pose;
PID_c left_pid;

//float speedsKinematics[MAX_ENTRIES]; // in mm/s
float times[MAX_ENTRIES]; // in ms
float distancesKinematics[MAX_ENTRIES]; // in mm
//float accelerationsIMU[MAX_ENTRIES]; // in mm/s^2
float speedsIMUAveraged[MAX_ENTRIES]; // in mm/s
float distancesIMU[MAX_ENTRIES];
//float PWM[MAX_ENTRIES]; // Power speed

float totalDistanceTravelled;
float targetDistance;
float distanceIMU;
float sumAccelerations;
int index = 0;
int index1 = 1;
int index2 = 1;
int counter = 0;
int counterr = 0;
int state;

float filteredAcceleration = 0;
float filteredSpeed = 0;
float offsetAcceleration;
float averageAcceleration;

float currentSpeedIMU;

float prevAccelerationIMU = 0;
float previousSpeedIMU = 0;
float currentAccelerationIMU;
float speedIMU = 0;
float maxIMU = -999999999;
float minIMU = 999999999;

float totalStabilisedIMUSpeed = 0;
float totalStabilisedIMUSpeedIndex = 0;
float totalStabilisedEncoderSpeed = 0;
float totalStabilisedEncoderSpeedIndex = 0;

float averageStabilisedIMUSpeed = 0;
float averageStabilisedEncoderSpeed = 0;

float k = 0;

bool isSlip = false;
bool isFirstValue = true;
bool calcAverageK = false;
bool isSlow = false;

unsigned long startCallibrationTime;
unsigned long prevTime;



float newSpeed = 80;
float remainingDistance = 0;




void beepOn() {
    analogWrite( BUZZER_PIN, 120 ); // on
  }

void beepOff() {
      analogWrite( BUZZER_PIN, 0 ); // off
    }



void testAdvance(float targetDistance, unsigned long updateInterval) {
  unsigned long startTime = millis();
  unsigned long lastUpdateTime = millis();
  motors.setPWM(-80, -80);


  while (sqrt(pow(pose.x, 2) + pow(pose.y, 2)) < targetDistance && !isSlip) {

    float prev_x = pose.x;
    float prev_y = pose.y;
    imu.read();

    float rawAcceleration = (imu.a.x * 0.061 * 0.00980665); // in m
    filteredAcceleration = ALPHA * rawAcceleration + (1 - ALPHA) * filteredAcceleration;
    currentAccelerationIMU = (filteredAcceleration - offsetAcceleration) * 1000; // in mm
    if ((rawAcceleration-offsetAcceleration) <= maxIMU && (rawAcceleration-offsetAcceleration) >= minIMU){
      currentAccelerationIMU = 0;
    }

    // Calculate speed using integration (v = v0 + a * dt)
    speedIMU += currentAccelerationIMU * (millis()-prevTime)/1000;

    if (millis() - lastUpdateTime >= updateInterval) {
      pose.update();

      //accelerationsIMU[index] = currentAccelerationIMU;

      speedsIMUAveraged[index] = speedIMU;

      // Compute distance traveled since last update
      float dx = pose.x - prev_x;
      float dy = pose.y - prev_y;
      float distance_travelled = dx;

      // Compute time elapsed
      unsigned long currentTime = millis();
      float timeElapsed = (currentTime - lastUpdateTime);

      // Compute speed and store it in circular buffer
      //speedsKinematics[index] = distance_travelled * 1000 / timeElapsed;
      filteredSpeed = ALPHA * rawAcceleration + (1 - ALPHA) * filteredSpeed;

      totalDistanceTravelled += dx;

      times[index] = (currentTime - startTime);
      distancesKinematics[index] = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
      

      

      // Update the PID controller with demand and current measurement of total counts, ensures both wheels rotate same number of counts
      float l_pwm = left_pid.update( pose.last_e0, pose.last_e1 );

      //PWM[index] = (80+80+l_pwm)/2;
      index += 1 % MAX_ENTRIES; // Circular index update
      // Adjust speed of left motor
      motors.setPWM( -l_pwm - newSpeed, -newSpeed );

      // Calculate average to determine speed IMU when stabilised
      if ( (millis() - startTime) > 500 && (millis() - startTime) < 2000) {
        totalStabilisedIMUSpeed += speedIMU;
        totalStabilisedIMUSpeedIndex += 1;

        totalStabilisedEncoderSpeed += distance_travelled * 1000 / timeElapsed;;
        totalStabilisedEncoderSpeedIndex += 1;
      }

      // Update the time for the next pose update.
      lastUpdateTime = millis();

    }
    prevAccelerationIMU = currentAccelerationIMU;
    previousSpeedIMU = speedIMU;
    prevTime = millis();

    // Detecting if slippage
    if (millis() - startTime > 2000) {
      if (!calcAverageK) {
        averageStabilisedIMUSpeed = totalStabilisedIMUSpeed / totalStabilisedIMUSpeedIndex;
        averageStabilisedEncoderSpeed = totalStabilisedEncoderSpeed / totalStabilisedEncoderSpeedIndex;

        // Coefficient to adjust IMU velocity to encoder velocity which we assume it is the real velocity for plain surface
        k = averageStabilisedEncoderSpeed / averageStabilisedIMUSpeed;
        calcAverageK = true;
      }
      if (abs((speedsIMUAveraged[index-1] / averageStabilisedIMUSpeed) - 1) > 0.05) {
        isSlip = true;
      }
    }
  }


  if (!isSlip) {
    motors.setPWM(0, 0);
    Serial.println("Test complete. Target distance reached.");
  }

  if (isSlip) {
    pose.update();
    unsigned long startSlipTime = millis();
    targetDistance = abs(pose.x);
    distanceIMU = abs(pose.x);

    // PID variables, tune to make speed decrease to zero in 0.5 seconds
    float targetSpeed = 0.0;  // we want to reach 0 speed
    float integral = 0.0;
    float previousError = 0.0;
    float kP = 0.02;
    float kI = 0.01;
    float kD = 0.01;
    
    while ( (distanceIMU - targetDistance) < 400 && !isSlow) {
      float prev_x = pose.x;
      float prev_y = pose.y;
      imu.read();

      float rawAcceleration = (imu.a.x * 0.061 * 0.00980665); // in m
      filteredAcceleration = ALPHA * rawAcceleration + (1 - ALPHA) * filteredAcceleration;

      currentAccelerationIMU = (filteredAcceleration - offsetAcceleration) * 1000; // in mm

      // Calculate speed using integration (v = v0 + a * dt)
      speedIMU += k*(currentAccelerationIMU * (millis()-prevTime)/1000);
      distanceIMU += abs((millis()-prevTime) * (speedIMU) / 1000);

      if (millis() - lastUpdateTime >= updateInterval) {
        pose.update();
        speedsIMUAveraged[index1+index] = speedIMU;

        // Compute distance traveled since last update
        float dx = pose.x - prev_x;
        float dy = pose.y - prev_y;
        float distance_travelled = dx;

        // Compute time elapsed
        unsigned long currentTime = millis();
        float timeElapsed = (currentTime - lastUpdateTime);
  
        // Compute speed and store it in circular buffer
        //speedsKinematics[index1+index] = distance_travelled * 1000 / timeElapsed;
        
        distancesIMU[index1+index] = abs(distanceIMU);
        distancesKinematics[index1+index] = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
        
        // PID control for speed -> target is 0
        float error = targetSpeed - speedIMU;
        integral += error * (millis() - startSlipTime);
        float derivative = (error - previousError) / (millis() - startSlipTime);
        previousError = error;
    
        float pidOutput = kP * error + kI * integral + kD * derivative;
    
        newSpeed += pidOutput; // apply PID correction
        newSpeed = constrain(newSpeed, 0, 255); // constrain PWM
        
        times[index1+index] = (millis() - startSlipTime);
        //PWM[index1+index] = newSpeed;

        

        motors.setPWM(-newSpeed, -newSpeed);

        if (newSpeed < 15){
          isSlow = true;
          motors.setPWM(0, 0);
        }

        // index += 1 % MAX_ENTRIES;
        index1 += 1% MAX_ENTRIES;
        
        // Update the time for the next pose update.
        lastUpdateTime = millis();
      }

      prevTime = millis();

    }
  }









  unsigned long finishTime = millis();






  while ((millis() - finishTime) < 700) {

    float prev_x = pose.x;
    float prev_y = pose.y;
    imu.read();

    float rawAcceleration = (imu.a.x * 0.061 * 0.00980665); // in m
    filteredAcceleration = ALPHA * rawAcceleration + (1 - ALPHA) * filteredAcceleration;

    currentAccelerationIMU = (filteredAcceleration - offsetAcceleration) * 1000; // in mm


    // Calculate speed using integration (v = v0 + a * dt)
    speedIMU += k*(currentAccelerationIMU * (millis()-prevTime)/1000);
    distanceIMU += abs((millis()-prevTime) * (speedIMU) / 1000);

    if (millis() - lastUpdateTime >= updateInterval) {
      counterr += 1;
      pose.update();

      //accelerationsIMU[index] = currentAccelerationIMU;
      speedsIMUAveraged[index2+index1+index] = speedIMU;
      distancesIMU[index2+index1+index] = abs(distanceIMU);
      
      

      // Compute distance traveled since last update
      float dx = pose.x - prev_x;
      float dy = pose.y - prev_y;
      float distance_travelled = dx;


      // Compute time elapsed
      unsigned long currentTime = millis();
      float timeElapsed = (currentTime - lastUpdateTime);

      // Compute speed and store it in circular buffer
      //speedsKinematics[index2+index1+index] = distance_travelled * 1000 / timeElapsed;
      distancesKinematics[index2+index1+index] = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
      times[index2+index1+index] = (currentTime - startTime);

      index2 += 1 % MAX_ENTRIES; // Circular index update

      // Update the time for the next pose update.
      lastUpdateTime = millis();


    }

    prevTime = millis();
  }
}























void setIMUCallibration() {
  startCallibrationTime = millis();
}

void setIMUCallibration2() {
  startCallibrationTime = millis();
}

void doIMUCallibration(unsigned long callibrationTime) {
  while (millis() - startCallibrationTime < callibrationTime) {
    imu.read();

    sumAccelerations = (imu.a.x * 0.061 * 0.00980665) + sumAccelerations;
    counter += 1;
    
  }
}

void doIMUCallibration2(unsigned long callibrationTime) {
  while (millis() - startCallibrationTime < callibrationTime) {
    imu.read();
    
    if ((imu.a.x * 0.061 * 0.00980665 - offsetAcceleration) > maxIMU){
      maxIMU = (imu.a.x * 0.061 * 0.00980665 - offsetAcceleration);
    }

    if ((imu.a.x * 0.061 * 0.00980665 - offsetAcceleration) < minIMU){
      minIMU = (imu.a.x * 0.061 * 0.00980665 - offsetAcceleration);
    }
    
  }
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);

  // Check the IMU initialised ok.
  if (!imu.init() ) {  // no..? :(

    // Since we failed to communicate with the
    // IMU, we put the robot into an infinite
    // while loop and report the error.
    while (1) {
      Serial.println("Failed to detect and initialize IMU!");
      delay(1000);
    }
  }

  // IMU initialise ok!
  // Set the IMU with default settings.
  imu.writeReg(LSM6::CTRL1_XL, 0b01000000); // 208 Hz ODR, ±2g
  //imu.writeReg(LSM6::CTRL1_XL, 0b01011000); // 208 Hz, +/4 g



  // Initialise components as before.

  motors.initialise();
  left_pid.initialise( 1, 0.01, 0.0 ); // Negative because counts are negative
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);
  state = STATE_CALLIBRATE;
  setIMUCallibration();
}

void loop() {

  if ( state == STATE_CALLIBRATE ) {
    // Ensure it starts fully at rest
    delay(500);

    doIMUCallibration(5000);

    setIMUCallibration2();

    offsetAcceleration = sumAccelerations / counter;

    doIMUCallibration2(4000);

    state = STATE_MISSION;
  }

  else if ( state == STATE_MISSION ) {

    testAdvance(1400.0, 40);
    state = STATE_FINISH;
  }

  else if (state == STATE_FINISH) {
    pose.update();
    for (int i = 0; i < MAX_ENTRIES; i++) {
      Serial.print(times[i]); // ms
      Serial.print(", ");
      Serial.print(speedsIMUAveraged[i]); // mm/s
      Serial.print(", ");
      Serial.print(distancesKinematics[i]);
      Serial.print(", ");
      Serial.println(distancesIMU[i]);
    }
    delay(5000);
  }

}
