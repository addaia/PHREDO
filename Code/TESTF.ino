// imports (provided in folder)
#include "LineSensors.h"
#include "Magnetometer.h"
#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "PID.h"
#include <math.h>

#define MAX_ENTRIES 220

Motors_c motors;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
Kinematics_c pose;
PID_c left_pid;

float speeds[MAX_ENTRIES];
float times[MAX_ENTRIES];
float distances[MAX_ENTRIES];
int index = 0;

void testAdvance(float targetDistance, unsigned long updateInterval) {
  unsigned long lastUpdateTime = millis();
  motors.setPWM(40, 40);


  while (sqrt(pow(pose.x, 2) + pow(pose.y, 2)) < targetDistance) {

    float prev_x = pose.x;
    float prev_y = pose.y;
    unsigned long prev_time = millis();
    
    if (millis() - lastUpdateTime >= updateInterval) {
      pose.update();

      // Compute distance traveled since last update
      float dx = pose.x - prev_x;
      float dy = pose.y - prev_y;
      float distance_travelled = sqrt(dx * dx + dy * dy);


      // Compute time elapsed
      unsigned long current_time = millis();
      float time_elapsed = (current_time - prev_time) / 1000.0; // Convert ms to seconds
      
      // Compute speed and store it in circular buffer
      speeds[index] = distance_travelled/time_elapsed;
      times[index] = current_time;
      distances[index] = sqrt(pose.x * pose.x + pose.y * pose.y);
      index = (index + 1) % MAX_ENTRIES; // Circular index update
      
      
      // Update the PID controller with demand and current measurement of total counts, ensures both wheels rotate same number of counts
      float l_pwm = left_pid.update( pose.last_e0, pose.last_e1 );
    
      // Adjust speed of left motor
      motors.setPWM( l_pwm + 40, 40 );


      // Print speed to Serial Monitor
      Serial.println(prev_x);
      Serial.println(prev_x);
      //Serial.print("Speed (mm/s): ");
      //Serial.print(speed_);
      //Serial.print("\n");



      
      
    
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

  for (int i = 0; i < MAX_ENTRIES; i++) {
    Serial.print("Time: ");
    Serial.print(times[i]);
    Serial.print("Distance: ");
    Serial.print(distances[i]);
    Serial.print("Speed: ");
    Serial.println(speeds[i]);
  }
  delay(5000);
  
}

