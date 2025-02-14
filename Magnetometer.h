/***************************************
 ,        .       .           .     ,--, 
 |        |       |           |       /  
 |    ,-: |-. ,-. |-. ,-. ,-. |-     `.  
 |    | | | | `-. | | |-' |-' |        ) 
 `--' `-` `-' `-' ' ' `-' `-' `-'   `-'  
***************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3

class Magnetometer_c {

  public:

    // Instance of the LIS3MDL class used to
    // interact with the magnetometer device.
    LIS3MDL mag;

    // A place to store the latest readings 
    // from the magnetometer
    float readings[ MAX_AXIS ];

    float minimum[ MAX_AXIS ];
    float maximum[ MAX_AXIS ];
    float offset[ MAX_AXIS ];
    float scaling[ MAX_AXIS ];

    float calibrated[ MAX_AXIS ];

    // Constructor, must exist.
    Magnetometer_c () {
      for( int sensor = 0; sensor < MAX_AXIS; sensor++ ) {
              minimum[sensor] =  9999.9;
              maximum[sensor] = -9999.9;
      }
    }

    // Call this function witin your setup() function
    // to initialise the I2C protocol and the
    // magnetometer sensor
    bool initialise() {

      // Start the I2C protocol
      Wire.begin();

      // Try to connect to the magnetometer
      if ( !mag.init() ) {
        return false;
      } else {
        mag.enableDefault();
        return true;
      }
      
    } // End of initialise()

    // Function to update readings array with
    // latest values from the sensor over i2c
    void getReadings() {
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    } // End of getReadings()

    void calibrate(){
      // Get latest readings (raw values)
      getReadings();

      // Apply calibration values, store in calibrated[]
      for( int sensor = 0; sensor < MAX_AXIS; sensor++ ) {
        if (readings[sensor] > maximum[sensor]){
          maximum[sensor] = readings[sensor];    
        }
        if (readings[sensor] < minimum[sensor]){
          minimum[sensor] = readings[sensor];    
        }
    }
    }

    void calcScaling() {
      for( int sensor = 0; sensor < MAX_AXIS; sensor++ ) {
        float midpoint = (maximum[sensor] - minimum[sensor])/2.0;
        offset[sensor] = (minimum[sensor] + (midpoint));
        scaling[sensor] = 1.0 / midpoint;
      }
  }

  void calcCalibrated() {

      // Get latest readings (raw values)
      getReadings();

      // Apply calibration values, store in calibrated[]
      for( int sensor = 0; sensor < MAX_AXIS; sensor++ ) {
              calibrated[sensor] = (readings[sensor] - offset[sensor]) * scaling[sensor];
      }
      
    } 

}; // End of Magnetometer_c class definition

#endif
