/************************************
,        .       .           .      , 
|        |       |           |     '| 
|    ,-: |-. ,-. |-. ,-. ,-. |-     | 
|    | | | | `-. | | |-' |-' |      | 
`--' `-` `-' `-' ' ' `-' `-' `-'    ' 
*************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

// Pin definitions.  By using #define we can
// switch the number here, and everywhere the
// text appears (i.e. L_PWM) it will be
// replaced.
#define L_PWM 10 // This is correct.
#define L_DIR 16 // This is the wrong pin! 
#define R_PWM 9 // This is the wrong pin! 
#define R_DIR 15 // This is the wrong pin! 

// It is a good idea to limit the maximum power
// sent to the motors. Using #define means we
// can set this value just once here, and it
// can be used in many places in the code below.
#define MAX_PWM 180.0

#define FWD LOW
#define REV HIGH

// define clipper
int clip_to_max(int val, int max_val) {
    if (max_val>val){
      return val;
    }
    else{
      return max_val;
    }
}

// Class to operate the motors.
class Motors_c {

  public:

    // Constructor, must exist.
    Motors_c() {
      // Leave empty. Ensure initialise() is called
      // instead.
    }

    // Use this function to initialise the pins that
    // will control the motors, and decide what first
    // value they should have.
    void initialise() {

      // initialise;
      pinMode(L_PWM, OUTPUT);
      pinMode(R_PWM, OUTPUT);
      pinMode(L_DIR, OUTPUT);
      pinMode(R_DIR, OUTPUT);

      digitalWrite( L_DIR, FWD );
      digitalWrite( R_DIR, FWD );
      analogWrite(L_PWM, 0);
      analogWrite(R_PWM, 0);

    } // End of initialise()


    // This function will be used to send a power value
    // to the motors.
    //
    // The power sent to the motors is created by the
    // analogWrite() function, which is producing PWM.
    // analogWrite() is intended to use a range between
    // [0:255].
    //
    // This function takes two input arguments: "left_pwr"
    // and "right_pwr", (pwr = power) and they are of the
    // type float. A float might be a value like 0.01, or
    // -150.6
    void setPWM( float left_pwr, float right_pwr ) {

      // correct behaviour for reverse (avoid negative values)

      if ( left_pwr < 0 ) {
        digitalWrite( L_DIR, REV );
        left_pwr = left_pwr * -1;
      } else {
        digitalWrite( L_DIR, FWD );
        
      }
      if ( right_pwr < 0 ) {
        digitalWrite( R_DIR, REV );
        right_pwr = right_pwr * -1;
      } else {
        digitalWrite( R_DIR, FWD );
      }

      // clip [0, MAX PWM] 
      left_pwr = clip_to_max(left_pwr, MAX_PWM);
      right_pwr = clip_to_max(right_pwr,  MAX_PWM);


      // run
      analogWrite( L_PWM, left_pwr );
      analogWrite( R_PWM, right_pwr );

      // Done!
      return;

    } // End of setPWM()


}; // End of Motors_c class definition.



#endif
