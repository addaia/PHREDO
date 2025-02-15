#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:
    // PID update variables
    float last_error; // To store the previous error
    float p_term;     // Proportional term
    float i_term;     // Integral term
    float d_term;     // Derivative term
    float i_sum;      // Accumulated integral
    float feedback;   // Feedback output of the PID controller

    // To store gains
    float p_gain;     // Proportional gain
    float i_gain;     // Integral gain
    float d_gain;     // Derivative gain

    // To determine time elapsed
    unsigned long ms_last_t; // Last update timestamp

    // Constructor
    PID_c() {
      // Leaving empty for explicit initialization
    }

    // Initialize PID with gains for P, I, and D
    void initialise(float p, float i, float d) {
      feedback = 0;
      last_error = 0;
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;

      p_gain = p;
      i_gain = i;
      d_gain = d;

      ms_last_t = millis();
    }

    // Reset the PID state, useful after delays or interruptions
    void reset() {
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;
      last_error = 0;
      feedback = 0;
      ms_last_t = millis();
    }

    // Update the PID controller
    float update(float demand, float measurement) {
      float error;               // Current error
      unsigned long ms_now_t;    // Current time
      unsigned long ms_dt;       // Time delta
      float float_dt;            // Time delta as a float
      float diff_error;          // Difference in error for derivative term

      // Get the current time and calculate elapsed time
      ms_now_t = millis();
      ms_dt = ms_now_t - ms_last_t;

      // Update ms_last_t for the next call
      ms_last_t = ms_now_t;

      // Convert time delta to float
      float_dt = (float)ms_dt / 1000.0; // Convert ms to seconds for better scaling

      // Avoid divide-by-zero errors
      if (float_dt == 0.0) {
        return feedback;
      }

      // Calculate the error signal
      error = demand - measurement;

      // P term: Proportional to the current error
      p_term = p_gain * error;

      // I term: Accumulate error over time
      i_sum += error * float_dt;
      i_term = i_gain * i_sum;

      // D term: Rate of change of error
      diff_error = (error - last_error) / float_dt;
      d_term = d_gain * diff_error;

      // Update the feedback (control output)
      feedback = p_term + i_term + d_term;

      // Save the current error for the next derivative calculation
      last_error = error;

      // Return the control output
      return feedback;
    }
};

#endif
