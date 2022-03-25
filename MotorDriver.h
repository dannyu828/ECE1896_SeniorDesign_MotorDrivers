#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#define WHEEL_DIAMETER 0.06 // meters
#define WHEEL_CIRCUMFERENCE 0.18849555921 // meters
#define COUNTS_PER_ROTATION 11

// Pins
const int ENCA[2] = {5,2};
const int ENCB[2] = {7,8};
const int PWM[2]  = {15,35};
const int IN1[2]  = {1,26};
const int IN2[2]  = {25,27};
const int STBY = {0};

// Globals for ISR
extern volatile int pos_enc[2];

class Motor {
  public:
    Motor() {};
    void setup();
    void PID_loop(uint8_t, char);
    float calcLinearVelocity(float);
    void enable();
    void disable();
    //static void readEncoder0();
    //static void readEncoder1();
    void drive(uint8_t, char);
    void forward(uint8_t);
    void backward(uint8_t);
    void left(uint8_t);
    void right(uint8_t);
    void stop();
    void standby();
    char direction;
    uint8_t currentSpeed, controlledSpeed;
    //volatile int get_pos_enc() { return *pos_enc; };
  private:
    float prevVelocity, currTime, prevTime, prevPos;
    
    //volatile int pos_enc[2] = {0,0}; // Position variable for interrupt should be volatile
};

// A class to compute the control signal
class PID {
  private:
    float Kp, Ki, Kd;          // Parameters
    float prevError, integral; // Storage

  public:
    // Constructor
    PID() : Kp(1), Ki(0), Kd(0), prevError(0.0), integral(0.0) {}
  
    // Set parameters
    void setParameters(float KpIn, float KiIn, float KdIn){
      Kp = KpIn;
      Ki = KiIn;
      Kd = KdIn;
    }
  
    // Compute the PID control signal
    float control(int currentSpeed, int desiredSpeed, float deltaT){
      // Determine error, its derivative, and its integral
      int error = desiredSpeed - currentSpeed;
      float derivative = (error - prevError)/(deltaT);
      integral = integral + error*deltaT;
  
      // Compute output signal
      float output = Kp*error + Ki*integral + Kd*derivative;
      output = (uint8_t) fabs(output);
    
      // Save previous error
      prevError = error;

      return output;
    }
};

#endif
