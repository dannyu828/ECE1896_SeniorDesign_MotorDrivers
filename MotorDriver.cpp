#include "MotorDriver.h"

volatile int pos_enc[2];
// ISR to read encoder pulses
// Create two functions as a template so each encoder gets its own function
void readEncoder0() {
  int b = digitalRead(7); // ENCB[0]
  if(b > 0) pos_enc[0]++;
  else      pos_enc[0]--;
}

void readEncoder1() {
  int b = digitalRead(8); // ENCB[1]
  if(b > 0) pos_enc[1]++;
  else      pos_enc[1]--;
}

//    float calcLinearVelocityAcc() {
//      float linearAcc = 0; // Should be gotten from accelerometer
//      float linearVelocity = prevVelocity + linearAcc*deltaT;
//      prevVelocity = linearVelocity
//      return linearVelocity;
//    }

void Motor::setup() {
  pinMode(STBY, OUTPUT);
  pos_enc[0] = 0;
  pos_enc[1] = 0;
  
  for (int i = 0; i < 2; i++) {
    // Set all required pins as outputs
    pinMode(PWM[i], OUTPUT);
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
  
    // Set encoder pins as inputs
    pinMode(ENCA[i], INPUT);
    pinMode(ENCB[i], INPUT);
  
    // Set all pins to low to start
    digitalWrite(PWM[i], LOW);
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], LOW);

    // Standby is active-low
    digitalWrite(STBY, HIGH);
    
    // Assign and setup PWM channels
    ledcAttachPin(PWM[i], i);
    ledcSetup(i, 3000, 8);
  };
};

// Main input function
void Motor::PID_loop(uint8_t desiredSpeed, char desiredDirection) {
  // Map from 0-100 to 0-255
  desiredSpeed = map(desiredSpeed,0,100,0,255);

  // Set direction variable
  direction = desiredDirection;
  
  // Read the position and temporarily disable interrupts while reading to prevent misreads
  int pos;
  noInterrupts();
  pos = pos_enc[0]; // If this function works, pos_enc doesn't need to be an array
  interrupts();

  // Set time and position, calc velocity
  long currTime = micros();
  float deltaT = ((float) (currTime - prevTime))/(1.0e6);
  float velocity = ((float) (pos - prevPos))/deltaT;
  prevPos = pos;
  prevTime = currTime;

  // Calculate linear velocity/current speed
  currentSpeed = calcLinearVelocity(velocity);

  // Calculate the control signal and output to motor
  PID pid;  // Set K params later
  controlledSpeed = pid.control(currentSpeed, desiredSpeed, deltaT);
  drive(controlledSpeed, direction);
}

float Motor::calcLinearVelocity(float encVelocity) {
  float rpm = encVelocity/COUNTS_PER_ROTATION*60;
  float linearVelocity = rpm*WHEEL_CIRCUMFERENCE;
  return linearVelocity;
}

void Motor::enable() {
  digitalWrite(STBY, HIGH);

  // Attach interrupts to encoder pulses
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), readEncoder1, RISING);
}

void Motor::disable() {
  digitalWrite(STBY, LOW);

  // Attach interrupts to encoder pulses
  detachInterrupt(digitalPinToInterrupt(ENCA[0]));
  detachInterrupt(digitalPinToInterrupt(ENCA[1]));
}

//Motor::ReadEncoder() {
//  // Read encoder B when ENCA rises
//  int B = digitalRead(ENCB);
//  int increment = 0;
//  // If B is high, increment forward
//  if(B > 0) {
//    position++;
//  }
//  // Otherwise, increment backward
//  else {
//    position--;
//  }
//
//  // Compute velocity with method 2
//  long currT = micros();
//  float deltaT = ((float) (currT - prevT_i))/1.0e6;
//  velocity_i = increment/deltaT;
//  prevT_i = currT;
//}

// Function call for motion planning
void Motor::drive(uint8_t speed, char inputDir) {
  direction = inputDir;

  // Set wheel directions
  switch (direction) {
    case 'F':
      forward(speed);
      break;
    case 'B':
      backward(speed);
      break;
    case 'R':
      right(speed);
      break;
    case 'L':
      left(speed);
      break;
    case 'S':
      stop();
      break;
    case 'T':
      standby();
      break;
  };
}

void Motor::forward(uint8_t speed) {  
  // Rotate Left Motor A CCW
  digitalWrite(IN1[0], LOW);
  digitalWrite(IN2[0], HIGH);
  
  // Rotate Right Motor B CW
  digitalWrite(IN1[1], HIGH);
  digitalWrite(IN2[1], LOW);

  // Output speed
  ledcWrite(0, speed);
  ledcWrite(1, speed);
}

void Motor::backward(uint8_t speed) {
  digitalWrite(STBY, HIGH);
  
  // Rotate Left Motor A CW
  digitalWrite(IN1[0], HIGH);
  digitalWrite(IN2[0], LOW);
  
  // Rotate Right Motor B CCW
  digitalWrite(IN1[1], LOW);
  digitalWrite(IN2[1], HIGH);

  // Output speed
  ledcWrite(0, speed);
  ledcWrite(1, speed);
}

void Motor::left(uint8_t speed) {
  digitalWrite(STBY, HIGH);
  
  // Rotate Left Motor A CCW
  digitalWrite(IN1[0], LOW);
  digitalWrite(IN2[0], HIGH);
  
  // Rotate Right Motor B CCW
  digitalWrite(IN1[1], LOW);
  digitalWrite(IN2[1], HIGH);

  // Output speed
  ledcWrite(0, speed);
  ledcWrite(1, speed);
}

void Motor::right(uint8_t speed) {
  digitalWrite(STBY, HIGH);
  
  // Rotate Left Motor A CW
  digitalWrite(IN1[0], HIGH);
  digitalWrite(IN2[0], LOW);
  
  // Rotate Right Motor B CW
  digitalWrite(IN1[1], HIGH);
  digitalWrite(IN2[1], LOW);

  // Output speed
  ledcWrite(0, speed);
  ledcWrite(1, speed);
}

void Motor::stop() {
  digitalWrite(STBY, HIGH);

  // Output speed
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void Motor::standby() {
  digitalWrite(STBY, LOW);

  // Output speed
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}
