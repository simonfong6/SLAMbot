// Motor driver shield library
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

void setup() {
    // Set up motor driving
    AFMS.begin();  // create with the default frequency 1.6KHz
}

void loop() {
  //Set motor speeds.
  leftMotor->setSpeed(60);
  rightMotor->setSpeed(60);

  testAllTurns();

}

void moveForward(){
	leftMotor->run(FORWARD);
	rightMotor->run(FORWARD);
}

void moveBackward(){
	leftMotor->run(BACKWARD);
	rightMotor->run(BACKWARD);
}

void motorStop(){
	leftMotor->run(RELEASE);
	rightMotor->run(RELEASE);
}

void testAllTurns(){
  turnLeftLeftOnly();
  wait();
  turnRightLeftOnly();
  wait();

  turnLeftRightOnly();
  wait();
  turnRightRightOnly();
  wait();

  turnLeftBoth();
  wait();
  turnRightBoth();
  wait();
}

//Waits for three seconds.
void wait(){
  delay(3000);
}

//Turns left using only the left wheel. Right wheel does not move at all.
void turnLeftLeftOnly(){
  leftMotor->run(BACKWARD);
  delay(1800);

  leftMotor->run(RELEASE);
}

//Turns right using only the left wheel.
void turnRightLeftOnly(){
  leftMotor->run(FORWARD);
  delay(1800);

  leftMotor->run(RELEASE);
}

//Turns left using only right wheel.
void turnLeftRightOnly(){
  rightMotor->run(FORWARD);
  delay(1800);

  rightMotor->run(RELEASE);
} 

//Turns right using only right wheel.
void turnRightRightOnly(){
  rightMotor->run(BACKWARD);
  delay(1800);

  rightMotor->run(RELEASE);
}

//Turns left using both wheels.
void turnLeftBoth(){
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  delay(1800);

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

//Turns right using both wheels.
void turnRightBoth(){
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  delay(1800);

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}



