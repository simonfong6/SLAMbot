// Motor driver shield library
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

void setupMotors(){
  //Set up motor driving
  AFMS.begin();  // create with the default frequency 1.6KHz

  //Set motor speeds.
  leftMotor->setSpeed(60);
  rightMotor->setSpeed(60);
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

//Rotates left using both wheels.
void rotateLeftBoth(int turningTime){
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  
  delay(turningTime);

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

//Rotates right using both wheels.
void rotateRightBoth(int turningTime){
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  
  delay(turningTime);

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

//Defining and left and right ultrasonic pins
#define LEFT_BOTTOM_ECHO_PIN 30
#define LEFT_BOTTOM_TRIG_PIN 31

#define RIGHT_BOTTOM_ECHO_PIN 33
#define RIGHT_BOTTOM_TRIG_PIN 32

//Maximum amount of times that the car will try to turn before it becomes a failed turn.
#define MAX_TURNING_ITERATIONS 100

//Tolerance of angle for turning.
#define ANGLE_TOLERANCE (PI/2/100)

//Amount of milliseconds to initially turn.
#define INITIAL_TURNING_TIME 50

//Amount of milliseconds to decrement turning time.
#define TURNING_TIME_CHANGE 1

//Flag to check if bot is turning.
//Need to update in turn function.
bool turning = false;

//Turns to desired angle in radians using both wheels.
bool turnBoth(double targetAngle){

  turning = true;

  double minAngleBound = targetAngle - ANGLE_TOLERANCE;
  double maxAngleBound = targetAngle + ANGLE_TOLERANCE;

  int turningTime = INITIAL_TURNING_TIME;

  bool turningLeft = false;
  
  for(int i = 0; i < MAX_TURNING_ITERATIONS; i++){
    double currentAngle = gyroRead();

    //Check if car is at target angle within tolerance.
    if( (currentAngle > minAngleBound) && (currentAngle < maxAngleBound) ){
      turning = false;
      return true;
    }
    //If the angle is too low rotate left.
    else if(currentAngle <= minAngleBound){
      if( !turningLeft ){
        turningTime -= TURNING_TIME_CHANGE;
        turningLeft = true;
      }
      rotateLeftBoth(turningTime);
    }
    //If the angle is too high rotate right.
    else if(currentAngle >= maxAngleBound){
      if( turningLeft ){
        turningTime -= TURNING_TIME_CHANGE;
        turningLeft = false;
      }
      rotateRightBoth(turningTime);
    }else{
      Serial1.println("Error in turning left.");
    }

    //Read left and right distances from ultrasonic sensors.
    double leftBottomDistance = readUltraSonic(LEFT_BOTTOM_TRIG_PIN, LEFT_BOTTOM_ECHO_PIN);
    double rightBottomDistance = readUltraSonic(RIGHT_BOTTOM_TRIG_PIN, RIGHT_BOTTOM_ECHO_PIN);

    //Detects if there is a left or right edge.
    bool isLeftEdge = detectsEdge(leftBottomDistance);
    bool isRightEdge = detectsEdge(rightBottomDistance);

    //If there is a left or right edge return false, indicating that turn failed.
    if(isLeftEdge && turningLeft){
      turning = false;
      return false;
    }

    if(isRightEdge && !turningLeft){
      turning = false;
      return false;
    }
  }

  turning = false;
  
  //If maximum iterations has been reached and angle is still not within tolerance.
  return false;
}

bool turnLeftBoth(){
  double currentAngle = gyroRead();
  double targetAngle = currentAngle + PI/2;

  bool success = turnBoth(targetAngle);
  if(!success){
    turnBoth(currentAngle);
  }

  return success;
}

bool turnRightBoth(){
  double currentAngle = gyroRead();
  double targetAngle = currentAngle - PI/2;

  bool success = turnBoth(targetAngle);
  if(!success){
    turnBoth(currentAngle);
  }

  return success;
}



#include <Adafruit_LSM9DS0.h>

// Create Adafruit_LSM9DS0 object
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

//Global variables for gyroscope data.
double anglez = 0;
double dt = 0;
double prev_time = 0;
double biasz = 0;

//Configuring the gyroscope for a specific range
void configureIMU(void) {
    /* Initialise the sensor */
  if(!lsm.begin()) {
      while(1);
  }
  
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);

  for(int i = 0; i < 100; i++) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    biasz += gyro.gyro.z;
  }

  biasz = biasz / 100.;
}

//Reads gyroscope and returns current angle in radians in reference to initial angle to be 0 radians.
double gyroRead() {
    sensors_event_t accel, mag, gyro, temp;

    lsm.getEvent(&accel, &mag, &gyro, &temp);

    dt = (millis() - prev_time) / (1000.0);
    anglez = (anglez) + (gyro.gyro.z - biasz) * (dt);
    
    prev_time = millis();

    return (int(anglez) % (360)) * (PI / 180);

}

// Define wheel dimensions in terms of meters. eg. 0.0325 m or 3.25 cm.
#define WHEELRADIUS 0.0325                //3.25 cm
#define CIRCUMFERENCE (2*PI*WHEELRADIUS)    //20.42 cm
#define TICK (CIRCUMFERENCE/8)              //2.55 cm

//Digital pins that can use interrupts.
//Wheel encoder pins.
#define LEFT_ENCODER_PIN 3 // Interrupt pin for the left wheel encoder
#define RIGHT_ENCODER_PIN 2 // Interrupt pin for the right wheel encoder




//Based on speed of sound this is constant to convert data from ultrasonics to meters.
#define SPEED_OF_SOUND (29.1*2*100) //2910 meters/ (10^-5 seconds) speed of sound twice due to
                         //distance there AND back.

//Defining pins for downfacing ultrasonic sensors
#define FORWARD_BOTTOM_ECHO_PIN 39
#define FORWARD_BOTTOM_TRIG_PIN 38

#define LEFT_BOTTOM_ECHO_PIN 30
#define LEFT_BOTTOM_TRIG_PIN 31

#define RIGHT_BOTTOM_ECHO_PIN 33
#define RIGHT_BOTTOM_TRIG_PIN 32

//Defining pins for front facing ultrasonic sensors.
//Used for detecting objects.
#define FORWARD_FRONT_ECHO_PIN 41
#define FORWARD_FRONT_TRIG_PIN 40

void setupUltraSonics(){
  // Setting up the input and output for the three ultrasonic sensors
  pinMode(FORWARD_BOTTOM_ECHO_PIN, INPUT);
  pinMode(FORWARD_BOTTOM_TRIG_PIN, OUTPUT);

  pinMode(LEFT_BOTTOM_ECHO_PIN, INPUT);
  pinMode(LEFT_BOTTOM_TRIG_PIN, OUTPUT);

  pinMode(RIGHT_BOTTOM_ECHO_PIN, INPUT);
  pinMode(RIGHT_BOTTOM_TRIG_PIN, OUTPUT);

  pinMode(FORWARD_FRONT_ECHO_PIN, INPUT);
  pinMode(FORWARD_FRONT_TRIG_PIN, OUTPUT);

}

//Returns distance read by ultrasonic sensor in meters.
double readUltraSonic(int TRIG_PIN, int ECHO_PIN){
  digitalWrite(TRIG_PIN, HIGH);
  digitalWrite(TRIG_PIN, LOW);

  //Gets amount of time for sound wave to bounce back.
  //Measured in 10^-5 seconds.
  double duration = pulseIn(ECHO_PIN, HIGH);
  
  //Calculate the distance (in cm) based on the speed of sound.
  double distance = duration / SPEED_OF_SOUND ;

  return distance;
}

//Processing data and command functions.
#define TABLE_DISTANCE 0.10
#define OBJECT_DANGER_DISTANCE 0.10

bool STOP = false;

/*
 * Tells if an edge was detected.
 * @param distanceAway Distance that ultrasonic sensor measures. 
 * @return true if there is an edge, false otherwise.
  */
bool detectsEdge(double distanceAway){
  if(distanceAway > TABLE_DISTANCE){
    return true;
  }else{
    return false;
  }
}

/*
 * Tells if an object was detected.
 * @param distanceAway Distance that ultrasonic sensor measures. 
 * @return true if there is an object, false otherwise.
  */
bool detectsObject(double distanceAway){
  if(distanceAway < TABLE_DISTANCE){
    return true;
  }else{
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  setupMotors();
  configureIMU();
  setupUltraSonics();
  bool success = turnLeftBoth();
  Serial.println(success);
  Serial1.println(success);
}

void loop() {

}
