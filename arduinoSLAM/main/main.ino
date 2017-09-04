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


//Flag to check if bot is turning.
//Need to update in turn function.
bool turning = false;

//Position data for left encoder.
int leftWheelTicks = 0;

//Position data for right encoder.
int rightWheelTicks = 0;

//Setup encoders.
void setupEncoders(){
  //Set left and right encoder pins to be pull resistors.
  //As reccomended in encoder datasheet.
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  digitalWrite(LEFT_ENCODER_PIN, HIGH);
  digitalWrite(RIGHT_ENCODER_PIN, HIGH);
  
  //Convert digital pin numbers to interrupt pin numbers.
  int encoderLeftInterrupt = digitalPinToInterrupt(LEFT_ENCODER_PIN);   //Interrupt pin 3.
  int encoderRightInterrupt = digitalPinToInterrupt(RIGHT_ENCODER_PIN);  //Interrupt pin 2.
  
  //Attach interrupts to left and right encoders that listen for a change on the input.
  //ie. When input changes from 0 to 1 or 1 to 0, it will run the code attached, interrupting any
  //current process.
  attachInterrupt(encoderLeftInterrupt, encoderLeftDist, CHANGE);
  attachInterrupt(encoderRightInterrupt, encoderRightDist, CHANGE);
}

//Interrupt service routine for right encoder
void encoderRightDist(){
  //If we are turning do nothing.
  if(turning == false){
    //Add one tick to wheel distance.
    rightWheelTicks++;
  }
}


//Interrupt service routine for left encoder.
void encoderLeftDist(){
  //If we are turning do nothing.
  if(turning == false){
    //Add one tick to wheel distance.
    leftWheelTicks++;

  }
}

//Based on speed of sound this is constant to convert data from ultrasonics to meters.
#define SPEED_OF_SOUND (29.1*2*100) //2910 meters/ (10^-5 seconds) speed of sound twice due to
                         //distance there AND back.

//Defining pins for downfacing ultrasonic sensors
#define FORWARD_BOTTOM_ECHO_PIN 39
#define FORWARD_BOTTOM_TRIG_PIN 38

#define LEFT_BOTTOM_ECHO_PIN 33
#define LEFT_BOTTOM_TRIG_PIN 32

#define RIGHT_BOTTOM_ECHO_PIN 30
#define RIGHT_BOTTOM_TRIG_PIN 31

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

void setupBluetooth(){
  Serial1.begin(9600);
  #ifndef ESP8266
    while(!Serial1);     // will pause Zero, Leonardo, etc until Serial1 console opens
  #endif
}



void setup(){
  Serial.begin(9600);
  setupMotors();
  configureIMU();
  setupEncoders();
  setupUltraSonics();
  setupBluetooth();
  
}

void loop(){
  double currentAngle = gyroRead();
  
  double forwardBottomDistance = readUltraSonic(FORWARD_BOTTOM_TRIG_PIN, FORWARD_BOTTOM_ECHO_PIN);
  double leftBottomDistance = readUltraSonic(LEFT_BOTTOM_TRIG_PIN, LEFT_BOTTOM_ECHO_PIN);
  double rightBottomDistance = readUltraSonic(RIGHT_BOTTOM_TRIG_PIN, RIGHT_BOTTOM_ECHO_PIN);
  double forwardFrontDistance = readUltraSonic(FORWARD_FRONT_TRIG_PIN, FORWARD_FRONT_ECHO_PIN);

  int leftWheelTicksAgain = leftWheelTicks;
  int rightWheelTicksAgain = rightWheelTicks;

  moveForward();
  if(forwardBottomDistance > 0.10 || forwardFrontDistance < 0.10){
    motorStop();
  }
}



