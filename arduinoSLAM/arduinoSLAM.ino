// Base libraries
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_LSM9DS0.h>     // Needed for accel/mag/gyro
#include <Adafruit_Sensor.h>      // Needed to convert values to usable units i.e. m/s^2, Gs, deg/s

#include <Servo.h>



// Motor driver shield library
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"



//Digital pins that can use interrupts.
//Wheel encoder pins.
#define encoderL 3 // Interrupt pin for the left wheel encoder
#define encoderR 2 // Interrupt pin for the right wheel encoder

double r_wheel = .0325; // 0.0325 meters (The diameter of the wheel was measured to be 6.5 cm. So, we divided by two to find the radius which gave us 3.25 cm. For calculation purposes, we will leave everything in meters.



//Current gyroscopic angle in range 0-2pi.
double angle = 0*PI;

//Circumference of one wheel in meters.
#define CIRCUMFERENCE 2*PI*r_wheel // This is to calculate the circumference of the wheel.

//1/8 Circumference
const double tickDistance = CIRCUMFERENCE/8; // Circumference of a wheel in meters = 2*PI*radius of the wheel. Because the car wil be traveling around a table, it makes more sense to measure distance traveled in meters.

//Flag to check if bot is turning.
//Need to update in turn function.
bool turning = false;

//Position data for left encoder.
double xLeft = 0;
double yLeft = 0;

//Position data for right encoder.
double xRight = 0;
double yRight = 0;



// Echo and Trig For Ultrasonic Sensor #1 --> Ultrasonic Sensor on the Bottom (Looking at the car from the front)
#define echoPin1 39
#define trigPin1 38


// Echo and Trig For Ultrasonic Sensor #2 --> Ultrasonic Sensor on the Right (Looking at the car from behind the LED)
#define echoPin2 30
#define trigPin2 31

// Echo and Trig For Ultrasonic Sensor #3 --> Ultrasonic Sensor on the Left (Looking at the car from behind the LED)
#define echoPin3 33
#define trigPin3 32


// Echo and Trig For Ultrasonic Sensor #4 --> Ultrasonic Sensor on the Front (Looking at the car from the front)
#define echoPin4 41
#define trigPin4 40


//Digital pins that can use interrupts.
//Wheel encoder pins.
#define encoderL 3 // Interrupt pin for the left wheel encoder
#define encoderR 2 // Interrupt pin for the right wheel encoder

//Defining the pin for the LED
#define led 24


//Constants for Turning Car
double L_wheelbase = .1;                // ? meters;
double k_motor = 1;                     // in units of [(rad/s) / (duty_cycle)]. duty_cycle should range from -255 to 255.
double dt_turning = 100;                // in milliseconds;
double fudge_factor = 1;
double maxspeed = 255;
double minspeed = 60;
double speedlimit = 64;
double theta_next = PI/2;
double theta_current = 0;               // in radians
double motor_drive_forward;             // in PWM (0 to 256)
double prev_time_turning = 0;
double motor_drive_L, motor_drive_R;

//Constants for Reading Sensors Wirelessly
double anglez = 0;
double dt = 0;
double prev_time = 0;
int i = 0;
double biasz = 0;
double currentGyro = 0;
double duration1;
double distance1;
double duration2;
double distance2;
double duration3;
double distance3;
double duration4;
double distance4;


// Create Adafruit_LSM9DS0 object
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
int acceReadCounter = 1;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);


void setup() {
    // Open up Bluetooth Virtual Serial1 COMM port
    Serial.begin(9600);
    Serial1.begin(9600);

    setupEncoders(); 

 

    // Setting up the input and output for the three ultrasonic sensors
    pinMode(echoPin1, INPUT);
    pinMode(trigPin1, OUTPUT);

    pinMode(echoPin2, INPUT);
    pinMode(trigPin2, OUTPUT);

    pinMode(echoPin3, INPUT);
    pinMode(trigPin3, OUTPUT);

    pinMode(echoPin4, INPUT);
    pinMode(trigPin4, OUTPUT);

    #ifndef ESP8266
      while(!Serial1);     // will pause Zero, Leonardo, etc until Serial1 console opens
    #endif

    /* Initialise the sensor */
    if(!lsm.begin()) {
        while(1);
    }
  
    /* Setup the sensor gain and integration time */
    configureSensor();
  
    for(i = 0; i < 100; i++) {
        sensors_event_t accel, mag, gyro, temp;
        lsm.getEvent(&accel, &mag, &gyro, &temp);
        biasz = gyro.gyro.z + biasz;
    } 

    biasz = biasz / 100.;

    //LED will only turn on when something happens (i.e. Making sure code is run successfully).
    pinMode(led, OUTPUT);

    // set up turning algorithm
    theta_current = 0;

    // Set the speed to start, from 0 (off) to 255 (max speed)
    motor_drive_forward = 0; // 0-255

    // Set up motor driving
    AFMS.begin();  // create with the default frequency 1.6KHz
}


void loop() {
    readultrasonic();
    acceRead();
   
    
     leftMotor->setSpeed(60);
    leftMotor->run(FORWARD);
    
    rightMotor->setSpeed(60);
    rightMotor->run(BACKWARD);

    //Updating the new angle estimate
    sensors_event_t accel, mag, gyro, temp;

    lsm.getEvent(&accel, &mag, &gyro, &temp);
    dt = (millis() - prev_time) / (1000.0);
    anglez = (anglez) + (gyro.gyro.z - biasz) * (dt);

    prev_time = millis();

    theta_current = (int(anglez) % (360)) * (PI / 180);
    //Serial.print(" current gyro angle: ");
    //Serial.print( theta_current );
    //Serial.print("\t\t");

    // Implementing the turning algorithim
    //theta_next = mapfloat(pot_value, 0, 1023, 0, 2 * PI); // min = 0 rad. max = 2 PI rad

  

    if( (distance1 > 3.9) || (distance2 > 3.9) || (distance3 > 3.9) || (distance4 < 12.50) ) { 
        turn(theta_next, theta_current, motor_drive_forward);
        //Serial.print( theta_next );
       // Serial.println(" " );

      Serial.print("EVENT,");
      Serial.print(distance1);
      Serial.print(",");
      Serial.print(distance2);
      Serial.print(",");
      Serial.print(distance3);
      Serial.print(",");
      Serial.print(distance4);
      Serial.print(",");
      Serial.print(theta_current);
      Serial.print(",");
      Serial.print(xLeft); 
      Serial.print(","); 
      Serial.print(-1*yLeft); 
      Serial.print(","); 
      Serial.print(xRight); 
      Serial.print(","); 
      Serial.println(yRight); 
      

    }
 
}


//Function that will display the sensor details (we're only using the angle so other parameters will not matter)
void displaySensorDetails(void) {
    sensor_t accel, mag, gyro, temp;
    lsm.getSensor(&accel, &mag, &gyro, &temp);
    delay(250);
}


//Configuring the gyroscope for a specific range
void configureSensor(void) {
    // 1.) Set the accelerometer range
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  
    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);

    // 3.) Setup the gyroscope
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}


//Turn Function => Allows car to turn
void turn( double theta_next, double theta_current, double motor_drive_forward ) {
    // calculate amount to turn
    double theta_dot, turn_adjust, motor_drive_L, motor_drive_R;
    theta_dot = theta_next - theta_current; //in radians

    turn_adjust = fudge_factor * L_wheelbase / 2.0 * theta_dot / (k_motor * r_wheel * (dt_turning / 1000)); //in units of duty cycle,between -255 to 255
    motor_drive_L = constrain( motor_drive_forward + turn_adjust, -speedlimit, speedlimit);
    motor_drive_R = constrain( motor_drive_forward - turn_adjust, -speedlimit, speedlimit);

    // Send command to motors
    //Serial.print(" Motor_drive_L: ");
    if(motor_drive_L > 0) {
        leftMotor->setSpeed(mapfloat(motor_drive_L, 0, maxspeed, minspeed, maxspeed));
        leftMotor->run(FORWARD);
        //Serial.print(mapfloat(motor_drive_L, 0, maxspeed, minspeed, maxspeed));
    }

    else {
        leftMotor->setSpeed( mapfloat(motor_drive_L, 0, -maxspeed, minspeed, maxspeed));
        leftMotor->run(BACKWARD);
        //Serial.print(mapfloat(motor_drive_L, 0, -maxspeed, minspeed, maxspeed));
    }
    
    //Serial.print(" Motor_drive_R: ");
    if(motor_drive_R < 0) {
        rightMotor->setSpeed(mapfloat(motor_drive_R, 0, -maxspeed, minspeed, maxspeed));
        rightMotor->run(BACKWARD);
        //Serial.print(mapfloat(motor_drive_R, 0, -maxspeed, minspeed, maxspeed));
    }
  
    else {
        rightMotor->setSpeed(mapfloat(motor_drive_R, 0, maxspeed, minspeed, maxspeed));
        rightMotor->run(FORWARD);
        //Serial.print(mapfloat(motor_drive_R, 0, maxspeed, minspeed, maxspeed));
    }

    //leftMotor->run(RELEASE);  //This is to unhooks the motor from the controller so you won't be able to control the wheels
    //rightMotor->run(RELEASE);

    //Serial.print(" theta_dot: ");
    //Serial.print( theta_dot );

    //Serial.print(" turn adjust: ");
    //Serial.print( turn_adjust );

    prev_time_turning = millis();
}


//Float maps max/min speed to actual values
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


void readultrasonic() {
    //Ultrasonic Sensor #1 (front)
    /* The following trigPin/echoPin cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. */
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin1, HIGH);
    digitalWrite(trigPin1, LOW);
    duration1 = pulseIn(echoPin1, HIGH);
    //Calculate the distance (in cm) based on the speed of sound.
    distance1 = duration1 / (29.1*2);

    //Serial.print(" Distance 1: ");
    //Serial.print(distance1);
    //Serial.print (",");

    

    //Ultrasonic Sensor #2 (On the right hand side)
    digitalWrite(trigPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin2, HIGH);
    digitalWrite(trigPin2, LOW);
    //Time pulse takes to bounce back into the sensor
    duration2 = pulseIn(echoPin2, HIGH);
    //calcuate the distance in cm for the time
    distance2 = duration2 / 58.2;
    
    //Serial.print("Distance 2: ");
    //Serial.print(distance2);
    //Serial.print(",");



    //Ultrasonic Sensor #3 (On the left hand side)
    digitalWrite(trigPin3, LOW);
    delayMicroseconds(3);
    digitalWrite(trigPin3, HIGH);
    digitalWrite(trigPin3, LOW);
    // Time pulse takes to bounce back into the sensor
    duration3 = pulseIn(echoPin3, HIGH);
    // Calcuate the distance in cm for the time
    
    distance3 = duration3 / 58.2;
    //Serial.print("Distance 3: ");
    //Serial.println(distance3);



    //Ultrasonic Sensor #4 (Front - bottom)
    digitalWrite(trigPin4, LOW);
    delayMicroseconds(3);
    digitalWrite(trigPin4, HIGH);
    digitalWrite(trigPin4, LOW);
    // Time pulse takes to bounce back into the sensor
    duration4 = pulseIn(echoPin4, HIGH);
    // Calcuate the distance in cm for the time
    
    distance4 = duration4 / 58.2;
    //Serial.print("Distance 4: ");
    //Serial.println(distance4);  
}


void acceRead() {
    // Printing out the anglular direction of the car ("Angle Z" from the gyroscope).
    sensors_event_t accel, mag, gyro, temp;

    lsm.getEvent(&accel, &mag, &gyro, &temp);

    dt = (millis() - prev_time) / (1000.0);
    anglez = (anglez) + (gyro.gyro.z - biasz) * (dt);

    prev_time = millis();

    //Serial.print(  abs (int(anglez) * (-1)  % (360) ) );
    //Serial1.print(  abs (int(anglez) * (-1)  % (360))  );
}



//Setup encoders.
void setupEncoders(){
  //Set left and right encoder pins to be pull resistors.
  //As reccomended in encoder datasheet.
  pinMode(encoderL, INPUT);
  pinMode(encoderR, INPUT);
  digitalWrite(encoderL, HIGH);
  digitalWrite(encoderR, HIGH);
  
  //Convert digital pin numbers to interrupt pin numbers.
  int encoderLeftInterrupt = digitalPinToInterrupt(encoderL);   //Interrupt pin 3.
  int encoderRightInterrupt = digitalPinToInterrupt(encoderR);  //Interrupt pin 2.
  
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
    //Calculate x and y components.
    double xDist = tickDistance * cos(theta_current);
    double yDist = tickDistance * sin(theta_current);
    
    //Add components to position data.
    xRight += xDist;
    yRight += yDist;
      
    /*
    //Serial.print("xRight: "); 
    Serial.print(xRight); 
    //Serial.print(" "); 
    Serial.print("yRight: "); 
    Serial.println(yRight); 
    */
  }
}


//Interrupt service routine for left encoder.
void encoderLeftDist(){
  //If we are turning do nothing.
  if(turning == false){
    //Calculate x and y components.
    double xDist = tickDistance * cos(theta_current);
    double yDist = tickDistance * sin(theta_current);
    
    //Add components to position data.
    xLeft += xDist;
    yLeft += yDist;


    /*
    Serial.print("xLeft: "); 
    Serial.print(xLeft); 
    Serial.print("   "); 
    Serial.print("yLeft: "); 
    Serial.print(yLeft);
    Serial.print("   ");
    Serial.print("theta current:  "); 
    Serial.println(theta_current); 
    */
    
  }

  
}



