#include <Adafruit_LSM9DS0.h>

//Constants for Reading Sensors Wirelessly
double anglez = 0;
double dt = 0;
double prev_time = 0;
double biasz = 0;
             // in radians

// Create Adafruit_LSM9DS0 object
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

void setup() {
  Serial.begin(9600);
    /* Initialise the sensor */
    if(!lsm.begin()) {
        while(1);
    }
  
    /* Setup the sensor gain and integration time */
    configureSensor();

      for(int i = 0; i < 100; i++) {
        sensors_event_t accel, mag, gyro, temp;
        lsm.getEvent(&accel, &mag, &gyro, &temp);
        biasz = gyro.gyro.z + biasz;
        Serial.println(biasz);
    } 

    biasz = biasz / 100.;

}

void loop() {
    acceRead();

    double theta_current = (int(anglez) % (360)) * (PI / 180);
    //Serial.println(theta_current);
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

void acceRead() {
    // Printing out the anglular direction of the car ("Angle Z" from the gyroscope).
    sensors_event_t accel, mag, gyro, temp;

    lsm.getEvent(&accel, &mag, &gyro, &temp);

    dt = (millis() - prev_time) / (1000.0);
    anglez = (anglez) + (gyro.gyro.z - biasz) * (dt);

    prev_time = millis();

}
