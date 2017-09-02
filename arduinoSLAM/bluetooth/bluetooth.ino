// Base libraries
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>      // Needed to convert values to usable units i.e. m/s^2, Gs, deg/s


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  #ifndef ESP8266
    while(!Serial1);     // will pause Zero, Leonardo, etc until Serial1 console opens
  #endif

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial1.println("Hello World!");

}
