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

void setup() {
  Serial.begin(9600);
  setupUltraSonics();
}

void loop() {
  printUltraSonics();
}

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

void printUltraSonics(){
  double forwardBottomDistance = readUltraSonic(FORWARD_BOTTOM_TRIG_PIN, FORWARD_BOTTOM_ECHO_PIN);
  double leftBottomDistance = readUltraSonic(LEFT_BOTTOM_TRIG_PIN, LEFT_BOTTOM_ECHO_PIN);
  double rightBottomDistance = readUltraSonic(RIGHT_BOTTOM_TRIG_PIN, RIGHT_BOTTOM_ECHO_PIN);
  double forwardFrontDistance = readUltraSonic(FORWARD_FRONT_TRIG_PIN, FORWARD_FRONT_ECHO_PIN);

  Serial.print("forwardBottom = ");
  Serial.print(forwardBottomDistance);
  withEqual("left");
  Serial.print(leftBottomDistance);
  withEqual("right");
  Serial.print(rightBottomDistance);
  withEqual("forwardFront");
  Serial.println(forwardFrontDistance);
}

void commaSpace(){
  Serial.print(", ");
}

void withEqual(char string[]){
  commaSpace();
  Serial.print(string);
  Serial.print(" = ");
}

