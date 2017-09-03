// Define wheel dimensions in terms of meters. eg. 0.0325 m or 3.25 cm.
#define WHEELRADIUS 0.0325                //3.25 cm
#define CIRCUMFERENCE (2*PI*WHEELRADIUS)    //20.42 cm
#define (TICK CIRCUMFERENCE/8)              //2.55 cm

//Digital pins that can use interrupts.
//Wheel encoder pins.
#define LEFT_ENCODER_PIN 3 // Interrupt pin for the left wheel encoder
#define RIGHT_ENCODER_PIN 2 // Interrupt pin for the right wheel encoder

//Current gyroscopic angle in range 0-2pi.
double currentAngle = 0*PI;

//Flag to check if bot is turning.
//Need to update in turn function.
bool turning = false;

//Position data for left encoder.
double xLeft = 0;
double yLeft = 0;

//Position data for right encoder.
double xRight = 0;
double yRight = 0;


void setup() {
  Serial.begin(9600);
  setupEncoders();

}

void loop() {
  printEncoderData();
}


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
    //Calculate x and y components.
    double xDist = TICK * cos(currentAngle);
    double yDist = TICK * sin(currentAngle);
    
    //Add components to position data.
    xRight += xDist;
    yRight += yDist;
      
  }
}


//Interrupt service routine for left encoder.
void encoderLeftDist(){
  //If we are turning do nothing.
  if(turning == false){
    //Calculate x and y components.
    double xDist = TICK * cos(currentAngle);
    double yDist = TICK * sin(currentAngle);
    
    //Add components to position data.
    xLeft += xDist;
    yLeft += yDist;

  }
}

void printEncoderData(){
  Serial.print("xLeft = ");
  Serial.print(xLeft);
  withEqual("yLeft");
  Serial.print(yLeft);
  withEqual("xRight");
  Serial.print(xRight);
  withEqual("yRight");
  Serial.println(yRight);
}

void commaSpace(){
  Serial.print(", ");
}

void withEqual(char string[]){
  commaSpace();
  Serial.print(string);
  Serial.print(" = ");
}

