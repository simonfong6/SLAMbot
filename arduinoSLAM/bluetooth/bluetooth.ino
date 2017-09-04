
void setup() {
  Serial.begin(9600);
  setupBluetooth();
}

void loop() {
  //Write to Matlab
  Serial1.println("I love big fat nuts");

  //Read from Matlab
  if(Serial1.available() > 0){
    Serial.print(Serial1.readString());
  }
}

void setupBluetooth(){
  Serial1.begin(9600);
  #ifndef ESP8266
    while(!Serial1);     // will pause Zero, Leonardo, etc until Serial1 console opens
  #endif
}

