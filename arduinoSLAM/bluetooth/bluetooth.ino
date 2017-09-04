
void setup() {
  setupBluetooth();
}

void loop() {
  
  Serial1.println("I love big fat nuts");
}

void setupBluetooth(){
  Serial1.begin(9600);
  #ifndef ESP8266
    while(!Serial1);     // will pause Zero, Leonardo, etc until Serial1 console opens
  #endif
}

