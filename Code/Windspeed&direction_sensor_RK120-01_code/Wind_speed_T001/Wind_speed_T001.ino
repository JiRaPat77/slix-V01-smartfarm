unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;    // the debounce time; increase if the output flickers
unsigned long previousMillis_ = 0;
int pinInterrupt = 15;
const unsigned long interval_1 = 60000*2;
 
int Count = 0;
 
void onChange()
{
  if ( digitalRead(pinInterrupt) == LOW )
    Count++;
}
 

void setup()
{
  Serial.begin(115200); //Initialize serial port
  pinMode( pinInterrupt, INPUT_PULLUP);// set the interrupt pin
 
  //Enable
  attachInterrupt( digitalPinToInterrupt(pinInterrupt), onChange, FALLING);
 
  
}




void loop(){
    windSpeed();

  
}
void windSpeed(){
  if ((millis() - lastDebounceTime) > debounceDelay){
    
    lastDebounceTime = millis();
    
    Serial.print((Count * 8.75)/100);
    Serial.println("m/s");
    Count = 0;
    
  }
  delay(1);
}
