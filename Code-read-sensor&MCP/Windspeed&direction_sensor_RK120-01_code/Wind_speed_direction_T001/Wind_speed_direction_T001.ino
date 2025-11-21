#include <Adafruit_GFX.h>
int dir_pin = 26;
int pinInterrupt = 15;
int Count = 0;

// Wind speed
 
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;    // the debounce time; increase if the output flickers
 
 
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
  delay(500);

  //direction
  pinMode(dir_pin,INPUT);
  Serial.begin(115200);
  
  
}
 
void loop()
{
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    lastDebounceTime = millis();
    Serial.print(" Wind Speed ");
    Serial.print((Count * 8.75)/100);
    
    Count = 0;
    
    Serial.println(" m/s ");
    
  }
  //delay(1500);

  //direction
  int sensorValue = analogRead(dir_pin);
  
  int wind_direction = map(sensorValue, 0, 32767 , 0, 360);
  Serial.print("Direction : ");
  Serial.print(wind_direction);
  Serial.println(" degreee ");
  delay(1500); 
}
