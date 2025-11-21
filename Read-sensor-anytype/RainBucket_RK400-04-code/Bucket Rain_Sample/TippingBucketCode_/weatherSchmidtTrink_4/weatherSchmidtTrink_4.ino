//const int RAIN_INT = 17;  

volatile int myTotal = 0;

void setup() {
  pinMode(17, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(17), myCount, FALLING);
  Serial.begin(115200);
}

void loop() {
 Serial.print("Count is: ");
 Serial.print(myTotal);
 Serial.print(" and interrupt is ");
 Serial.println(digitalRead(17));
 delay(1000);
}

void myCount() {
 myTotal +=1;
}
