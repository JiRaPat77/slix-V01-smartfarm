// The bucket tested with this code is summarized here:
// http://texaselectronics.com/media/mconnect_uploadfiles/t/r/tr-525i_rainfall_user_s_manual.pdf

// Testing shows this bucket has a normally open reed switch.

#define RAIN_PIN 17          // interrupt pin
int CALC_INTERVAL = 1000;  // increment of measurements
#define DEBOUNCE_TIME 80    // time * 1000 in microseconds required to get through bounce noise

unsigned long nextCalc;
unsigned long timer;

volatile unsigned int rainTrigger = 0;
volatile unsigned long last_micros_rg;
float litersPerTip = 0.2;
float rainfallPerMinute;
unsigned long lastMinuteMillis = 0;

void setup() {
  Serial.begin(115200); 
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), countingRain, FALLING); 
  
  pinMode(RAIN_PIN, INPUT_PULLUP);
  nextCalc = millis() + CALC_INTERVAL;
}

void loop() {
  timer = millis();
  if(timer > nextCalc) {
    nextCalc = timer + CALC_INTERVAL;
    Serial.print("Total Tips: ");
    Serial.println((float) rainTrigger);

    if (timer - lastMinuteMillis >= 60000){
      rainfallPerMinute = rainTrigger * litersPerTip;
      Serial.print("Rainfall per minute: ");
      Serial.print(rainfallPerMinute);
      Serial.println(" mm/min");
      rainTrigger = 0;
      lastMinuteMillis = timer;
    }
         
  }
}

void countingRain() {
  // ATTEMPTED: Check to see if time since last interrupt call is greater than 
  // debounce time. If so, then the last interrupt call is through the 
  // noisy period of the reed switch bouncing, so we can increment by one.   
  if((long)(micros() - last_micros_rg) >= DEBOUNCE_TIME * 1000) { 
   rainTrigger += 1;
   last_micros_rg = micros();
  }  
}
