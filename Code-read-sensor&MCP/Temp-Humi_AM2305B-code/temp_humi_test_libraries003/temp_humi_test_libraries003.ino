#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


#define DHTPIN 14
#define DHTTYPE DHT22

DHT_Unified dht(DHTPIN, DHTTYPE);




uint32_t delayMS;


void initial TempHumi(){
  Serial.begin(115200);
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
}



void TempHumi(){
  delay(delayMS);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } 
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
}



//void setup() {
//  Serial.begin(115200);
//  dht.begin();
//  Serial.println(F("DHTxx Unified Sensor Example"));
//  sensor_t sensor;
//  dht.temperature().getSensor(&sensor);
//  dht.humidity().getSensor(&sensor);
//  delayMS = sensor.min_delay / 1000;
//}


//void loop() {
//  
//  
//
//}
