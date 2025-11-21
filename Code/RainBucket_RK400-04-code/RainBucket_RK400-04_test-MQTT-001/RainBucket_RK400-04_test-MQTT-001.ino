
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define RAIN_PIN 17          // interrupt pin
int CALC_INTERVAL = 1000;    // increment of measurements
#define DEBOUNCE_TIME 80     // time * 1000 in microseconds required to get through bounce noise

unsigned long nextCalc;
unsigned long timer;

volatile unsigned int rainTrigger = 0;
volatile unsigned long last_micros_rg;
float litersPerTip = 0.2;
float rainfallPerMinute;
unsigned long lastMinuteMillis = 0;

const char *ssid = "Weaverbase";
const char *password = "1212312121";

#define MQTT_SERVER "thingsboard.weaverbase.com" 
#define MQTT_PORT 1883
#define MQTT_USERNAME "7p2oxwi1ssz0zyybgo7o"
#define MQTT_PASSWORD "123456"
#define MQTT_NAME "PICO_W"
#define MQTT_TOPIC "v1/devices/me/telemetry"



WiFiClient client;
PubSubClient mqtt(client);

void setup() {
  Serial.begin(115200); 
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), countingRain, FALLING); 
  pinMode(RAIN_PIN, INPUT);
  nextCalc = millis() + CALC_INTERVAL;

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker with username and password
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  if (mqtt.connected() == false){
    Serial.print("MQTT connection...");
    if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)){
      Serial.print("connected");
    }else{
      Serial.println("connected");
      delay(5000);
    }
  }
//  while (!mqtt.connected()) {
//    if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
//      Serial.println("Connected to MQTT broker");
//    } else {
//      Serial.println("Failed, retrying in 5 seconds...");
//      delay(5000);
//    }
//  }
  
}

void loop() {
//  if (mqtt.connected() == false){
//    Serial.print("MQTT connection...");
//    if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)){
//      Serial.print("connected");
//    }else{
//      Serial.println("connected");
//      delay(5000);
//    }
//  }
  timer = millis();
  if (timer > nextCalc) {
    nextCalc = timer + CALC_INTERVAL;
    Serial.print("Total Tips: ");
    Serial.println((float) rainTrigger);

    if (timer - lastMinuteMillis >= 60000){
      rainfallPerMinute = rainTrigger * litersPerTip;
      Serial.print("Rainfall per minute: ");
      Serial.print(rainfallPerMinute);
      Serial.println(" mm/min");

      // ส่ง JSON string ไปที่ MQTT topic
      mqtt.loop();
      String dataJS = "{\"Rainfall per Min\":" + String(rainfallPerMinute,3) + "}";
      char json[100];
      dataJS.toCharArray(json,dataJS.length()+1);
      mqtt.publish("v1/devices/me/telemetry" , json);
      rainTrigger = 0;
      lastMinuteMillis = timer;
    }
  }

}

void countingRain() {
  if ((long)(micros() - last_micros_rg) >= DEBOUNCE_TIME * 1000) { 
    rainTrigger += 1;
    last_micros_rg = micros();
  }  
}
