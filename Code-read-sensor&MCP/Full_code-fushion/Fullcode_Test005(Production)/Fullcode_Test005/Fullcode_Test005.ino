///////////////////////////////////////////// VARIABLE & INCLUDE ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////// SetUP WiFi & Publish //////////////////////////////////////////////
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <stdio.h>

const char *ssid = "Lucky Cat2";   //สวนหน้า
const char *password = "12344321";
// const char *ssid = "Wireless 2.4G";
// const char *password = "66666666"; //สวนหลัง

// const char *ssid = "Weaverbase";
// const char *password = "1212312121";  //Test office

#define MQTT_SERVER "thingsboard.weaverbase.com"
#define MQTT_PORT 1883
#define MQTT_USERNAME "7p2oxwi1ssz0zyybgo7o"
#define MQTT_PASSWORD "123456"
#define MQTT_NAME "PICO_W"
#define MQTT_TOPIC "v1/devices/me/telemetry"

WiFiClient client;
PubSubClient mqtt(client);
const unsigned long interval_1 = 60000 * 10;
const unsigned long interval_2 = 12 * 6 * 60 * 1000;
unsigned long previousMillis_ = 0;
volatile unsigned int rainTrigger = 0;

///////////////////////////////////////////// Var Scan I2c Pin address //////////////////////////////////////////////

#include <Wire.h>
#include <MCP342x.h>

/* Demonstrate the use of convertAndRead().*/
long value_1_1, value_2_1, value_3_1, value_4_1;
long value_1_2, value_2_2, value_3_2, value_4_2;
long value_1_3, value_2_3, value_3_3, value_4_3;

// 0x68 is the default address for all MCP342x devices
uint8_t address_mcp3424_1 = 0x68;
uint8_t address_mcp3424_2 = 0x6A;
uint8_t address_mcp3424_3 = 0x6C;

MCP342x adc1 = MCP342x(address_mcp3424_1);
MCP342x adc2 = MCP342x(address_mcp3424_2);
MCP342x adc3 = MCP342x(address_mcp3424_3);

///////////////////////////////////////////// Var Soil moisture-RK520-01 ////////////////////////////////////////////



///////////Variable of Soil moisture//////////////
const float Vmin = 0.0;
const float Vmax = 100.0;
const float soilMin = 0.0;
const float soilMax = 100.0;
///////////Variable of Soil temperature//////////////
int decimalPrecision = 2;        // decimal places for all values shown in LED Display & Serial Monitor
float voltageDividerR1 = 10000;  // Resistor value in R1 for voltage devider method
float BValue = 3950;             // The B Value of the thermistor for the temperature measuring range
float R1 = 6500;                 // Thermistor resistor rating at based temperature (25 degree celcius)
float T1 = 298.15;               /* Base temperature T1 in Kelvin (default should be at 25 degree)*/
float R2_0, R2_1, R2_2, R2_3;    /* Resistance of Thermistor (in ohm) at Measuring Temperature*/
float T2_0, T2_1, T2_2, T2_3;    /* Measurement temperature T2 in Kelvin */
float a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3;
float e = 2.718281828;
float tempLastSample_0 = 0, tempSampleSum_0 = 0, tempMean_0 = 0, tempSampleCount_0 = 1000;
float tempLastSample_1 = 0, tempSampleSum_1 = 0, tempMean_1 = 0, tempSampleCount_1 = 1000;
float tempLastSample_2 = 0, tempSampleSum_2 = 0, tempMean_2 = 0, tempSampleCount_2 = 1000;
float tempLastSample_3 = 0, tempSampleSum_3 = 0, tempMean_3 = 0, tempSampleCount_3 = 1000;
////////// soil temperature calc_1 ////////
int value_soucre_T0;
int value_soucreMAP_T0;
int valueTemp_0;
////////// soil temperature calc_2 ////////
int value_soucre_T1;
int value_soucreMAP_T1;
int valueTemp_1;
////////// soil temperature calc_3 ////////
int value_soucre_T2;
int value_soucreMAP_T2;
int valueTemp_2;
////////// soil temperature calc_4 ////////
int value_soucre_T3;
int value_soucreMAP_T3;
int valueTemp_3;
////////// soil moisture calc_1 ////////
int sensorValuesoil_0;
float voltageSoil_0;
float SoilMoisture_0;
//////////// soil moisture calc_2 ////////
int sensorValuesoil_1;
float voltageSoil_1;
float SoilMoisture_1;
//////////// soil moisture calc_3 ////////
int sensorValuesoil_2;
float voltageSoil_2;
float SoilMoisture_2;
//////////// soil moisture calc_4 ////////
int sensorValuesoil_3;
float voltageSoil_3;
float SoilMoisture_3;


///////////////////////////////////////// Var UV ///////////////////////////////////////////////////////
int mapUV;
float voltageUV;
float mapUV1;

///////////////////////////////////////// Var Publish MQTT ///////////////////////////////////////////////////////
String dataJS_1;
char json_1[300];
StaticJsonDocument<200> jsonDocument_1;
char jsonString_1[200];
String dataJS_2;
char json_2[300];
StaticJsonDocument<200> jsonDocument_2;
char jsonString_2[200];
String dataJS_3;
char json_3[100];
StaticJsonDocument<200> jsonDocument_3;
char jsonString_3[200];


///////////////////////////////////////// Var Temperature & Humidity-AM2305B //////////////////////////////////////

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 14
#define DHTTYPE DHT22
float Temperature_S;
float Humidity_S;
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;


///////////////////////////////////////// Var Wind directions & speed-RK120-01 //////////////////////////////////////
int pinInterrupt = 15;
int Count = 0;
float windSpeed;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;  // the debounce time; increase if the output flickers
int sensor_windDirectValue;
int wind_directions;


///////////////////////////////////////// Var RainBucket-RK400-04 //////////////////////////////////////////////////

#define RAIN_PIN 17
int CALC_INTERVAL = 1000;  // increment of measurements
#define DEBOUNCE_TIME 80   // time * 1000 in microseconds required to get through bounce noise

unsigned long nextCalc_rain;
unsigned long timer_rain;

volatile unsigned long last_micros_rg;
float litersPerTip = 0.2;
float rainfallPerMinute;
unsigned long lastMinuteMillis_rain = 0;



/////////////////////////////////////////////// Var Reconnect ////////////////////////////////////////////////////////
unsigned long lastResetAttempt = 0;
unsigned long reconnectInterval = 300000;






//////////////////////////////////////////////////// MAIN VOID ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  pinMode(18, OUTPUT);
  Serial.begin(115200);
  Wire.begin();


  connectWiFi();
  connect_MQTT();
  MCP3424_scan_initial();
  initial_TempHumi();
  initial_wind();
  initial_Rain();
}

void loop() {

  MCP3424_scan_loop();
  soilMoisture();
  TempHumi();
  UV_sensor();
  Wind();
  Rain_loop();

  /////////// Reconnect Wifi ///////////
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  /////////// Reconnect MQTT ///////////
  unsigned long currentMillis_ = millis();
  if (currentMillis_ - previousMillis_ >= interval_1) {
    Serial.print("Let Publish");
    connect_MQTT();

    // soil publish
    mqtt.loop();
    dataJS_1 = "{\"Soil Teamperature_1\":" + String(T2_0, 3) + ",\"Soil Teamperature_2\":" + String(T2_1, 3) + ",\"Soil Teamperature_3\":" + String(T2_2, 3) + ",\"Soil Teamperature_4\":" + String(T2_3, 3) + ",\"Soil Moisture_1\":" + String(voltageSoil_0, 3) + ",\"Soil Moisture_2\":" + String(voltageSoil_1, 3) + ",\"Soil Moisture_3\":" + String(voltageSoil_2, 3) + ",\"Soil Moisture_4\":" + String(voltageSoil_3, 3) + "}";
    dataJS_1.toCharArray(json_1, dataJS_1.length() + 1);
    mqtt.publish("v1/devices/me/telemetry", json_1);
    tempSampleSum_0 = 0;
    tempSampleCount_0 = 0;
    tempSampleSum_1 = 0;
    tempSampleCount_1 = 0;
    tempSampleSum_2 = 0;
    tempSampleCount_2 = 0;
    tempSampleSum_3 = 0;
    tempSampleCount_3 = 0;


    // UV publish
    //mqtt.loop();
    jsonDocument_1["UV value"] = mapUV1;
    serializeJson(jsonDocument_1, jsonString_1);
    mqtt.publish("v1/devices/me/telemetry", jsonString_1);


    // Temp&Humi publish
    //mqtt.loop();
    dataJS_2 = "{\"Teamperature\":" + String(Temperature_S, 3) + ",\"Humidity\":" + String(Humidity_S, 3) + "}";
    dataJS_2.toCharArray(json_2, dataJS_2.length() + 1);
    mqtt.publish("v1/devices/me/telemetry", json_2);


    // WInd speed&directions publish
    //mqtt.loop();
    jsonDocument_2["Wind speed"] = windSpeed;
    jsonDocument_2["Wind directions"] = wind_directions;
    serializeJson(jsonDocument_2, jsonString_2);
    mqtt.publish("v1/devices/me/telemetry", jsonString_2);
    Count = 0;


    // Rain publish
    //mqtt.loop();
    dataJS_3 = "{\"Rainfall per Min\":" + String(rainTrigger, 3) + "}";
    dataJS_3.toCharArray(json_3, dataJS_3.length() + 1);
    mqtt.publish("v1/devices/me/telemetry", json_3);
    lastMinuteMillis_rain = timer_rain;
    rainTrigger = 0;





    previousMillis_ = currentMillis_;
    digitalWrite(18, !digitalRead(18));
    mqtt.disconnect();
    Serial.print("Finished Publish");


  } else {
    // Reset Soil value
    tempSampleSum_0 = 0;
    tempSampleCount_0 = 0;
    tempSampleSum_1 = 0;
    tempSampleCount_1 = 0;
    tempSampleSum_2 = 0;
    tempSampleCount_2 = 0;
    tempSampleSum_3 = 0;
    tempSampleCount_3 = 0;

    // Reset Wind value
    Count = 0;

    // Reset Rain value
    //rainTrigger = 0;
  }

  Serial.print("");
  Serial.print("");

  // Reset 12 hour
  if (currentMillis_ - previousMillis_ >= interval_2) {
    Serial.print("12 Hour");
    resetPico();
  }
}







//////////////////////////////////////////////////// MCP3424 Scan /////////////////////////////////////////////////////////////////
void MCP3424_scan_initial() {

  MCP342x::generalCallReset();
  delay(1);  // MC342x needs 300us to settle, wait 1ms

  // Check device present

  Wire.requestFrom(address_mcp3424_1, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address_mcp3424_1, HEX);
    while (1)
      ;
  }
  Wire.requestFrom(address_mcp3424_2, (uint8_t)2);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address_mcp3424_2, HEX);
    while (1)
      ;
  }
  Wire.requestFrom(address_mcp3424_3, (uint8_t)3);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address_mcp3424_3, HEX);
    while (1)
      ;
  }
}


void MCP3424_scan_loop() {

  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err1_1 = adc1.convertAndRead(MCP342x::channel1, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_1_1, status);
  uint8_t err2_1 = adc1.convertAndRead(MCP342x::channel2, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_2_1, status);
  uint8_t err3_1 = adc1.convertAndRead(MCP342x::channel3, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_3_1, status);
  uint8_t err4_1 = adc1.convertAndRead(MCP342x::channel4, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_4_1, status);

  uint8_t err1_2 = adc2.convertAndRead(MCP342x::channel1, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_1_2, status);
  uint8_t err2_2 = adc2.convertAndRead(MCP342x::channel2, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_2_2, status);
  uint8_t err3_2 = adc2.convertAndRead(MCP342x::channel3, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_3_2, status);
  uint8_t err4_2 = adc2.convertAndRead(MCP342x::channel4, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_4_2, status);

  uint8_t err1_3 = adc3.convertAndRead(MCP342x::channel1, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_1_3, status);
  uint8_t err2_3 = adc3.convertAndRead(MCP342x::channel2, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_2_3, status);
  uint8_t err3_3 = adc3.convertAndRead(MCP342x::channel3, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_3_3, status);
  uint8_t err4_3 = adc3.convertAndRead(MCP342x::channel4, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1, 1000000, value_4_3, status);


  if (err1_1) {
    Serial.print("Convert error: ");
    Serial.println(err1_1);
    Serial.println(err2_1);
    Serial.println(err3_1);
    Serial.println(err4_1);
  }


  if (err1_2) {
    Serial.print("Convert error: ");
    Serial.println(err1_2);
    Serial.println(err2_2);
    Serial.println(err3_2);
    Serial.println(err4_2);
  }


  if (err1_3) {
    Serial.print("Convert error: ");
    Serial.println(err1_3);
    Serial.println(err2_3);
    Serial.println(err3_3);
    Serial.println(err4_3);
  }
}



//////////////////////////////////////////////////// SOIL MOISTURE & TEMPERATURE VOID ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void soilMoisture() {

  //////// Temperature calc_1 ////////
  value_soucre_T0 = value_1_3;
  value_soucreMAP_T0 = map(value_soucre_T0, 0, 32767, 32767, 0);
  valueTemp_0 = value_soucreMAP_T0;
  if (millis() >= tempLastSample_0 + 1) {
    tempSampleSum_0 = tempSampleSum_0 + valueTemp_0;
    tempSampleCount_0 = tempSampleCount_0 + 1;
    tempLastSample_0 = millis();
  }

  tempMean_0 = tempSampleSum_0 / tempSampleCount_0;
  R2_0 = ((voltageDividerR1 * tempMean_0) / (32767 - tempMean_0));
  a0 = 1 / T1;
  b0 = log10(R1 / R2_0);
  c0 = b0 / log10(e);
  d0 = c0 / BValue;
  T2_0 = 1 / (a0 - d0);
  T2_0 = T2_0 - 273.15, decimalPrecision;


  //////// Temperature calc_2 ////////
  value_soucre_T1 = value_3_3;
  value_soucreMAP_T1 = map(value_soucre_T1, 0, 32767, 32767, 0);
  valueTemp_1 = value_soucreMAP_T1;
  if (millis() >= tempLastSample_1 + 1) {
    tempSampleSum_1 = tempSampleSum_1 + valueTemp_1;
    tempSampleCount_1 = tempSampleCount_1 + 1;
    tempLastSample_1 = millis();
  }

  tempMean_1 = tempSampleSum_1 / tempSampleCount_1;
  R2_1 = ((voltageDividerR1 * tempMean_1) / (32767 - tempMean_1));
  a1 = 1 / T1;
  b1 = log10(R1 / R2_1);
  c1 = b1 / log10(e);
  d1 = c1 / BValue;
  T2_1 = 1 / (a1 - d1);
  T2_1 = T2_1 - 273.15, decimalPrecision;



  ////////// Temperature calc_3 ////////
  value_soucre_T2 = value_1_1;
  value_soucreMAP_T2 = map(value_soucre_T2, 0, 32767, 32767, 0);
  valueTemp_2 = value_soucreMAP_T2;
  if (millis() >= tempLastSample_2 + 1) {
    tempSampleSum_2 = tempSampleSum_2 + valueTemp_2;
    tempSampleCount_2 = tempSampleCount_2 + 1;
    tempLastSample_2 = millis();
  }

  tempMean_2 = tempSampleSum_2 / tempSampleCount_2;
  R2_2 = ((voltageDividerR1 * tempMean_2) / (32767 - tempMean_2));
  a2 = 1 / T1;
  b2 = log10(R1 / R2_2);
  c2 = b2 / log10(e);
  d2 = c2 / BValue;
  T2_2 = 1 / (a2 - d2);
  T2_2 = T2_2 - 273.15, decimalPrecision;





  //////// Temperature calc_4 ////////
  value_soucre_T3 = value_3_1;
  value_soucreMAP_T3 = map(value_soucre_T3, 0, 32767, 32767, 0);
  valueTemp_3 = value_soucreMAP_T3;
  if (millis() >= tempLastSample_3 + 1) {
    tempSampleSum_3 = tempSampleSum_3 + valueTemp_3;
    tempSampleCount_3 = tempSampleCount_3 + 1;
    tempLastSample_3 = millis();
  }

  tempMean_3 = tempSampleSum_3 / tempSampleCount_3;
  R2_3 = ((voltageDividerR1 * tempMean_3) / (32767 - tempMean_3));
  a3 = 1 / T1;
  b3 = log10(R1 / R2_3);
  c3 = b3 / log10(e);
  d3 = c3 / BValue;
  T2_3 = 1 / (a3 - d3);
  T2_3 = T2_3 - 273.15, decimalPrecision;



  ////////// soil moisture calc_1 ////////
  sensorValuesoil_0 = value_2_3;
  voltageSoil_0 = sensorValuesoil_0 * (Vmax - Vmin) / 32767.0 + Vmin;

  //SoilMoisture_0 = (voltageSoil_0 - Vmin) / (Vmax - Vmin) * (soilMax - soilMin) + soilMin;


  //////////// soil moisture calc_2 ////////
  sensorValuesoil_1 = value_4_3;
  voltageSoil_1 = sensorValuesoil_1 * (Vmax - Vmin) / 32767.0 + Vmin;

  //SoilMoisture_1 = (voltageSoil_1 - Vmin) / (Vmax - Vmin) * (soilMax - soilMin) + soilMin;



  //////////// soil moisture calc_3 ////////
  sensorValuesoil_2 = value_2_1;
  voltageSoil_2 = sensorValuesoil_2 * (Vmax - Vmin) / 32767.0 + Vmin;

  //SoilMoisture_2 = (voltageSoil_2 - Vmin) / (Vmax - Vmin) * (soilMax - soilMin) + soilMin;



  //////////// soil moisture calc_4 ////////
  sensorValuesoil_3 = value_4_1;
  voltageSoil_3 = sensorValuesoil_3 * (Vmax - Vmin) / 32767.0 + Vmin;

  //SoilMoisture_3 = (voltageSoil_3 - Vmin) / (Vmax - Vmin) * (soilMax - soilMin) + soilMin;



  /////////// PRINT CHECK VALUE //////////////
  Serial.print("Soil Teamperature_1 : ");
  Serial.print(T2_0);
  Serial.print("Soil Teamperature_2 : ");
  Serial.print(T2_1);
  Serial.print("Soil Teamperature_3 : ");
  Serial.print(T2_2);
  Serial.print("Soil Teamperature_4 : ");
  Serial.print(T2_3);
  Serial.print("Soil Moisture_1 : ");
  Serial.print(voltageSoil_0);
  Serial.print("Soil Moisture_2 : ");
  Serial.print(voltageSoil_1);
  Serial.print("Soil Moisture_3 : ");
  Serial.print(voltageSoil_2);
  Serial.print("Soil Moisture_4 : ");
  Serial.println(voltageSoil_3);
}


///////////////////////////////////////// Temperature & Humidity-AM2305B VOID /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initial_TempHumi() {
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
}


void TempHumi() {

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    Temperature_S = event.temperature;
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  } else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    Humidity_S = event.relative_humidity;
  }
}





///////////////////////////////////////// UV_sensor-RK200-04 Void ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void UV_sensor() {
  mapUV = value_3_2;
  voltageUV = (mapUV * (5.0 / 32767.0)) * 1000;
  //  Serial.print("milliV : ");
  //  Serial.println(voltageUV);
  mapUV1 = map(voltageUV, 0, 5000, 0, 11);  //5000 คือค่า Max สุดของหำืหนพส่งมาเอาไปคำนวณ (mapUV * (5.0/32767.0))*1000 จะได้ 5000 เอามาแมพกับ 0-10
  Serial.print("UVindex : ");
  Serial.println(mapUV1);
}




///////////////////////////////////////// Wind directions & speed-RK120-01 VOID //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void onChange() {
  if (digitalRead(pinInterrupt) == LOW)
    Count++;
}


void initial_wind() {

  pinMode(pinInterrupt, INPUT_PULLUP);  // set the interrupt pin
  attachInterrupt(digitalPinToInterrupt(pinInterrupt), onChange, FALLING);
}


void Wind() {
  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    windSpeed = (Count * 8.75) / 100;
    Serial.print("Wind Speed : ");
    Serial.println(windSpeed);
  }

  //direction
  sensor_windDirectValue = value_4_2;
  wind_directions = map(sensor_windDirectValue, 0, 32767, 0, 360);
  Serial.print("Wind Directions : ");
  Serial.println(wind_directions);
}





///////////////////////////////////////// RainBucket-RK400-04 VOID //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initial_Rain() {

  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), countingRain, RISING);

  pinMode(RAIN_PIN, INPUT);
  nextCalc_rain = millis() + CALC_INTERVAL;
}


void Rain_loop() {

  timer_rain = millis();
  if (timer_rain > nextCalc_rain) {
    nextCalc_rain = timer_rain + CALC_INTERVAL;
    Serial.print("Total Tips: ");
    Serial.println((int)rainTrigger);
  }
}
void countingRain() {
  if ((long)(micros() - last_micros_rg) >= DEBOUNCE_TIME * 1000) {
    rainTrigger += 1;
    last_micros_rg = micros();
  }
}





/////////////////////////////////////////////// ResetNormal VOID ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void resetPico() {
  Serial.print("Resetting Pi Pico...");
  NVIC_SystemReset();
}



/////////////////////////////////////////////// Reconnect VOID //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void connectWiFi() {

  unsigned long currentMillis = millis();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Attempting to reconnect to WiFi...");

    // รีเซ็ตเวลาล่าสุดของการลองเชื่อมต่อใหม่
    lastResetAttempt = currentMillis;

    // รอจนกว่าจะเชื่อมต่อ Wi-Fi สำเร็จหรือถึงเวลาที่กำหนด เกิน 5 นาทีให้ reset
    while (WiFi.status() != WL_CONNECTED && currentMillis - lastResetAttempt < reconnectInterval) {
      WiFi.begin(ssid, password);
      delay(1000);
      Serial.println("Connecting to WiFi...");
      currentMillis = millis();
    }
    Serial.println("Connected to WiFi");

    // ถ้ายังไม่ได้เชื่อมต่อ Wi-Fi หลังจากเวลาที่กำหนด ให้ทำการรีเซ็ตบอร์ด
    if (WiFi.status() != WL_CONNECTED && currentMillis - lastResetAttempt >= reconnectInterval) {
      resetPico();
    }
  } else {
    lastResetAttempt = 0;
  }
  digitalWrite(18, 1);
  delay(300);
  digitalWrite(18, 0);
  delay(100);
  digitalWrite(18, 1);
  delay(300);
  digitalWrite(18, 0);
  delay(100);
  digitalWrite(18, 1);
  delay(100);
  digitalWrite(18, 0);
}


/////////////////////////////////////////////// CONNECT MQTT VOID ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void connect_MQTT() {

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  if (mqtt.connected() == false) {
    Serial.print("MQTT connection...");
    if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.print("connected");
    } else {
      Serial.println("connected");
    }
  }
}
