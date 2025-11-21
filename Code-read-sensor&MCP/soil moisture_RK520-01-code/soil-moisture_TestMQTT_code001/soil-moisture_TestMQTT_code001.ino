
//Test function and result

#include <Wire.h>
#include <MCP342x.h>

#include <PubSubClient.h>
#include <WiFi.h>

// const char *ssid = "EINSOFNINE_2G";
// const char *password = "hunter01";

// #define MQTT_SERVER "thingsboard.weaverbase.com" 
// #define MQTT_PORT 1883
// #define MQTT_USERNAME "7p2oxwi1ssz0zyybgo7o"
// #define MQTT_PASSWORD "123456"
// #define MQTT_NAME "PICO_W"
// #define MQTT_TOPIC "v1/devices/me/telemetry"

// WiFiClient client;
// PubSubClient mqtt(client);


/////////Variable of Temp//////////////
int decimalPrecision = 2;               // decimal places for all values shown in LED Display & Serial Monitor
float voltageDividerR1 = 10000;         // Resistor value in R1 for voltage devider method 
float BValue = 3950;                    // The B Value of the thermistor for the temperature measuring range
float R1 = 6500;                        // Thermistor resistor rating at based temperature (25 degree celcius)
float T1 = 298.15;                      /* Base temperature T1 in Kelvin (default should be at 25 degree)*/
float R2_0 , R2_1 , R2_2 , R2_3 ;       /* Resistance of Thermistor (in ohm) at Measuring Temperature*/
float T2_0 , T2_1 , T2_2 , T2_3 ;       /* Measurement temperature T2 in Kelvin */

float a0,b0,c0,d0,a1,b1,c1,d1,a2,b2,c2,d2,a3,b3,c3,d3;
float e = 2.718281828;

float tempLastSample_0 = 0, tempSampleSum_0 = 0, tempMean_0 = 0 , tempSampleCount_0 = 1000 ;
float tempLastSample_1 = 0, tempSampleSum_1 = 0, tempMean_1 = 0 , tempSampleCount_1 = 1000 ;
float tempLastSample_2 = 0, tempSampleSum_2 = 0, tempMean_2 = 0 , tempSampleCount_2 = 1000 ;
float tempLastSample_3 = 0, tempSampleSum_3 = 0, tempMean_3 = 0 , tempSampleCount_3 = 1000 ;




/////////Variable of Soil moisture//////////////
const float Vmin = 0.0;   
const float Vmax = 5.0;   
const float soilMin = 0.0;  
const float soilMax = 100.0;  
 

/* Demonstrate the use of convertAndRead().*/
long value_1_1,value_2_1,value_3_1,value_4_1;
long value_1_2,value_2_2,value_3_2,value_4_2;
long value_1_3,value_2_3,value_3_3,value_4_3;

// 0x68 is the default address for all MCP342x devices
uint8_t address_mcp3424_1 = 0x68;
uint8_t address_mcp3424_2 = 0x6A;
uint8_t address_mcp3424_3 = 0x6C;

MCP342x adc1 = MCP342x(address_mcp3424_1);
MCP342x adc2 = MCP342x(address_mcp3424_2);
MCP342x adc3 = MCP342x(address_mcp3424_3);


void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  // Enable power for MCP342x (needed for FL100 shield only)
  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms

  // Check device present
  Wire.requestFrom(address_mcp3424_1, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address_mcp3424_1, HEX);
    while (1);
  }
  Wire.requestFrom(address_mcp3424_2, (uint8_t)2);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address_mcp3424_2, HEX);
    while (1);
  }
  Wire.requestFrom(address_mcp3424_3, (uint8_t)3);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address_mcp3424_3, HEX);
    while (1);
  }
  
  
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }
  // Serial.println("Connected to WiFi");

  // // Connect to MQTT broker with username and password
  // mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  // if (mqtt.connected() == false){
  //   Serial.print("MQTT connection...");
  //   if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)){
  //     Serial.print("connected");
  //   }else{
  //     Serial.println("connected");
  //     delay(5000);
  //   }
  // }

}

void loop(void)
{
 // long value = 0;
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  uint8_t err1_1 = adc1.convertAndRead(MCP342x::channel1, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_1_1, status);
  uint8_t err2_1 = adc1.convertAndRead(MCP342x::channel2, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_2_1, status);
  uint8_t err3_1 = adc1.convertAndRead(MCP342x::channel3, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_3_1, status);
  uint8_t err4_1 = adc1.convertAndRead(MCP342x::channel4, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_4_1, status);

  uint8_t err1_2 = adc2.convertAndRead(MCP342x::channel1, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_1_2, status);
  uint8_t err2_2 = adc2.convertAndRead(MCP342x::channel2, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_2_2, status);
  uint8_t err3_2 = adc2.convertAndRead(MCP342x::channel3, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_3_2, status);
  uint8_t err4_2 = adc2.convertAndRead(MCP342x::channel4, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_4_2, status);

  uint8_t err1_3 = adc3.convertAndRead(MCP342x::channel1, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_1_3, status);
  uint8_t err2_3 = adc3.convertAndRead(MCP342x::channel2, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_2_3, status);
  uint8_t err3_3 = adc3.convertAndRead(MCP342x::channel3, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_3_3, status);
  uint8_t err4_3 = adc3.convertAndRead(MCP342x::channel4, MCP342x::oneShot,MCP342x::resolution16, MCP342x::gain1,1000000, value_4_3, status);

  
  if (err1_1) {
    Serial.print("Convert error: ");
    Serial.println(err1_1);
  }
  else {
    Serial.print("  Value1_1: ");
    Serial.print(value_1_1); 
    Serial.print("  Value2_1: ");
    Serial.print(value_2_1);
    Serial.print("  Value3_1: ");
    Serial.print(value_3_1);
    Serial.print("  Value4_1: ");
    Serial.print(value_4_1);
    Serial.print("  Value1_2: ");
  }

  if(err1_2){
    Serial.print("Convert error: ");
    Serial.println(err1_2);
  }
  else{
    Serial.print(value_1_2); 
    Serial.print("  Value2_2: ");
    Serial.print(value_2_2);
    Serial.print("  Value3_2: ");
    Serial.print(value_3_2);
    Serial.print("  Value4_2: ");
    Serial.print(value_4_2);
  }

  if(err1_3){
    Serial.print("Convert error: ");
    Serial.println(err1_3);
  }
  else{
    Serial.print("  Value1_3: ");
    Serial.print(value_1_3); 
    Serial.print("  Value2_3: ");
    Serial.print(value_2_3);
    Serial.print("  Value3_3: ");
    Serial.print(value_3_3);
    Serial.print("  Value4_3: ");
    Serial.println(value_4_3);
  
  }

//  if (mqtt.connected() == false){
//    reconnectMqtt();
//  }

  //////// Temperature calc_1 ////////
  // int value_soucre_T0 = value_1_3;
  // int value_soucreMAP_T0 = map(value_soucre_T0 , 0 , 32767 , 32767 , 0);
  // int valueTemp_0 = value_soucreMAP_T0;
  // if(millis() >= tempLastSample_0 + 1){
  //   tempSampleSum_0 = tempSampleSum_0 + valueTemp_0;
  //   tempSampleCount_0 = tempSampleCount_0 + 1;
  //   tempLastSample_0 = millis();
  // }
  
  // tempMean_0 = tempSampleSum_0 / tempSampleCount_0;
  // R2_0 = ((voltageDividerR1 * tempMean_0)/(32767 - tempMean_0));
  // a0 = 1/T1;
  // b0 = log10(R1/R2_0);
  // c0 = b0 / log10(e);
  // d0 = c0 / BValue;
  // T2_0 = 1 / (a0-d0);
  // T2_0 = T2_0 - 273.15,decimalPrecision;

  int value_soucre_T0 = value_1_3;
  Serial.println(value_soucre_T0);







  //////// Temperature calc_2 ////////
  int value_soucre_T1 = value_3_3;
  int value_soucreMAP_T1 = map(value_soucre_T1 , 0 , 32767 , 32767 , 0);
  int valueTemp_1 = value_soucreMAP_T1;
  if(millis() >= tempLastSample_1 + 1){
    tempSampleSum_1 = tempSampleSum_1 + valueTemp_1;
    tempSampleCount_1 = tempSampleCount_1 + 1;
    tempLastSample_1 = millis();
  }
  
  tempMean_1 = tempSampleSum_1 / tempSampleCount_1;
  R2_1 = ((voltageDividerR1 * tempMean_1)/(32767 - tempMean_1));
  a1 = 1/T1;
  b1 = log10(R1/R2_1);
  c1 = b1 / log10(e);
  d1 = c1 / BValue;
  T2_1 = 1 / (a1-d1);
  T2_1 = T2_1 - 273.15,decimalPrecision;






  //////// Temperature calc_3 ////////
  int value_soucre_T2 = value_1_1;
  int value_soucreMAP_T2 = map(value_soucre_T2 , 0 , 32767 , 32767 , 0);
  int valueTemp_2 = value_soucreMAP_T2;
  if(millis() >= tempLastSample_2 + 1){
    tempSampleSum_2 = tempSampleSum_2 + valueTemp_2;
    tempSampleCount_2 = tempSampleCount_2 + 1;
    tempLastSample_2 = millis();
  }
  
  tempMean_2 = tempSampleSum_2 / tempSampleCount_2;
  R2_2 = ((voltageDividerR1 * tempMean_2)/(32767 - tempMean_2));
  a2 = 1/T1;
  b2 = log10(R1/R2_2);
  c2 = b2 / log10(e);
  d2 = c2 / BValue;
  T2_2 = 1 / (a2-d2);
  T2_2 = T2_2 - 273.15,decimalPrecision;

  



  //////// Temperature calc_4 ////////
  int value_soucre_T3 = value_3_1;
  int value_soucreMAP_T3 = map(value_soucre_T3 , 0 , 32767 , 32767 , 0);
  int valueTemp_3 = value_soucreMAP_T3;
  if(millis() >= tempLastSample_3 + 1){
    tempSampleSum_3 = tempSampleSum_3 + valueTemp_3;
    tempSampleCount_3 = tempSampleCount_3 + 1;
    tempLastSample_3= millis();
  }
  
  tempMean_3 = tempSampleSum_3 / tempSampleCount_3;
  R2_3 = ((voltageDividerR1 * tempMean_3)/(32767 - tempMean_3));
  a3 = 1/T1;
  b3 = log10(R1/R2_3);
  c3 = b3 / log10(e);
  d3 = c3 / BValue;
  T2_3 = 1 / (a3-d3);
  T2_3 = T2_3 - 273.15,decimalPrecision;





//////// soil moisture calc_1 ////////
  int sensorValuesoil_0 = value_2_3;
  float voltageSoil_0 = sensorValuesoil_0 * (Vmax - Vmin) / 32767.0 + Vmin;
  
  float SoilMoisture_0 = (voltageSoil_0 - Vmin) / (Vmax - Vmin) * (soilMax - soilMin) + soilMin;





//  //////// soil moisture calc_2 ////////
  int sensorValuesoil_1 = value_4_3;
  float voltageSoil_1 = sensorValuesoil_1 * (Vmax - Vmin) / 32767.0 + Vmin;
  
  float SoilMoisture_1 = (voltageSoil_1 - Vmin) / (Vmax - Vmin) * (soilMax - soilMin) + soilMin;






//  //////// soil moisture calc_3 ////////
  int sensorValuesoil_2 = value_2_1;
  float voltageSoil_2 = sensorValuesoil_2 * (Vmax - Vmin) / 32767.0 + Vmin;
  
  float SoilMoisture_2 = (voltageSoil_2 - Vmin) / (Vmax - Vmin) * (soilMax - soilMin) + soilMin;





//  //////// soil moisture calc_4 ////////
  int sensorValuesoil_3 = value_4_1;
  float voltageSoil_3 = sensorValuesoil_3 * (Vmax - Vmin) / 32767.0 + Vmin;
  
  float SoilMoisture_3 = (voltageSoil_3 - Vmin) / (Vmax - Vmin) * (soilMax - soilMin) + soilMin;




  // mqtt.loop();
  // String dataJS = "{\"Soil Teamperature_1\":" + String(T2_0, 3) + ",\"Soil Teamperature_2\":" + String(T2_1, 3) + ",\"Soil Teamperature_3\":" + String(T2_2, 3) + ",\"Soil Teamperature_4\":" + String(T2_3, 3) + ",\"Soil Moisture_1\":" + String(SoilMoisture_0, 3) + ",\"Soil Moisture_2\":" + String(SoilMoisture_1, 3) + ",\"Soil Moisture_3\":" + String(SoilMoisture_2, 3) + ",\"Soil Moisture_4\":" + String(SoilMoisture_3, 3) + "}";
  // char json[300];
  // dataJS.toCharArray(json,dataJS.length()+1);
  // mqtt.publish("v1/devices/me/telemetry" , json);


  tempSampleSum_0 = 0;
  tempSampleCount_0  = 0;
  tempSampleSum_1 = 0;
  tempSampleCount_1  = 0;
  tempSampleSum_2 = 0;
  tempSampleCount_2  = 0;
  tempSampleSum_3 = 0;
  tempSampleCount_3  = 0;
  



  delay(1000);
  
}
