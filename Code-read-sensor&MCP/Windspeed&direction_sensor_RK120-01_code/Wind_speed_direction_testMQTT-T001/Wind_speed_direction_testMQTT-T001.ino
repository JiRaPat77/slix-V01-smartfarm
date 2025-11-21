#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

const char *ssid = "EINSOFNINE_2G";
const char *password = "hunter01";

//const char *ssid = "Capu";
//const char *password = "Boat77777";

#define MQTT_SERVER "thingsboard.weaverbase.com" 
#define MQTT_PORT 1883
#define MQTT_USERNAME "j71ylnr503sgxyhn8dyk"
#define MQTT_PASSWORD "123456"
#define MQTT_NAME "PICO_W"
#define MQTT_TOPIC "v1/devices/me/telemetry"

WiFiClient client;
PubSubClient mqtt(client);






#include <Wire.h>
#include <MCP342x.h>
#include <Adafruit_GFX.h>
int pinInterrupt = 15;
int Count = 0;
float windSpeed;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;    // the debounce time; increase if the output flickers
/* Demonstrate the use of convertAndRead().
*/
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




void onChange()
{
  if ( digitalRead(pinInterrupt) == LOW )
    Count++;
}


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


  //For wind speed
  Serial.begin(115200); //Initialize serial port
  pinMode( pinInterrupt, INPUT_PULLUP);// set the interrupt pin
 
  //Enable
  attachInterrupt( digitalPinToInterrupt(pinInterrupt), onChange, FALLING);
  delay(500);

  //direction
  //pinMode(dir_pin,INPUT);
  //Serial.begin(115200);


  

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




  //Calculator Wind directions and Speed
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    lastDebounceTime = millis();
    windSpeed = (Count * 8.75)/100;
//    Serial.print(" Wind Speed ");
//    Serial.print((Count * 8.75)/100);
    
    
    
//    Serial.println(" m/s ");
    
  }
  //delay(1500);

  //direction
  int sensorValue = value_4_2;
  
//  int wind_directions = map(sensorValue, 0, 32767 , 0, 360);
//  Serial.print("Direction : ");
//  Serial.print(wind_direction);
//  Serial.println(" degreee ");
//  delay(1500);

  int volt_wind_directions = (sensorValue * (5.0/32767.0))*1000;
  int wind_directions = map(volt_wind_directions , 0 , 5000 , 0 , 360 );

  mqtt.loop();
  StaticJsonDocument<200> jsonDocument;
  jsonDocument["Wind speed"] = windSpeed;
  jsonDocument["Wind directions"] = wind_directions;
  char jsonString[200];
  serializeJson(jsonDocument, jsonString);
  mqtt.publish("v1/devices/me/telemetry", jsonString);
  Count = 0;


   
//  mqtt.loop();
//  String dataJS_3 = "{\"Wind speed\":" + String(windSpeed, 3) + ",\"Wind directions\":" + String(wind_directions, 3) + "}";
//  char json_3[100];
//  dataJS_3.toCharArray(json_3,dataJS_3.length()+1);
//  mqtt.publish("v1/devices/me/telemetry" , json_3);
//
//  Count = 0;
  
}
