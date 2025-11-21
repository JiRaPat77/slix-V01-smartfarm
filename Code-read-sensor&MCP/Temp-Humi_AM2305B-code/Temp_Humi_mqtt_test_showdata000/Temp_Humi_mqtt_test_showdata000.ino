#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define DHTPIN 14
#define DHTTYPE DHT22

DHT_Unified dht(DHTPIN, DHTTYPE);

const char* ssid = "Capu";
const char* password = "Boat77777";
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttUser = "mqtt_username";
const char* mqttPassword = "mqtt_password";
const char* mqttTopic = "smartfarmtest01"; // กำหนด MQTT topic ที่คุณต้องการ

WiFiClient espClient;
PubSubClient client(espClient);

uint32_t delayMS;

void setup() {
  Serial.begin(115200);
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
  setupWiFi();
  client.setServer(mqttServer, mqttPort);
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("RP2040Client", mqttUser, mqttPassword)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  delay(delayMS);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    // ส่งค่าอุณหภูมิไปยัง MQTT topic
    if (client.connected()) {
      //char temperatureStr[6];
      //dtostrf(event.temperature, 4, 2, temperatureStr);
      //client.publish(mqttTopic, temperatureStr);
      char temperatureMessage[50];  // สร้างตัวแปรเพื่อเก็บ payload ของ MQTT message
      sprintf(temperatureMessage, "Temperature: %.2f°C", event.temperature);
      client.publish(mqttTopic, temperatureMessage);
    }
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  } else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    // ส่งค่าความชื้นไปยัง MQTT topic
    if (client.connected()) {
      //char humidityStr[6];
      //dtostrf(event.relative_humidity, 4, 2, humidityStr);
      //client.publish(mqttTopic, humidityStr);
      char humidityMessage[50];  // สร้างตัวแปรเพื่อเก็บ payload ของ MQTT message
      sprintf(humidityMessage, "Humidity: %.2f %", event.relative_humidity);
      client.publish(mqttTopic, humidityMessage);
    }
  }

  if (!client.connected()) {
    reconnect();
  }
}
