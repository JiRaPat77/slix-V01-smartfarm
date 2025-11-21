#include <WiFi.h>
#include <PubSubClient.h>
#include <RP2040.h>  // ไลบรารีสำหรับการรีเซ็ตบอร์ด
#include <ArduinoJson.h>

// ข้อมูลการเชื่อมต่อ Wi-Fi
const char* ssid = "Weaverbase";
const char* password = "1212312121";

// ข้อมูลการเชื่อมต่อ ThingsBoard
const char* mqtt_server = "thingsboard.weaverbase.com";
const char* access_token = "oa1vhNejZyH43SOZ9Y76";

WiFiClient espClient;
PubSubClient client(espClient);

// ฟังก์ชันการเชื่อมต่อ Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// ฟังก์ชันสำหรับการจัดการข้อความที่ได้รับจาก MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  // แปลง payload เป็น String
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // พิมพ์ข้อความที่ได้รับ
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);

  // สร้างออบเจกต์ JSON document ขนาด 200 ไบต์
  StaticJsonDocument<200> doc;
  
  // แปลงข้อความ JSON เป็นออบเจกต์ JSON
  DeserializationError error = deserializeJson(doc, message);
  
  // ตรวจสอบว่าแปลงสำเร็จหรือไม่
  if (!error) {
    // อ่านค่าของ "method" และ "params.params"
    const char* method = doc["method"];
    const char* params = doc["params"]["params"];

    // ตรวจสอบว่าค่าที่ได้รับคือ "reset" หรือไม่
    if (method && params && strcmp(method, "ResetPico") == 0 && strcmp(params, "reset") == 0) {
      Serial.println("Resetting device...");
      NVIC_SystemReset();
    }
  } else {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
  }
}

// ฟังก์ชันการเชื่อมต่อ MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("RaspberryPiPicoClient", access_token, "")) {
      Serial.println("connected");
      client.subscribe("v1/devices/me/rpc/request/+");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
