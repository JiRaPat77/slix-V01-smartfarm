#include <SoftwareSerial.h>

SoftwareSerial co2Serial(13, 12); // RX, TX

byte cmd_get_co2[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
unsigned char response[9];

void setup() {
  Serial.begin(9600);
  co2Serial.begin(9600);
}

void loop() {
  co2Serial.write(cmd_get_co2, 9);
  delay(100); // รอการตอบกลับจากเซ็นเซอร์

  if (co2Serial.available()) {
    for (int i = 0; i < 9; i++) {
      response[i] = co2Serial.read();
    }

    if (response[0] == 0xFF && response[1] == 0x86) {
      int co2ppm = response[2] * 256 + response[3]; // คำนวณค่าความเข้ม CO2
      Serial.print("CO2 Concentration: ");
      Serial.print(co2ppm);
      Serial.println(" ppm");
    }
  }

  delay(2000); // รอ 2 วินาทีก่อนอ่านใหม่
}
