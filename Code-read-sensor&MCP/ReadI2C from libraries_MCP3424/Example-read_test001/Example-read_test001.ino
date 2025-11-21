#include <scan-MCP3424_P_gatekong_libraries_T002.h>

MyLibrary myLibrary;  // ชื่อ class ชื่อ ตั้งขึ้นมาเพื่อใช้ในโปรแกรมนี้ 

void setup() {
  Serial.begin(115200);
  myLibrary.initialize();  // เรียกเมทอด initialize จากไลบรารี
}

void loop() {
  myLibrary.updateValues();  // เรียกเมทอด updateValues จากไลบรารี
  long v2_2 = myLibrary.getValue_2_2();  // เรียกเมทอด getValue_2_2 และนำค่ามาใช้
  
  Serial.print("Value_2_2: ");
  Serial.println(v2_2);
  
  delay(1000);
}
