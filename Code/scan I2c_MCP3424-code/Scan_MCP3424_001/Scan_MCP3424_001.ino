#include <Wire.h>
#include <MCP342x.h>


/* Demonstrate the use of convertAndRead().
*/
long value_1_1,value_2_1,value_3_1,value_4_1;
long value_1_2,value_2_2,value_3_2,value_4_2;
long value_1_3,value_2_3,value_3_3,value_4_3;

// 0x68 is the default address for all MCP342x devices
// uint8_t address_mcp3424_1 = 0x10;
uint8_t address_mcp3424_1 = 0x68;
uint8_t address_mcp3424_2 = 0x6A;
uint8_t address_mcp3424_3 = 0x6B;

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

  delay(1500); 
  
  
}
