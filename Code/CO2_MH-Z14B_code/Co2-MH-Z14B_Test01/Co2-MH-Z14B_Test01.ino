//#include <SoftwareSerial.h>

#define MH_Tx 12
#define MH_Rx 13
//#define PIN_10 10

//SoftwareSerial MH_Z14a(MH_Rx, MH_Tx);

void setup() {
  Serial.begin(12,13);
  //MH_Z14a.begin(115200);
  pinMode(MH_Rx, INPUT);
  pinMode(MH_Tx, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2 , HIGH);
  digitalWrite(3 , HIGH);
//  digitalWrite(10 , HIGH);
 
  
  byte cmd[9] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88, 0x8F};   //Detection range 5000pmm
  //byte cmd[9] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F}; //Detection range 2000pmm
  //byte cmd[9] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x27, 0x10, 0x2F}; //Detection range 10,000pmm
  //MH_Z14a.write(cmd, 9);
}

void loop() {

  
  int CO2 = MH_Rx;
  Serial.print("CO2 : ");
  Serial.print(CO2);
  Serial.println("ppm");

  
  delay(3000);// heat time 3minut
}

//int mh_Z14A_function(){
//  byte Z14a[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; //Read CO2
//  MH_Z14a.write(Z14a, 9);
//  
//  byte data_byte[9];
//
//  if(MH_Z14a.available()>0)
//  {
//    MH_Z14a.readBytes(data_byte, 9);
//  }
//
//  if(data_byte[1] == 1){
//    return 0;
//  }
//  else{
//    int CO2_ppm = data_byte[2]*256+data_byte[3];
//    return CO2_ppm;
//  }
//}
