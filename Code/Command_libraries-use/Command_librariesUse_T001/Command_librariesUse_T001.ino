#include <soil-moisture_RK520-01_libraries_T002.h>
#include <Temp_Humi_AM2305B_T001.h>
#include <Windspeed_direction_RK120-01_libraries_T002.h>
#include <UV_sensor_RK200-04_libraries_T001.h>

void setup() {
  initial_TempHumi();
  initial_SoilMoisture();
  initial_Wind();
  initial_UV();

}

void loop() {
  TempHumi();
  delay(1500);
  SoilMoisture();
  delay(1500);
  Wind();
  delay(1500);
  UV_sensor();
  delay(1500);

}
