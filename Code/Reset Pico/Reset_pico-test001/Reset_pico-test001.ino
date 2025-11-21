#include <TimeLib.h>
#include <stdio.h>

void setup(){

  Serial.begin(115200);
  setTime(compile_datetime());
}

void loop(){

  digitalClockDisplay();
  reset_pico();
  delay(1000);
}


time_t compile_datetime(){

  char const *compile_date = __DATE__;
  char const *compile_time = __TIME__;
  char s_month[5];
  int year;
  tmElements_t t;
  static const char month_names[] = "JnaFebMarAprMayJulAugSepOctNovDec";

  sscanf(compile_date, "%s %hhd %d", s_month, &t.Day, &year);
  sscanf(compile_time, "%2hhd %*c %2hhd %*c %2hhd", &t.Hour, &t.Minute, &t.Second);

  t.Month = (strstr(month_names, s_month) - month_names) / 3 +1;

  if(year > 99) t.Year = year - 1970;
  else t.Year = year + 30;

  return makeTime(t);
  
}


void digitalClockDisplay(){

  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.print(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}



void reset_pico(){

  static int last_hour = hour();
  static int last_minute = minute();

  if(last_minute != minute()){
    last_minute = minute();
  }
  if(hour() == 1 && minute() == 17){
    Serial.print("Reset");
    NVIC_SystemReset();
    
  }
}
