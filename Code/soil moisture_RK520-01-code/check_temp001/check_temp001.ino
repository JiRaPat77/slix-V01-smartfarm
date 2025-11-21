int decimalPrecision = 2;               // decimal places for all values shown in LED Display & Serial Monitor
float voltageDividerR1 = 10000;         // Resistor value in R1 for voltage devider method 
float BValue = 32767;                    // The B Value of the thermistor for the temperature measuring range
float R1 = 1000;                        // Thermistor resistor rating at based temperature (25 degree celcius)
float T1 = 298.15;                      /* Base temperature T1 in Kelvin (default should be at 25 degree)*/
float R2 ;                              /* Resistance of Thermistor (in ohm) at Measuring Temperature*/
float T2 ;                              /* Measurement temperature T2 in Kelvin */
        
float a ;                               /* Use for calculation in Temperature*/
float b ;                               /* Use for calculation in Temperature*/
float c ;                               /* Use for calculation in Temperature*/
float d ;                               /* Use for calculation in Temperature*/
float e = 2.718281828 ;                 /* the value of e use for calculation in Temperature*/
        
float tempSampleRead  = 0;               /* to read the value of a sample including currentOffset1 value*/
float tempLastSample  = 0;               /* to count time for each sample. Technically 1 milli second 1 sample is taken */
float tempSampleSum   = 0;               /* accumulation of sample readings */
float tempSampleCount = 1000;               /* to count number of sample. */
float tempMean ;                         /* to calculate the average value from all samples, in analog values*/ 
void setup() {
  Serial.begin(115200);

}

void loop() {
  int valueTemp_1 = value_1_1;
  if(millis() >= tempLastSample + 1){
    tempSampleSum = tempSampleSum + valueTemp_1;
    tempSampleCount = tempSampleCount + 1;
    tempLastSample = millis();
    
  }

  tempMean = tempSampleSum / tempSampleCount;
  R2 = ((voltageDividerR1 * tempMean)/(1023 - tempMean));

  a = 1/T1;
  b = log10(R1/R2);
  c = b / log10(e);
  d = c / BValue;
  T2 = 1 / (a-d);
  Serial.print(T2 - 273.15,decimalPrecision);
  Serial.println(" Â°C");

  tempSampleSum = 0;
  tempSampleCount = 0;

}
