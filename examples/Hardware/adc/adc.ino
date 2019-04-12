/*
  ADC get input
  get adc value from pin and print value.

  This example code is in the public domain.

  2019/01/02
  by Afantor
*/
int adc_pin    = A5;
int adc_value = 0;
float mv_per_lsb = 3600.0F/1024.0F; // 10-bit ADC with 3.6V input range

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Get a fresh ADC value
  adc_value = analogRead(adc_pin);

  // Print the results
  Serial.print(adc_value);
  Serial.print(" [");
  Serial.print((float)adc_value * mv_per_lsb);
  Serial.println(" mV]");

  delay(100);
}
