/*
  ADC get battery value
  get adc value from pin and print value.

  This example code is in the public domain.

  2019/01/04
  by Afantor
*/

/*
 * This sketch demonstrate how to pass ISR_DEFFERED as additional parameter
 * to defer callback from ISR context with attachInterrupt
 */
#include <Arduino.h>

int interruptPin = A0;

void setup()
{
  Serial.begin(115200);

  pinMode(interruptPin, INPUT_PULLUP);

  // ISR_DEFERRED flag cause the callback to be deferred from ISR context
  // and invoked within a callback thread.
  // It is required to use ISR_DEFERRED if callback function take long time 
  // to run e.g Serial.print() or using any of Bluefruit API() which will
  // potentially call rtos API
  attachInterrupt(interruptPin, digital_callback, ISR_DEFERRED | CHANGE);
}

void loop()
{
  // nothing to do
}

void digital_callback(void)
{
  Serial.print("Pin value: ");
  Serial.println(digitalRead(interruptPin));
}
