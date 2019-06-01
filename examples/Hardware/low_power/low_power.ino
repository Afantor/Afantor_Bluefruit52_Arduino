/*
  low power mode, button for wakeup.

  This example code is in the public domain.

  2019/05/27
  by Afantor
*/


#include <Arduino.h>

int interruptPin = 18;

void setup()
{
  Serial.begin(115200);
  Serial.println("low power mode test!");
}

void loop()
{
  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
  systemOff(interruptPin, LOW);
}


