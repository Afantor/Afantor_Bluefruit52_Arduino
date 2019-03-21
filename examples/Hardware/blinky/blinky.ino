/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  This example code is in the public domain.

  2019/01/01
  by Afantor
*/
#include <bluefruit52.h>
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_2, OUTPUT);
}
     
// the loop function runs over and over again forever
void loop() {
  digitalToggle(LED_2);   // turn the LED on (HIGH is the voltage level)
  delay(500);              // wait for a second
}
