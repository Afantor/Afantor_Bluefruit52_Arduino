/*
  button scan example.

  This example code is in the public domain.

  2019/01/05
  by Afantor
*/
#include <bluefruit52.h>
// The setup() function runs once each time the micro-controller starts
void setup() {
  // init lcd, serial, IMU
  Bluefruit52.begin(true, true, false);

}

// Add the main program code into the continuous loop() function
void loop() {
  Bluefruit52.update();
 
  // if want use Releasefor; suggest use Release in press event
   if (Bluefruit52.BtnA.wasReleased()) {
    Serial.println('A');
  } else if (Bluefruit52.BtnB.wasReleased()) {
    Serial.println('B');
  } else if (Bluefruit52.BtnB.wasReleasefor(700)) {
  	//Serial.println('AB');
  }
}