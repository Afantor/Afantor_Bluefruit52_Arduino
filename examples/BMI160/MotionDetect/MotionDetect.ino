/*
   Copyright (c) 2019 CallMeCode Corporation.  All rights reserved.
   See the bottom of this file for license terms.
*/

/*
   This sketch example demonstrates how the BMI160 accelerometer on the
   Intel(R) Curie(TM) module can be used to detect motion events
*/

#include <bluefruit52.h>

const int irq_pin = 30;
const int i2c_addr = 0x69;

bool blinkState = false;          // state of the LED
unsigned long loopTime = 0;          // get the time since program started
unsigned long interruptsTime = 0;    // get the time when motion event is detected

void setup() {
  Serial.begin(115200); // initialize Serial communication
  Wire.begin();
  pinMode(19, OUTPUT);
  // initialize device
  Serial.println("Initializing IMU device...");
  // BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr, irq_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);
  BMI160.attachInterrupt(eventCallback);

  /* Enable Motion Detection */
  BMI160.setDetectionThreshold(CURIE_IMU_MOTION, 20); // 20mg
  BMI160.setDetectionDuration(CURIE_IMU_MOTION, 10);  // trigger times of consecutive slope data points
  BMI160.setIntMotionEnabled(true);

  Serial.println("IMU initialisation complete, waiting for events...");
}

void loop() {
  // if motion is detected in 1000ms, LED will be turned up
  loopTime = millis();
  if(abs(loopTime -interruptsTime) < 1000 )
    blinkState = true;
  else
    blinkState = false;
  digitalWrite(19, blinkState);
}


static void eventCallback(void){
  if (BMI160.getInterruptStatus(CURIE_IMU_MOTION)) {
    if (BMI160.motionDetected(X_AXIS, POSITIVE))
      Serial.println("Negative motion detected on X-axis");
    if (BMI160.motionDetected(X_AXIS, NEGATIVE))
      Serial.println("Positive motion detected on X-axis");
    if (BMI160.motionDetected(Y_AXIS, POSITIVE))
      Serial.println("Negative motion detected on Y-axis");
    if (BMI160.motionDetected(Y_AXIS, NEGATIVE))
      Serial.println("Positive motion detected on Y-axis");
    if (BMI160.motionDetected(Z_AXIS, POSITIVE))
      Serial.println("Negative motion detected on Z-axis");
    if (BMI160.motionDetected(Z_AXIS, NEGATIVE))
      Serial.println("Positive motion detected on Z-axis");
    interruptsTime = millis(); 
  } 
}

/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/
