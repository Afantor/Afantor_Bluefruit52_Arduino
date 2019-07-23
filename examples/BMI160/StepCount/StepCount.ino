/*
 * Copyright (c) 2019 CallMeCode Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 accelerometer on the
   Intel(R) Curie(TM) module can be used as a Step Counter (pedometer)
*/

#include <bluefruit52.h>

const int irq_pin = 30;
const int i2c_addr = 0x69;

/* To get an interrupt notification for every step detected,
    set stepEventsEnabeled to true. Otherwise, the main loop will
    poll for the current step count.

   By design, the step counter does not immediately update on every step detected.
   Please refer to Section 2.7 of the BMI160 IMU SensorData Sheet
   for more information on this feature.
*/
const int ledPin = 19;

bool stepEventsEnabeled = true;   // whether you're polling or using events
long lastStepCount = 0;              // step count on previous polling check
bool blinkState = false;          // state of the LED

void setup() {
  Serial.begin(115200); // initialize Serial communication
  Wire.begin();
  pinMode(ledPin, OUTPUT);
  // initialize device
  Serial.println("Initializing IMU device...");
  // BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr, irq_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);
  // turn on step detection mode:
  BMI160.setStepDetectionMode(CURIE_IMU_STEP_MODE_NORMAL);
  // enable step counting:
  BMI160.setStepCountEnabled(true);

  if (stepEventsEnabeled) {
    // attach the eventCallback function as the
    // step event handler:
    BMI160.attachInterrupt(eventCallback);
    // BMI160.interruptsEnabled(CURIE_IMU_STEP);  // turn on step detection
    BMI160.setIntStepEnabled(true);
    Serial.println("IMU initialisation complete, waiting for events...");
  }
}

void loop() {
  /* Instead of using step detection event notifications,
     we can check the step count periodically */
  if (!stepEventsEnabeled) {
    updateStepCount();
  }
  digitalWrite(ledPin, blinkState);
  blinkState = !blinkState;
  delay(1000);
}

static void updateStepCount() {
  // get the step count:
  int stepCount = BMI160.getStepCount();

  // if the step count has changed, print it:
  if (stepCount != lastStepCount) {
    Serial.print("Step count: ");
    Serial.println(stepCount);
    // save the current count for comparison next check:
    lastStepCount = stepCount;
  }
}

static void eventCallback(void) {
  if (BMI160.stepsDetected())
    updateStepCount();
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
