/*
 * Copyright (c) 2019 CallMeCode Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 accelerometer on the
   Intel(R) Curie(TM) module can be used to detect shocks or sudden movements
*/

#include <bluefruit52.h>

const int irq_pin = 30;
const int i2c_addr = 0x69;

bool blinkState = false;          // state of the LED

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

  /* Enable Shock Detection */
  BMI160.setDetectionThreshold(CURIE_IMU_SHOCK, 1500); // 1.5g = 1500 mg
  BMI160.setDetectionDuration(CURIE_IMU_SHOCK, 50);   // 50ms
  BMI160.setIntShockEnabled(true);

  Serial.println("IMU initialisation complete, waiting for events...");
}

void loop() {
  // blink the LED in the main loop:
  digitalWrite(19, blinkState);
  blinkState = !blinkState;
  delay(1000);
}


static void eventCallback(void)
{
  if (BMI160.getInterruptStatus(CURIE_IMU_SHOCK)) {
    if (BMI160.shockDetected(X_AXIS, POSITIVE))
      Serial.println("Negative shock detected on X-axis");
    if (BMI160.shockDetected(X_AXIS, NEGATIVE))
      Serial.println("Positive shock detected on X-axis");
    if (BMI160.shockDetected(Y_AXIS, POSITIVE))
      Serial.println("Negative shock detected on Y-axis");
    if (BMI160.shockDetected(Y_AXIS, NEGATIVE))
      Serial.println("Positive shock detected on Y-axis");
    if (BMI160.shockDetected(Z_AXIS, POSITIVE))
      Serial.println("Negative shock detected on Z-axis");
    if (BMI160.shockDetected(Z_AXIS, NEGATIVE))
      Serial.println("Positive shock detected on Z-axis");
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
