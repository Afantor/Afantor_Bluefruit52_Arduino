/*
   Copyright (c) 2019 CallMeCode Corporation.  All rights reserved.
   See the bottom of this file for license terms.
*/

/*
   This sketch example demonstrates how the BMI160 accelerometer on the
   Intel(R) Curie(TM) module can be used to detect tap events
*/

#include <bluefruit52.h>

const int irq_pin = 30;
const int i2c_addr = 0x69;

void setup() {
  Serial.begin(115200); // initialize Serial communication
  Wire.begin();
  // initialize device
  Serial.println("Initializing IMU device...");
  // BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr, irq_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);
  BMI160.attachInterrupt(eventCallback);

  // Increase Accelerometer range to allow detection of stronger taps (< 4g)
  BMI160.setAccelerometerRange(4);

  // Reduce threshold to allow detection of weaker taps (>= 750mg)
  BMI160.setDetectionThreshold(CURIE_IMU_DOUBLE_TAP, 750); // (750mg)

  // Set the quite time window for 2 taps to be registered as a double-tap (Gap time between taps <= 1000 milliseconds)
  BMI160.setDetectionDuration(CURIE_IMU_DOUBLE_TAP, 1000);

  // Enable Double-Tap detection
  BMI160.setIntDoubleTapEnabled(true);

  Serial.println("IMU initialization complete, waiting for events...");
}

void loop() {
  // nothing happens in the loop because all the action happens
  // in the callback function.
}

static void eventCallback()
{
  if (BMI160.getInterruptStatus(CURIE_IMU_DOUBLE_TAP)) {
    if (BMI160.tapDetected(X_AXIS, NEGATIVE))
      Serial.println("Double Tap detected on negative X-axis");
    if (BMI160.tapDetected(X_AXIS, POSITIVE))
      Serial.println("Double Tap detected on positive X-axis");
    if (BMI160.tapDetected(Y_AXIS, NEGATIVE))
      Serial.println("Double Tap detected on negative Y-axis");
    if (BMI160.tapDetected(Y_AXIS, POSITIVE))
      Serial.println("Double Tap detected on positive Y-axis");
    if (BMI160.tapDetected(Z_AXIS, NEGATIVE))
      Serial.println("Double Tap detected on negative Z-axis");
    if (BMI160.tapDetected(Z_AXIS, POSITIVE))
      Serial.println("Double Tap detected on positive Z-axis");
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
