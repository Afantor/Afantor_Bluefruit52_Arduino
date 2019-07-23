/*
 * Copyright (c) 2019 CallMeCode Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read accelerometer data
   and translate it to an orientation
*/

#include <bluefruit52.h>

const int irq_pin = 30;
const int i2c_addr = 0x69;

int lastOrientation = - 1; // previous orientation (for comparison)

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

  // Set the accelerometer range to 2G
  BMI160.setAccelerometerRange(2);
  Serial.println("Initializing IMU device...done.");
}

void loop() {
  int orientation = - 1;   // the board's orientation
  String orientationString; // string for printing description of orientation
  /*
    The orientations of the board:
    0: flat, processor facing up
    1: flat, processor facing down
    2: landscape, analog pins down
    3: landscape, analog pins up
    4: portrait, USB connector up
    5: portrait, USB connector down
  */
  // read accelerometer:
  int x = BMI160.readAccelerometer(X_AXIS);
  int y = BMI160.readAccelerometer(Y_AXIS);
  int z = BMI160.readAccelerometer(Z_AXIS);

  // calculate the absolute values, to determine the largest
  int absX = abs(x);
  int absY = abs(y);
  int absZ = abs(z);

  if ( (absZ > absX) && (absZ > absY)) {
    // base orientation on Z
    if (z > 0) {
      orientationString = "up";
      orientation = 0;  
    } else {
      orientationString = "down";
      orientation = 1;
    }
  } else if ( (absY > absX) && (absY > absZ)) {
    // base orientation on Y
    if (y > 0) {
      orientationString = "digital pins up";
      orientation = 2;
    } else {
      orientationString = "analog pins up";
      orientation = 3;
    }
  } else {
    // base orientation on X
    if (x < 0) {
      orientationString = "connector up";
      orientation = 4;
    } else {
      orientationString = "connector down";
      orientation = 5;
    }
  }

  // if the orientation has changed, print out a description:
  if (orientation != lastOrientation) {
    Serial.println(orientationString);
    lastOrientation = orientation;
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
