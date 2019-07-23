/*
  ===============================================
  Example sketch for BMI160 library for Intel(R) Curie(TM) devices.
  Copyright (c) 2019 CallMeCode Corporation.  All rights reserved.

  Based on I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050
  class by Jeff Rowberg: https://github.com/jrowberg/i2cdevlib

  ===============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/
#include <bluefruit52.h>

const int irq_pin = 30;
const int i2c_addr = 0x69;

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values

const int ledPin = 19;      // activity LED pin
bool blinkState = false; // state of the LED

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

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

  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(BMI160.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(BMI160.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(BMI160.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(BMI160.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(BMI160.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(BMI160.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //BMI160.setAccelerometerOffset(X_AXIS,495.3);
    //BMI160.setAccelerometerOffset(Y_AXIS,-15.6);
    //BMI160.setAccelerometerOffset(Z_AXIS,491.4);
    //BMI160.setGyroOffset(X_AXIS,7.869);
    //BMI160.setGyroOffset(Y_AXIS,-0.061);
    //BMI160.setGyroOffset(Z_AXIS,15.494);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    BMI160.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(BMI160.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(BMI160.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(BMI160.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(BMI160.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(BMI160.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(BMI160.getGyroOffset(Z_AXIS));
  }
  
  // configure Arduino LED for activity indicator
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // read raw accel/gyro measurements from device
  BMI160.readMotionSensor(ax, ay, az, gx, gy, gz);

  // these methods (and a few others) are also available

  //BMI160.readAcceleration(ax, ay, az);
  //BMI160.readRotation(gx, gy, gz);

  //ax = BMI160.readAccelerometer(X_AXIS);
  //ay = BMI160.readAccelerometer(Y_AXIS);
  //az = BMI160.readAccelerometer(Z_AXIS);
  //gx = BMI160.readGyro(X_AXIS);
  //gy = BMI160.readGyro(Y_AXIS);
  //gz = BMI160.readGyro(Z_AXIS);

  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(ledPin, blinkState);
  delay(100);
}
