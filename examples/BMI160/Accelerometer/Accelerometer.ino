/*
 * Copyright (c) 2019 CallMeCode Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read accelerometer data
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

  // Set the accelerometer range to 2G
  BMI160.setAccelerometerRange(2);
  Serial.println("Initializing IMU device...done.");
}

void loop() {
  int axRaw, ayRaw, azRaw;         // raw accel values
  float ax, ay, az;   //scaled accelerometer values

  // read accelerometer measurements from device, scaled to the configured range
  // read raw accel measurements from device
  BMI160.readAccelerometer(axRaw, ayRaw, azRaw);
  // convert the raw accel data to g
  ax = convertRawAccel(axRaw);
  ay = convertRawAccel(ayRaw);
  az = convertRawAccel(azRaw);
  // display tab-separated accelerometer x/y/z values
  Serial.print("Accel:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.println();
  delay(100);
}

float convertRawAccel(int aRaw) {
  // since we are using 2000 degrees/seconds range
  // -2g to a raw value of -32768
  // +2g to a raw value of 32767

  float a = ((aRaw * 2.0) / 32768.0) * 9.8;

  return a;
}


