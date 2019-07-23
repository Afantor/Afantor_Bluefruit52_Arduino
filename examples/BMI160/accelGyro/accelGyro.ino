 /*
  * accelGyro.ino
  *
  * I2C addr:
  *   0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
  *   0x69: set I2C address by parameter
  *
  * Through the example, you can get the sensor data by using getSensorData:
  * get acell by paremeter onlyAccel;
  * get gyro by paremeter onlyGyro;
  * get both acell and gyro by paremeter bothAccelGyro.
  * 
  * With the rotation of the sensor, data changes are visible.
  *
  * Copyright   [Afantor](http://www.afantor.cc), 2019
  * Copyright   MIT License
  *
  * version  V1.0
  * date  2019-05-27
  */

#include <bluefruit52.h>

const int irq_pin = 30;
const int i2c_addr = 0x69;


void setup() {
  Serial.begin(115200); // initialize Serial communication
  Wire.begin();

  // initialize device
  Serial.println("Initializing IMU device...");

  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr, irq_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

   // Set the gyro range to 2000 degrees/second
  BMI160.setGyroRate(3200);
  BMI160.setGyroRange(2000);
   // Set the accelerometer range to 2g
  BMI160.setAccelerometerRate(1600);
  BMI160.setAccelerometerRange(2);
  Serial.println("Initializing BMI160 device...done.");
}

void loop() {
  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  float gx, gy, gz;

  int axRaw, ayRaw, azRaw;         // raw accel values
  float ax, ay, az;

  // read raw gyro measurements from device
  BMI160.readGyro(gxRaw, gyRaw, gzRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);

  // display tab-separated gyro x/y/z values
  Serial.print("Gyro:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();

  // read raw accel measurements from device
  BMI160.readAccelerometer(axRaw, ayRaw, azRaw);

  // convert the raw accel data to g
  ax = convertRawAccel(axRaw);
  ay = convertRawAccel(ayRaw);
  az = convertRawAccel(azRaw);

  // display tab-separated accel x/y/z values
  Serial.print("Accel:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.println();

  delay(100);
}

float convertRawGyro(int gRaw) {
  // since we are using 2000 degrees/seconds range
  // -2000 maps to a raw value of -32768
  // +2000 maps to a raw value of 32767

  float gyro = (gRaw * 2000.0) / (32767.0 * 57.29577);

  return gyro;
}

float convertRawAccel(int aRaw) {
  // since we are using 2g range
  // -2g to a raw value of -32768
  // +2g to a raw value of 32767

  float accel = ((aRaw * 2.0) / 32768.0) * 9.8;

  return accel;
}









