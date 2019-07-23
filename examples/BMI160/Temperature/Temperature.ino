 /*
  * Temperature.ino
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
  int TempRaw;         // raw temperature values
  float temp;


  // read raw temperature from device
  TempRaw = BMI160.getTemperature();

  // convert the raw temp data to °C.
  temp = ((float) TempRaw) / 512.0 + 23.0;
  Serial.print("Temp:\t");
  Serial.print(temp,1);
  Serial.println("°C");
  delay(1000);
}




