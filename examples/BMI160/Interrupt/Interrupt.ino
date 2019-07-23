#include <bluefruit52.h>

const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 30;

void bmi160_intr(void)
{
  Serial.println("BMI160 interrupt: TAP!");
}

void setup() {
  Serial.begin(115200); // initialize Serial communication

  // initialize device
  Serial.println("Initializing IMU device...");
  // BMI160.begin(BMI160GenClass::SPI_MODE, bmi160_select_pin, bmi160_interrupt_pin);
  BMI160.begin(BMI160GenClass::I2C_MODE, bmi160_i2c_addr, bmi160_interrupt_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  BMI160.attachInterrupt(bmi160_intr);
  BMI160.setIntTapEnabled(true);
  Serial.println("Initializing IMU device...done.");
}

void loop() {
}
