/*************************************************** 
  This is an example for our PCA9685 16-channel PWM & Servo driver
  GPIO test - this will set a pin high/low

  These drivers use I2C to communicate, 2 pins are required to  
  interface.

 ****************************************************/

#include <Wire.h>
#include <pca9685.h>

// called this way, it uses the default address 0x40
PCA9685 pwm = PCA9685();
// you can also call it with a different address you want
//PCA9685 pwm = PCA9685(0x41);
// you can also call it with a different address and I2C interface
//PCA9685 pwm = PCA9685(&Wire, 0x40);

void setup() {
  Serial.begin(9600);
  Serial.println("GPIO test!");

  pwm.begin();
  pwm.setPWMFreq(1000);  // Set to whatever you like, we don't use it in this demo!

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);
}

void loop() {
  // Drive each pin in a 'wave'
  for (uint8_t pin=0; pin<16; pin++) {
    pwm.setPWM(pin, 4096, 0);       // turns pin fully on
    delay(100);
    pwm.setPWM(pin, 0, 4096);       // turns pin fully off
  }
}
