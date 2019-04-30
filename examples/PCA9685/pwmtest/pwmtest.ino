/*************************************************** 
  This is an example for our PCA9685 16-channel PWM & Servo driver
  PWM test - this will drive 16 PWMs in a 'wave'

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
  Serial.begin(115200);
  Serial.println("16 channel PWM test!");

  pwm.begin();
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);
}

void loop() {
  // Drive each PWM in a 'wave'
  for (uint16_t i=0; i<4096; i += 8) {
    for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) {
      pwm.setPWM(pwmnum, 0, (i + (4096/16)*pwmnum) % 4096 );
    }
  }
}
