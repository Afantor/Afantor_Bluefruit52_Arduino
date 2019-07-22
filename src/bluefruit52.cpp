/**************************************************************************/
/*!
    @file     bluefruit52.cpp
    @author   Afantor

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2019, Afantor Industries.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include "bluefruit52.h"

AfantorBluefruit52 Bluefruit52;

/**
 * Constructor
 */
AfantorBluefruit52::AfantorBluefruit52(void):isInited(0){

}

void AfantorBluefruit52::begin(bool SerialEnable, bool LCDEnable, bool IMUEnable) {

  // Correct init once
  if (isInited) return;
  else isInited = true;

  // UART
  if (SerialEnable) {
    Serial.begin(115200);
    Serial.flush();
    delay(50);
    Serial.println("Bluefruit52 initializing...");
  }

  // LCD INIT
  if (LCDEnable) {
    pinMode(LCD_CS_PIN, OUTPUT);
    Lcd.showEnable();
    Lcd.init(240, 135);   // initialize a ST7789 chip, 135x240 pixels
    Lcd.setRotation(1);
    Lcd.fillScreen(BLACK);
    delay(50);
    Lcd.setTextWrap(false);
    Lcd.fillScreen(BLACK);
    Lcd.setCursor(0, 0);
    Lcd.setTextColor(RED);
    Lcd.setTextSize(2);
    Lcd.println(" Hello Master!");
    Lcd.setTextColor(WHITE);
    Lcd.setTextSize(2);
    Lcd.println("Wellcome to BLE.");    
    Lcd.setTextColor(YELLOW);
    Lcd.setTextSize(1);
    Lcd.println("Hello Master, Welcome to the Bluetooth");
    Lcd.setTextColor(GREEN);
    Lcd.setTextSize(1);
    Lcd.println("5.0 board, you can connect ");
    Lcd.setTextColor(BLUE);
    Lcd.setTextSize(1);
    Lcd.println("everything with Bluetooth.");
    Lcd.setTextColor(RED);
    Lcd.setTextSize(2);
    Lcd.println(" initializing......");
    delay(500);
  }

  // I2C init
  Wire.begin();
  delay(10);

  //BMI160 init
  if (IMUEnable)
  {
    Serial.println("BMI160 6-DOF 16-bit motion sensor 60 ug LSB!");
   
    // initialize device
    Serial.println("Initializing IMU device...");

    IMU.begin(BMI160GenClass::I2C_MODE, 0x69, 30);
    uint8_t dev_id = IMU.getDeviceID();
    Serial.print("DEVICE ID: 0x");
    Serial.println(dev_id, HEX);
    if (dev_id == 0xd1 )
    {
      Serial.println("BMI160 device Online...");
    }
    else
    {
      while(1) // Loop forever if communication doesn't happen
      {
        Serial.println("Could not connect to BMI160: 0xD1");
        Serial.println("Please check if the hardware device is damaged.");
      }
    }

     // Set the accelerometer range to 2000 degrees/second
    IMU.setGyroRate(3200);
    IMU.setGyroRange(2000);
     // Set the accelerometer range to 2000 degrees/second
    IMU.setAccelerometerRate(1600);
    IMU.setAccelerometerRange(2);
    Serial.println("Initializing IMU device...done."); 
  }
  
  if (LCDEnable) {
    Lcd.setTextColor(RED);
    Lcd.setTextSize(2);
    Lcd.println(" Initialize Done!");
    delay(100);
  }
  if (SerialEnable) {
    Serial.println("Initialize Done!");
  }
}

void AfantorBluefruit52::update() {

  //Button update
  BtnA.read();
  BtnB.read();

}