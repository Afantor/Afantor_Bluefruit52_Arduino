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

void AfantorBluefruit52::begin(bool LCDEnable, bool SerialEnable) {

  // Correct init once
  if (isInited) return;
  else isInited = true;

  // UART
  if (SerialEnable) {
    Serial.begin(115200);
    Serial.flush();
    delay(50);
    Serial.print("Bluefruit52 initializing...");
  }

  // LCD INIT
  if (LCDEnable) {
    //Lcd.begin();
  }

  // I2C init
  #ifdef Bluefruit52_FIRE
  Wire.begin(25, 26);
  #endif

  if (SerialEnable) {
    Serial.println("OK");
  }
}

void AfantorBluefruit52::update() {

  //Button update
  BtnA.read();
  BtnB.read();

}