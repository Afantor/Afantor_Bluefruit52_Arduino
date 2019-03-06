/**************************************************************************/
/*!
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
/**
 * \par Copyright (C), 2019, Afantor
 * \class Bluefruit52
 * \brief   Bluefruit52 library.
 * @file    bluefruit52.h
 * @author  Afantor
 * @version V0.0.1
 * @date    2019/01/01
 * @brief   Header for bluefruit52.cpp module
 *
 * \par Description
 * This file is a drive for Bluefruit52 core.
 *
 * \par Method List:
 *    
 *  System:
        Bluefruit52.begin();
        Bluefruit52.update();

    LCD:

    Button:
        Bluefruit52.BtnA/B.read();
        Bluefruit52.BtnA/B.isPressed();
        Bluefruit52.BtnA/B.isReleased();
        Bluefruit52.BtnA/B.wasPressed();
        Bluefruit52.BtnA/B.wasReleased();
        Bluefruit52.BtnA/B.wasreleasedFor()
        Bluefruit52.BtnA/B.pressedFor(uint32_t ms);
        Bluefruit52.BtnA/B.releasedFor(uint32_t ms);
        Bluefruit52.BtnA/B.lastChange();
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Afantor         2019/01/01           0.0.1          Rebuild the new.
 * </pre>
 *
 */
#ifndef _BLUEFRUIT52_H_
#define _BLUEFRUIT52_H_

#include <Arduino.h>
#include <bluefruit.h>

#include "utility/config.h"
#include "utility/button.h"
#include "utility/MPU6050.h"
#include "utility/ST7789.h"

class AfantorBluefruit52
{
  public:
    AfantorBluefruit52(void); // Constructor
    void begin(bool LCDEnable=true, bool SerialEnable=true);
    void update();
    // Button API
    #define DEBOUNCE_MS 10
    Button BtnA = Button(BUTTON_A_PIN, true, DEBOUNCE_MS);
    Button BtnB = Button(BUTTON_B_PIN, true, DEBOUNCE_MS);
    
    //MPU6050
    MPU6050 IMU = MPU6050();
  private:
    bool isInited;
};

extern AfantorBluefruit52 Bluefruit52;

#endif
