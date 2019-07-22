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
        BF52.Lcd.setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
        BF52.Lcd.pushColor(uint16_t color);
        BF52.Lcd.fillScreen(uint16_t color);
        BF52.Lcd.drawPixel(int16_t x, int16_t y, uint16_t color);
        BF52.Lcd.drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
        BF52.Lcd.drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
        BF52.Lcd.fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
        BF52.Lcd.setRotation(uint8_t r);
        BF52.Lcd.invertDisplay(boolean i);
        BF52.Lcd.init(uint16_t width, uint16_t height);

        BF52.Lcd.drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color),
        BF52.Lcd.drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
        BF52.Lcd.drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color),
        BF52.Lcd.drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,uint16_t color),
        BF52.Lcd.fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color),
        BF52.Lcd.fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,int16_t delta, uint16_t color),
        BF52.Lcd.drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color),
        BF52.Lcd.fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color),
        BF52.Lcd.drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color),
        BF52.Lcd.fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color),
        BF52.Lcd.drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color),
        BF52.Lcd.drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color, uint16_t bg),
        BF52.Lcd.drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color),
        BF52.Lcd.drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg),
        BF52.Lcd.drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color),
        BF52.Lcd.drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h),
        BF52.Lcd.drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h),
        BF52.Lcd.drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], const uint8_t mask[], int16_t w, int16_t h),
        BF52.Lcd.drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint8_t *mask, int16_t w, int16_t h),
        BF52.Lcd.drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h),
        BF52.Lcd.drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h),
        BF52.Lcd.drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], const uint8_t mask[], int16_t w, int16_t h),
        BF52.Lcd.drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, uint8_t *mask, int16_t w, int16_t h),
        BF52.Lcd.drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size),
        BF52.Lcd.drawText(char *text, uint16_t color),
        BF52.Lcd.setCursor(int16_t x, int16_t y),
        BF52.Lcd.setTextColor(uint16_t c),
        BF52.Lcd.setTextColor(uint16_t c, uint16_t bg),
        BF52.Lcd.setTextSize(uint8_t s),
        BF52.Lcd.setTextWrap(boolean w),
        BF52.Lcd.setFont(const GFXfont *f = NULL),
        BF52.Lcd.getTextBounds(const char *string, int16_t x, int16_t y,int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h),
        BF52.Lcd.getTextBounds(const __FlashStringHelper *s, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h),
        BF52.Lcd.getTextBounds(const String &str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
             
        BF52.Lcd.printf();
        BF52.Lcd.print();
        BF52.Lcd.println();
        BF52.Lcd.drawCenterString(const char *string, int dX, int poY, int font);
        BF52.Lcd.drawRightString(const char *string, int dX, int poY, int font);
        BF52.Lcd.drawJpg(const uint8_t *jpg_data, size_t jpg_len, uint16_t x, uint16_t y);
        BF52.Lcd.drawJpgFile(fs::FS &fs, const char *path, uint16_t x, uint16_t y);
        BF52.Lcd.drawBmpFile(fs::FS &fs, const char *path, uint16_t x, uint16_t y);

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
#include <Wire.h>
#include <bluefruit.h>

#include "utility/config.h"
#include "utility/button.h"
#include "utility/ST7789.h"
#include "utility/BMI160.h"
#include "utility/BMI160Gen.h"
#include "utility/CurieIMU.h"

class AfantorBluefruit52
{
  public:
    AfantorBluefruit52(void); // Constructor
    void begin( bool SerialEnable=true, bool LCDEnable=true, bool IMUEnable=false);
    void update();
    // Button API
    #define DEBOUNCE_MS 10
    Button BtnA = Button(BUTTON_A_PIN, true, DEBOUNCE_MS);
    Button BtnB = Button(BUTTON_B_PIN, true, DEBOUNCE_MS);

    // LCD
    LCD_ST7789 Lcd = LCD_ST7789(LCD_DC_PIN, LCD_RST_PIN, LCD_SDA_PIN, LCD_SCL_PIN);

    //BMI160
    BMI160GenClass IMU;

  private:
    bool isInited;
};

extern AfantorBluefruit52 Bluefruit52;
#define BF52 Bluefruit52
#define lcd Lcd

#endif
