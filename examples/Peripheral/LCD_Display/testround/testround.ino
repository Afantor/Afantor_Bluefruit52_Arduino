/***************************************************
  This is a library for the ST7789 IPS SPI display.

  This example code is in the public domain.

  2019/01/05
  by Afantor
 ****************************************************/


#include <bluefruit52.h> // Hardware-specific library for ST7789 (with or without CS pin)


float p = 3.1415926;

void setup(void) {
  BF52.begin(true, true, false);
  Serial.print("Hello! ST7789 TFT Test");

  Serial.println("Initialized");

  uint16_t time = millis();
  BF52.Lcd.fillScreen(BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  // a single pixel
  BF52.Lcd.drawPixel(BF52.Lcd.width()/2, BF52.Lcd.height()/2, GREEN);
  delay(500);

  testroundrects();
  delay(500);

  Serial.println("done");
  delay(1000);
}

void loop() {
  delay(500);
}


void testroundrects() {
  BF52.Lcd.fillScreen(BLACK);
  int color = 100;
  int i;
  int t;
  for(t = 0 ; t <= 4; t+=1) {
    int x = 0;
    int y = 0;
    int w = BF52.Lcd.width()-2;
    int h = BF52.Lcd.height()-2;
    for(i = 0 ; i <= 16; i+=1) {
      BF52.Lcd.drawRoundRect(x, y, w, h, 5, color);
      x+=2;
      y+=3;
      w-=4;
      h-=6;
      color+=1100;
    }
    color+=100;
  }
}

