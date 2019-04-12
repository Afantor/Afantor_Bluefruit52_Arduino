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

  // large block of text
  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.drawText("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", WHITE);
  delay(2000);

  // tft print function
  tftPrintTest();
  delay(4000);

  // a single pixel
  BF52.Lcd.drawPixel(BF52.Lcd.width()/2, BF52.Lcd.height()/2, GREEN);
  delay(1000);

  // line draw test
  testlines(YELLOW);
  delay(500);

  // optimized lines
  testfastlines(RED, BLUE);
  delay(500);

  testdrawrects(GREEN);
  delay(500);

  testfillrects(YELLOW, MAGENTA);
  delay(500);

  BF52.Lcd.fillScreen(BLACK);
  testfillcircles(10, BLUE);
  testdrawcircles(10, WHITE);
  delay(500);

  testroundrects();
  delay(500);

  testtriangles();
  delay(500);

  mediabuttons();
  delay(500);

  Serial.println("done");
  delay(1000);
}

void loop() {
  //BF52.Lcd.invertDisplay(true);
  delay(500);
  //BF52.Lcd.invertDisplay(false);
  delay(500);
}

void testlines(uint16_t color) {
  BF52.Lcd.fillScreen(BLACK);
  for (int16_t x=0; x < BF52.Lcd.width(); x+=6) {
    BF52.Lcd.drawLine(0, 0, x, BF52.Lcd.height()-1, color);
  }
  for (int16_t y=0; y < BF52.Lcd.height(); y+=6) {
    BF52.Lcd.drawLine(0, 0, BF52.Lcd.width()-1, y, color);
  }

  BF52.Lcd.fillScreen(BLACK);
  for (int16_t x=0; x < BF52.Lcd.width(); x+=6) {
    BF52.Lcd.drawLine(BF52.Lcd.width()-1, 0, x, BF52.Lcd.height()-1, color);
  }
  for (int16_t y=0; y < BF52.Lcd.height(); y+=6) {
    BF52.Lcd.drawLine(BF52.Lcd.width()-1, 0, 0, y, color);
  }

  BF52.Lcd.fillScreen(BLACK);
  for (int16_t x=0; x < BF52.Lcd.width(); x+=6) {
    BF52.Lcd.drawLine(0, BF52.Lcd.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < BF52.Lcd.height(); y+=6) {
    BF52.Lcd.drawLine(0, BF52.Lcd.height()-1, BF52.Lcd.width()-1, y, color);
  }

  BF52.Lcd.fillScreen(BLACK);
  for (int16_t x=0; x < BF52.Lcd.width(); x+=6) {
    BF52.Lcd.drawLine(BF52.Lcd.width()-1, BF52.Lcd.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < BF52.Lcd.height(); y+=6) {
    BF52.Lcd.drawLine(BF52.Lcd.width()-1, BF52.Lcd.height()-1, 0, y, color);
  }
}


void testfastlines(uint16_t color1, uint16_t color2) {
  BF52.Lcd.fillScreen(BLACK);
  for (int16_t y=0; y < BF52.Lcd.height(); y+=5) {
    BF52.Lcd.drawFastHLine(0, y, BF52.Lcd.width(), color1);
  }
  for (int16_t x=0; x < BF52.Lcd.width(); x+=5) {
    BF52.Lcd.drawFastVLine(x, 0, BF52.Lcd.height(), color2);
  }
}

void testdrawrects(uint16_t color) {
  BF52.Lcd.fillScreen(BLACK);
  for (int16_t x=0; x < BF52.Lcd.width(); x+=6) {
    BF52.Lcd.drawRect(BF52.Lcd.width()/2 -x/2, BF52.Lcd.height()/2 -x/2 , x, x, color);
  }
}

void testfillrects(uint16_t color1, uint16_t color2) {
  BF52.Lcd.fillScreen(BLACK);
  for (int16_t x=BF52.Lcd.width()-1; x > 6; x-=6) {
    BF52.Lcd.fillRect(BF52.Lcd.width()/2 -x/2, BF52.Lcd.height()/2 -x/2 , x, x, color1);
    BF52.Lcd.drawRect(BF52.Lcd.width()/2 -x/2, BF52.Lcd.height()/2 -x/2 , x, x, color2);
  }
}

void testfillcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=radius; x < BF52.Lcd.width(); x+=radius*2) {
    for (int16_t y=radius; y < BF52.Lcd.height(); y+=radius*2) {
      BF52.Lcd.fillCircle(x, y, radius, color);
    }
  }
}

void testdrawcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=0; x < BF52.Lcd.width()+radius; x+=radius*2) {
    for (int16_t y=0; y < BF52.Lcd.height()+radius; y+=radius*2) {
      BF52.Lcd.drawCircle(x, y, radius, color);
    }
  }
}

void testtriangles() {
  BF52.Lcd.fillScreen(BLACK);
  int color = 0xF800;
  int t;
  int w = BF52.Lcd.width()/2;
  int x = BF52.Lcd.height()-1;
  int y = 0;
  int z = BF52.Lcd.width();
  for(t = 0 ; t <= 15; t++) {
    BF52.Lcd.drawTriangle(w, y, y, x, z, x, color);
    x-=4;
    y+=4;
    z-=4;
    color+=100;
  }
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

void tftPrintTest() {
  BF52.Lcd.setTextWrap(false);
  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.setCursor(0, 30);
  BF52.Lcd.setTextColor(RED);
  BF52.Lcd.setTextSize(1);
  BF52.Lcd.println("Hello World!");
  BF52.Lcd.setTextColor(YELLOW);
  BF52.Lcd.setTextSize(2);
  BF52.Lcd.println("Hello World!");
  BF52.Lcd.setTextColor(GREEN);
  BF52.Lcd.setTextSize(3);
  BF52.Lcd.println("Hello World!");
  BF52.Lcd.setTextColor(BLUE);
  BF52.Lcd.setTextSize(4);
  BF52.Lcd.print(1234.567);
  delay(1500);
  BF52.Lcd.setCursor(0, 0);
  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.setTextColor(WHITE);
  BF52.Lcd.setTextSize(0);
  BF52.Lcd.println("Hello World!");
  BF52.Lcd.setTextSize(1);
  BF52.Lcd.setTextColor(GREEN);
  BF52.Lcd.print(p, 6);
  BF52.Lcd.println(" Want pi?");
  BF52.Lcd.println(" ");
  BF52.Lcd.print(8675309, HEX); // print 8,675,309 out in HEX!
  BF52.Lcd.println(" Print HEX!");
  BF52.Lcd.println(" ");
  BF52.Lcd.setTextColor(WHITE);
  BF52.Lcd.println("Sketch has been");
  BF52.Lcd.println("running for: ");
  BF52.Lcd.setTextColor(MAGENTA);
  BF52.Lcd.print(millis() / 1000);
  BF52.Lcd.setTextColor(WHITE);
  BF52.Lcd.print(" seconds.");
}

void mediabuttons() {
  // play  
  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.fillRoundRect(25, 40, 78, 60, 8, WHITE);
  BF52.Lcd.fillTriangle(42, 50, 42, 90, 90, 70, RED);
  delay(500);  
  // pause
  BF52.Lcd.fillRoundRect(140, 40, 78, 60, 8, WHITE);
  BF52.Lcd.fillRoundRect(154, 48, 20, 45, 5, BLUE);
  BF52.Lcd.fillRoundRect(184, 48, 20, 45, 5, BLUE);
  delay(500);
  // play color
  BF52.Lcd.fillTriangle(42, 50, 42, 90, 90, 70, GREEN);
  delay(50);
  // pause color
  BF52.Lcd.fillRoundRect(154, 48, 20, 45, 5, RED);
  BF52.Lcd.fillRoundRect(184, 48, 20, 45, 5, RED);
  // play color
  BF52.Lcd.fillTriangle(42, 50, 42, 90, 90, 70, BLUE);
}