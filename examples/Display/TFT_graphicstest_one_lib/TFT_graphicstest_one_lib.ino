/*
 Adapted from the Adafruit graphicstest sketch, see orignal header at end
 of sketch.

 This sketch uses the GLCD font (font 1) only.

 Make sure all the display driver and pin comnenctions are correct by
 editting the User_Setup.h file in the eSPI library folder.

 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################
*/

#include <bluefruit52.h>

unsigned long total = 0;
unsigned long tn = 0;
void setup() {

  BF52.begin(true, true, false);

  Serial.println(""); Serial.println("");
  Serial.println("eSPI library test!");

  tn = micros();
  BF52.Lcd.fillScreen(BLACK);

  yield(); Serial.println(F("Benchmark                Time (microseconds)"));

  yield(); Serial.print(F("Screen fill              "));
  yield(); Serial.println(testFillScreen());
  //total+=testFillScreen();
  delay(500);

  yield(); Serial.print(F("Text                     "));
  yield(); Serial.println(testText());
  //total+=testText();
  delay(3000);

  yield(); Serial.print(F("Lines                    "));
  yield(); Serial.println(testLines(CYAN));
  //total+=testLines(CYAN);
  delay(500);

  yield(); Serial.print(F("Horiz/Vert Lines         "));
  yield(); Serial.println(testFastLines(RED, BLUE));
  //total+=testFastLines(RED, BLUE);
  delay(500);

  yield(); Serial.print(F("Rectangles (outline)     "));
  yield(); Serial.println(testRects(GREEN));
  //total+=testRects(GREEN);
  delay(500);

  yield(); Serial.print(F("Rectangles (filled)      "));
  yield(); Serial.println(testFilledRects(YELLOW, MAGENTA));
  //total+=testFilledRects(YELLOW, MAGENTA);
  delay(500);

  yield(); Serial.print(F("Circles (filled)         "));
  yield(); Serial.println(testFilledCircles(10, MAGENTA));
  //total+= testFilledCircles(10, MAGENTA);
  delay(500);
  
  yield(); Serial.print(F("Circles (outline)        "));
  yield(); Serial.println(testCircles(10, WHITE));
  //total+=testCircles(10, WHITE);
  delay(500);

  yield(); Serial.print(F("Triangles (outline)      "));
  yield(); Serial.println(testTriangles());
  //total+=testTriangles();
  delay(500);

  yield(); Serial.print(F("Triangles (filled)       "));
  yield(); Serial.println(testFilledTriangles());
  //total += testFilledTriangles();
  delay(500);

  yield(); Serial.print(F("Rounded rects (outline)  "));
  yield(); Serial.println(testRoundRects());
  //total+=testRoundRects();
  delay(500);

  yield(); Serial.print(F("Rounded rects (filled)   "));
  yield(); Serial.println(testFilledRoundRects());
  //total+=testFilledRoundRects();
  delay(500);

  yield(); Serial.println(F("Done!")); yield();
  //Serial.print(F("Total = ")); Serial.println(total);
  
  //yield();Serial.println(millis()-tn);
}

void loop(void) {
  for (uint8_t rotation = 0; rotation < 8; rotation++) {
    BF52.Lcd.setRotation(rotation);
    Serial.printf("rotation:%d\r\n", rotation);
    testText();
    delay(2000);
  }
}


unsigned long testFillScreen() {
  unsigned long start = micros();
  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.fillScreen(RED);
  BF52.Lcd.fillScreen(GREEN);
  BF52.Lcd.fillScreen(BLUE);
  BF52.Lcd.fillScreen(BLACK);
  return micros() - start;
}

unsigned long testText() {
  BF52.Lcd.fillScreen(BLACK);
  unsigned long start = micros();
  BF52.Lcd.setCursor(0, 0);
  BF52.Lcd.setTextColor(WHITE);  BF52.Lcd.setTextSize(1);
  BF52.Lcd.println("Hello World!");
  BF52.Lcd.setTextColor(YELLOW); BF52.Lcd.setTextSize(2);
  BF52.Lcd.println(1234.56);
  BF52.Lcd.setTextColor(RED);    BF52.Lcd.setTextSize(3);
  BF52.Lcd.println(0xDEADBEEF, HEX);
  BF52.Lcd.println();
  BF52.Lcd.setTextColor(GREEN);
  BF52.Lcd.setTextSize(5);
  BF52.Lcd.println("Groop");
  BF52.Lcd.setTextSize(2);
  BF52.Lcd.println("I implore thee,");
  //BF52.Lcd.setTextColor(GREEN,BLACK);
  BF52.Lcd.setTextSize(1);
  BF52.Lcd.println("my foonting turlingdromes.");
  BF52.Lcd.println("And hooptiously drangle me");
  BF52.Lcd.println("with crinkly bindlewurdles,");
  BF52.Lcd.println("Or I will rend thee");
  BF52.Lcd.println("in the gobberwarts");
  BF52.Lcd.println("with my blurglecruncheon,");
  BF52.Lcd.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = BF52.Lcd.width(),
                h = BF52.Lcd.height();

  BF52.Lcd.fillScreen(BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) BF52.Lcd.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) BF52.Lcd.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  BF52.Lcd.fillScreen(BLACK);

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) BF52.Lcd.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for (y2 = 0; y2 < h; y2 += 6) BF52.Lcd.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  BF52.Lcd.fillScreen(BLACK);

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) BF52.Lcd.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) BF52.Lcd.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  BF52.Lcd.fillScreen(BLACK);

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) BF52.Lcd.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for (y2 = 0; y2 < h; y2 += 6) BF52.Lcd.drawLine(x1, y1, x2, y2, color);

  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = BF52.Lcd.width(), h = BF52.Lcd.height();

  BF52.Lcd.fillScreen(BLACK);
  start = micros();
  for (y = 0; y < h; y += 5) BF52.Lcd.drawFastHLine(0, y, w, color1);
  for (x = 0; x < w; x += 5) BF52.Lcd.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = BF52.Lcd.width()  / 2,
                cy = BF52.Lcd.height() / 2;

  BF52.Lcd.fillScreen(BLACK);
  n     = min(BF52.Lcd.width(), BF52.Lcd.height());
  start = micros();
  for (i = 2; i < n; i += 6) {
    i2 = i / 2;
    BF52.Lcd.drawRect(cx - i2, cy - i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = BF52.Lcd.width()  / 2 - 1,
                cy = BF52.Lcd.height() / 2 - 1;

  BF52.Lcd.fillScreen(BLACK);
  n = min(BF52.Lcd.width(), BF52.Lcd.height());
  for (i = n - 1; i > 0; i -= 6) {
    i2    = i / 2;
    start = micros();
    BF52.Lcd.fillRect(cx - i2, cy - i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    BF52.Lcd.drawRect(cx - i2, cy - i2, i, i, color2);
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = BF52.Lcd.width(), h = BF52.Lcd.height(), r2 = radius * 2;

  BF52.Lcd.fillScreen(BLACK);
  start = micros();
  for (x = radius; x < w; x += r2) {
    for (y = radius; y < h; y += r2) {
      BF52.Lcd.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                      w = BF52.Lcd.width()  + radius,
                      h = BF52.Lcd.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for (x = 0; x < w; x += r2) {
    for (y = 0; y < h; y += r2) {
      BF52.Lcd.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = BF52.Lcd.width()  / 2 - 1,
                      cy = BF52.Lcd.height() / 2 - 1;

  BF52.Lcd.fillScreen(BLACK);
  n     = min(cx, cy);
  start = micros();
  for (i = 0; i < n; i += 5) {
    BF52.Lcd.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      BF52.Lcd.color565(0, 0, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = BF52.Lcd.width()  / 2 - 1,
                   cy = BF52.Lcd.height() / 2 - 1;

  BF52.Lcd.fillScreen(BLACK);
  start = micros();
  for (i = min(cx, cy); i > 10; i -= 5) {
    start = micros();
    BF52.Lcd.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     BF52.Lcd.color565(0, i, i));
    t += micros() - start;
    BF52.Lcd.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     BF52.Lcd.color565(i, i, 0));
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = BF52.Lcd.width()  / 2 - 1,
                cy = BF52.Lcd.height() / 2 - 1;

  BF52.Lcd.fillScreen(BLACK);
  w     = min(BF52.Lcd.width(), BF52.Lcd.height());
  start = micros();
  for (i = 0; i < w; i += 6) {
    i2 = i / 2;
    BF52.Lcd.drawRoundRect(cx - i2, cy - i2, i, i, i / 8, BF52.Lcd.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = BF52.Lcd.width()  / 2 - 1,
                cy = BF52.Lcd.height() / 2 - 1;

  BF52.Lcd.fillScreen(BLACK);
  start = micros();
  for (i = min(BF52.Lcd.width(), BF52.Lcd.height()); i > 20; i -= 6) {
    i2 = i / 2;
    BF52.Lcd.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, BF52.Lcd.color565(0, i, 0));
  }

  return micros() - start;
}

/***************************************************
  Original Adafruit text:

  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

