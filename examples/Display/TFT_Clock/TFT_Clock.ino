/*
 An example analogue clock using a TFT LCD screen to show the time
 use of some of the drawing commands with the library.

 For a more accurate clock, it would be better to use the RTClib library.
 But this is just a demo. 
 
 This sketch uses font 4 only.

 Make sure all the display driver and pin comnenctions are correct by
 editting the User_Setup.h file in the eSPI library folder.

 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################
 
 Based on a sketch by Gilchrist 6/2/2014 1.0
 */

#include <bluefruit52.h> 

#define GREY 0x5AEB

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg=0, mdeg=0, hdeg=0;
uint16_t osx=67, osy=67, omx=67, omy=67, ohx=67, ohy=67;  // Saved H, M, S x & y coords
uint16_t x0=0, x1=0, yy0=0, yy1=0;
uint32_t targetTime = 0;                    // for next 1 second timeout

static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x
uint8_t hh=conv2d(__TIME__), mm=conv2d(__TIME__+3), ss=conv2d(__TIME__+6);  // Get H, M, S from compile time

boolean initial = 1;

void setup(void) {
  BF52.begin(true, true, false);
  // BF52.Lcd.setRotation(0);
  
  //BF52.Lcd.fillScreen(BLACK);
  //BF52.Lcd.fillScreen(RED);
  //BF52.Lcd.fillScreen(GREEN);
  //BF52.Lcd.fillScreen(BLUE);
  //BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.fillScreen(GREY);
  
  BF52.Lcd.setTextColor(WHITE, GREY);  // Adding a background colour erases previous text automatically
  
  // Draw clock face
  BF52.Lcd.fillCircle(120, 67, 66, GREEN);
  BF52.Lcd.fillCircle(120, 67, 60, BLACK);

  // Draw 12 lines
  for(int i = 0; i<360; i+= 30) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*57+120;
    yy0 = sy*57+67;
    x1 = sx*50+120;
    yy1 = sy*50+67;

    BF52.Lcd.drawLine(x0, yy0, x1, yy1, GREEN);
  }

  // Draw 60 dots
  for(int i = 0; i<360; i+= 6) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*51+120;
    yy0 = sy*51+67;
    // Draw minute markers
    BF52.Lcd.drawPixel(x0, yy0, WHITE);
    
    // Draw main quadrant dots
    if(i==0 || i==180) BF52.Lcd.fillCircle(x0, yy0, 2, WHITE);
    if(i==90 || i==270) BF52.Lcd.fillCircle(x0, yy0, 2, WHITE);
  }

  BF52.Lcd.fillCircle(67, 68, 3, WHITE);

  // Draw text at position 120,260 using fonts 4
  // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . - a p m
  // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .

  targetTime = millis() + 1000; 
}

void loop() {
  if (targetTime < millis()) {
    targetTime += 1000;
    ss++;              // Advance second
    if (ss==60) {
      ss=0;
      mm++;            // Advance minute
      if(mm>59) {
        mm=0;
        hh++;          // Advance hour
        if (hh>23) {
          hh=0;
        }
      }
    }

    // Pre-compute hand degrees, x & y coords for a fast screen update
    sdeg = ss*6;                  // 0-59 -> 0-354
    mdeg = mm*6+sdeg*0.01666667;  // 0-59 -> 0-360 - includes seconds
    hdeg = hh*30+mdeg*0.0833333;  // 0-11 -> 0-360 - includes minutes and seconds
    hx = cos((hdeg-90)*0.0174532925);    
    hy = sin((hdeg-90)*0.0174532925);
    mx = cos((mdeg-90)*0.0174532925);    
    my = sin((mdeg-90)*0.0174532925);
    sx = cos((sdeg-90)*0.0174532925);    
    sy = sin((sdeg-90)*0.0174532925);

    if (ss==0 || initial) {
      initial = 0;
      // Erase hour and minute hand positions every minute
      BF52.Lcd.drawLine(ohx, ohy, 120, 67, BLACK);
      ohx = hx*34+121;    
      ohy = hy*34+67;
      BF52.Lcd.drawLine(omx, omy, 120, 67, BLACK);
      omx = mx*46+120;    
      omy = my*46+67;
    }

      // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
      BF52.Lcd.drawLine(osx, osy, 120, 67, BLACK);
      osx = sx*46+121;    
      osy = sy*46+67;
      BF52.Lcd.drawLine(osx, osy, 120, 67, RED);
      BF52.Lcd.drawLine(ohx, ohy, 120, 67, WHITE);
      BF52.Lcd.drawLine(omx, omy, 120, 67, WHITE);
      BF52.Lcd.drawLine(osx, osy, 120, 67, RED);

    BF52.Lcd.fillCircle(120, 67, 3, RED);
  }
}

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

