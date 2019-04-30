/*  
 Test the BF52.Lcd.print() viz embedded BF52.Lcd.write() function

 This sketch used font 2, 4, 7

 Make sure all the display driver and pin comnenctions are correct by
 editting the User_Setup.h file in the BF52.eSPI library folder.

 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################
 */

#include <bluefruit52.h>

int tempInt = 0;
float tempF = 1.01;

void setup(void) {
  BF52.begin(true, true, false);
}

void loop() {
  tempInt = 0;
  tempF = 1.01;
  // Fill screen with grey so we can see the effect of printing with and without 
  // a background colour defined
  BF52.Lcd.fillScreen(BLACK);
  
  // Set "cursor" at top left corner of display (0,0) and select font 2
  // (cursor will move to next line automatically during printing with 'BF52.Lcd.println'
  //  or stay on the line is there is room for the text with BF52.Lcd.print)
  BF52.Lcd.setCursor(0, 0);
  // Set the font colour to be white with a black background, set text size multiplier to 1
  BF52.Lcd.setTextColor(WHITE,BLACK);  
  BF52.Lcd.setTextSize(1);
  // We can now plot text on screen using the "print" class
  BF52.Lcd.println("LCD draw Number Test!");
  
  // Set the font colour to be yellow with no background, set to font 7
  BF52.Lcd.setTextColor(YELLOW); 
  BF52.Lcd.setTextSize(2);
  BF52.Lcd.println(1234.56);
  
  BF52.Lcd.drawChar(0, 40, 'A', RED, BLACK, 2);
  BF52.Lcd.drawNumber(0, 80, 102, 3, BLUE, BLACK, 2);
  BF52.Lcd.drawFloat(0, 110, 10.89, 4, YELLOW, BLACK, 2);
  // BF52.Lcd.drawString(0, 80, "Hello BLE", WHITE, BLACK, 4);
  for(int i=0; i<900; i++){
    tempInt++;
    tempF+=0.2;
    BF52.Lcd.drawNumber(100, 80, tempInt, 3, BLUE, BLACK, 2);
    BF52.Lcd.drawFloat(100, 110, tempF, 4, YELLOW, BLACK, 2);
    delay(100);
  }
  delay(1000);
}



