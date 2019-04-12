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


void setup(void) {
  BF52.begin(true, true, false);
}

void loop() {
  
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
  BF52.Lcd.println("Hello World!");
  
  // Set the font colour to be yellow with no background, set to font 7
  BF52.Lcd.setTextColor(YELLOW); 
  BF52.Lcd.setTextSize(6);
  BF52.Lcd.println(1234.56);
  
  // Set the font colour to be red with black background, set to font 4
  BF52.Lcd.setTextColor(RED,BLACK);    
  BF52.Lcd.setTextSize(3);
  BF52.Lcd.println(37359285L, HEX); // Should print 23A0EB5

  // Set the font colour to be green with black background, set to font 4
  BF52.Lcd.setTextColor(GREEN,BLACK);
  BF52.Lcd.setTextSize(3);
  BF52.Lcd.println("Groop");
  BF52.Lcd.println("I implore thee,");

  delay(5000);
  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.setCursor(0, 0);
  // Change to font 2
  BF52.Lcd.setTextSize(1);
  BF52.Lcd.println("my foonting turlingdromes.");
  BF52.Lcd.println("And hooptiously drangle me");
  BF52.Lcd.println("with crinkly bindlewurdles,");
  // This next line is deliberately made too long for the display width to test
  // automatic text wrapping onto the next line
  BF52.Lcd.println("Or I will rend thee in the");
  BF52.Lcd.println("gobberwarts with my blurglecruncheon,");
  BF52.Lcd.println("see if I don't!");

  delay(5000);
  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.setCursor(0, 0);  
  // Test some print formatting functions
  float fnumber = 123.45;
   // Set the font colour to be blue with no background, set to font 4
  BF52.Lcd.setTextColor(BLUE);    
  BF52.Lcd.setTextSize(2);
  BF52.Lcd.print("Float = "); 
  BF52.Lcd.println(fnumber);           // Print floating point number
  BF52.Lcd.print("Binary = "); 
  BF52.Lcd.println((int)fnumber, BIN); // Print as integer value in binary
  BF52.Lcd.print("Hexadecimal = "); 
  BF52.Lcd.println((int)fnumber, HEX); // Print as integer number in Hexadecimal
  delay(5000);
}



