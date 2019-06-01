/*************************************************** 
  This is an example for Wireless Joystick Board.

 ****************************************************/

#include <bluefruit52.h>

#define JOYSTICK_START    11
#define JOYSTICK_MODE     18
#define JOYSTICK_KL       14
#define JOYSTICK_KR       13
#define JOYSTICK_BTN_L    12
#define JOYSTICK_BTN_R    20
#define JOYSTICK_BEEP     19
#define JOYSTICK_POT1     A4
#define JOYSTICK_POT2     A5
#define JOYSTICK_POT3     A6
#define JOYSTICK_LX       A1
#define JOYSTICK_LY       A0
#define JOYSTICK_RX       A3
#define JOYSTICK_RY       A2

void setup() {

  BF52.begin(true, true, false);

  Serial.println(""); 
  Serial.println("");
  Serial.println("Joystick Board test!");

  pinMode(JOYSTICK_START, INPUT);
  pinMode(JOYSTICK_MODE, INPUT);
  pinMode(JOYSTICK_KL, INPUT);
  pinMode(JOYSTICK_KR, INPUT);
  pinMode(JOYSTICK_BTN_L, INPUT);
  pinMode(JOYSTICK_BTN_R, INPUT);
  pinMode(JOYSTICK_BEEP, OUTPUT);

  delay(1000);
}

void loop() {
  // Serial.println(digitalRead(JOYSTICK_KL));
  Serial.println(analogRead(JOYSTICK_LX));
  delay(50);
}

