/*
 Controlling a servo position using a potentiometer (variable resistor)

*/

#include <Servo.h>

Servo servo0;  // create servo object to control a servo
Servo servo1;
Servo servo2;

int potpin = A0;  // P0.28 Pin analog pin used to connect the potentiometer
int Xpin = A2;
int Ypin = A1;

int valPot;    // variable to read the value from the analog pin
int valX;
int valY;

void setup() {
    Serial.begin(115200);
  	servo0.attach(11);  // attaches the servo on P0.11 Pin to the servo object
    servo1.attach(12);  
    servo2.attach(13);  
}

void loop() {
  	valPot = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    Serial.print("Pot: ");
    Serial.println(valPot);  	
    valPot = map(valPot, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)

    valX = analogRead(Xpin);            // reads the value of the potentiometer (value between 0 and 1023)
    Serial.print("Xval: ");
    Serial.println(valX);
    valX = map(valX, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)

    valY = analogRead(Ypin);            // reads the value of the potentiometer (value between 0 and 1023)
    Serial.print("Yval: ");
    Serial.println(valY);  	
    valY = map(valY, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)

    servo0.write(valPot);                  // sets the servo position according to the scaled value
  	servo1.write(valX);
  	servo2.write(valY);
  	delay(15);                           // waits for the servo to get there
}
