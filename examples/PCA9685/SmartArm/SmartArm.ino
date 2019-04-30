/*************************************************** 
  This is an example for our PCA9685 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

 ****************************************************/

#include <Wire.h>
#include <pca9685.h>



// called this way, it uses the default address 0x40
PCA9685 pwm = PCA9685();


// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  650 // this is the 'maximum' pulse length count (out of 4096)

#define L1 135
#define L5 147

#define PI 3.1416

// our servo # counter
uint8_t servonum = 0;

int Bpin = A0;  // P0.28 Pin analog pin used to connect the potentiometer
int Xpin = A2;
int Ypin = A1;

int sAngle0 = 0;
int sAngle1 = 0;
int sAngle2 = 0;
int sAngle90 = 0;

int sAngle0_offset = 0;
int sAngle1_offset = 0;
int sAngle2_offset = 0;
int sAngle90_offset = 0;

int valB;    // variable to read the value from the analog pin
int valX;
int valY;

double horzA = 1.01;
double vertA = 1.01;

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 53;   // 50 Hz
  // Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  // Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;  // convert to us
  pulse /= pulselength;
  // Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void setup() {
  Serial.begin(115200);
  Serial.println("4 channel Servo test!");
  pinMode(Bpin, INPUT_PULLUP);

  pwm.begin();
  
  pwm.setPWMFreq(53);  // Analog servos run at ~50 Hz updates
  delay(100);
  valB = digitalRead(Bpin);
  valX = analogRead(Xpin);
  valY = analogRead(Ypin);
  // Serial.println(valX-10);
  // Serial.println(valY-20);
  sAngle0_offset = map(1000-valY, 0, 1000, 0, 180);
  sAngle1_offset = map(1000-valX, 0, 1000, 0, 180);
  sAngle2_offset = map(valX/2, 0, 1000, 0, 180);
  sAngle90_offset = 0;
  sAngle2_offset = sAngle2_offset + 100;
  setServoAngle(0, sAngle0_offset);
  setServoAngle(1, sAngle1_offset);
  setServoAngle(2, sAngle2_offset);
  setServoAngle(3, sAngle90_offset);
  Serial.println(atan(1.0)*180/PI);
  delay(1000);
}

void loop() {
  valB = digitalRead(Bpin);
  valX = analogRead(Xpin);
  valY = analogRead(Ypin);

  // sAngle0 = map(1000-valY, 0, 1000, 0, 180);
  // sAngle1 = map(1000-valX, 0, 1000, 0, 180);
  // sAngle2 = map(valX/2, 0, 1000, 0, 180);
  // // Serial.println(sAngle0);
  // setServoAngle(0, sAngle0);
  // setServoAngle(1, sAngle1);
  // setServoAngle(2, sAngle2+100);

  sAngle0 = map(valY, 0, 1000, 0, 300);
  sAngle1 = map(valX, 0, 1000, 0, 290);

  pos_to_angle (&horzA,&vertA, sAngle0, sAngle1);
  Serial.println(vertA*180/PI);
  setServoAngle(1, sAngle1_offset + vertA*180/PI);
  setServoAngle(2, sAngle2_offset + horzA*180/PI);

  if(valB == 1){
      setServoAngle(3, 40);
  }
  else
  {
    setServoAngle(3, 170);
  }
  delay(20);
}

void setServoAngle(uint8_t n, float angle) {
  double pulse = angle;
  pulse = pulse/90 + 0.5;
  setServoPulse(n,pulse);
}

void pos_to_angle (double * horz_angle,double * vert_angle,double radius,double height)
{
  double alpha1 = atan(height/radius);
  double bf_2 = radius * radius + height * height;
  double alpha2 = acos((bf_2 + L1 * L1  -  L5 * L5)/( 2 * sqrt(bf_2)* L1));
  * horz_angle = PI/2 - (alpha1 + alpha2);
  double beta2 = acos((L1 * L1 + L5 * L5-bf_2)/(2 * L1 * L5));
  * vert_angle = PI - beta2 - alpha1 - alpha2;
}