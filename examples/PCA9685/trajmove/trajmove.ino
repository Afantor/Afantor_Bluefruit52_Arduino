#include <Wire.h>
#include <pca9685.h>
#include <math.h>

#define L1 135 //lengths are in mm
#define L2 147
#define L3 95
#define PI 3.14159

// called this way, it uses the default address 0x40
PCA9685 pwm = PCA9685();

int potpinx = 0;  // analog pin used to connect the potentiometer
int valx;    // variable to read the value from the analog pin
int potpiny = 1;  // analog pin used to connect the potentiometer
int valy;    // variable to read the value from the analog pin

double x;
double y;
 
// our servo # counter
uint8_t servonum = 0;

int Bpin = 9;  // P0.28 Pin analog pin used to connect the potentiometer
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
  pulselength /= 60;   // 50 Hz
  // Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  // Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;  // convert to us
  pulse /= pulselength;
  // Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void setServoAngle(uint8_t n, float angle) {
  double pulse = angle;
  pulse = pulse/90.0 + 0.5;
  setServoPulse(n,pulse);
}

// move gripper to x,y,z,gripangle
void movexy(double x1,double y1)
{
    //inverse kinematics for joint angles (RADIANS)
    double t2 = PI - acos((L1 * L1 + L2 * L2 - x1 * x1 - y1 * y1 ) / (2 * L1 * L2));
    double t1 = atan2(y1, x1) + atan2((L2*sin(t2)),(L1+(L2*cos(t2))));

    //convert to degrees
    double t1deg = t1 / PI * 180;
    double t2deg = t2 / PI * 180;

    //convert joint angles to servo command angles
    int h = (360 - (2*t2deg))/2;
    int b = 15;
    int t1servo = 180 - t1deg;
    int t2servo = t1deg + h - b;

    setServoAngle(1, t1servo);
    setServoAngle(2, t2servo);
    // delay(20);  // change this to make the motions faster or slower 
}

void setup(){
  Serial.begin(115200);
  Serial.println("4 channel Servo test!");
  pinMode(Bpin, INPUT_PULLUP);

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~50 Hz updates
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
  delay(500);

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

  sAngle0 = map(valY, 20, 1000, 0, 250);
  sAngle1 = map(valX, 20, 1000, 0, 225);

  movexy(sAngle0,sAngle1);

  if(valB == 1){
      setServoAngle(3, 40);
      
  }
  else
  {
    setServoAngle(3, 170);
    Serial.println("Hello");
  }
  delay(20);  
}
