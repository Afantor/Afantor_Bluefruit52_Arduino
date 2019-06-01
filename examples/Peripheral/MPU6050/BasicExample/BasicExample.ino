/* MPU6050 Basic Example Code
 by: Afantor
 date: 2019/01/05
 
 Demonstrate  MPU-6050 basic functionality including initialization, accelerometer trimming, sleep mode functionality as well as
 parameterizing the register addresses. Added display functions to allow display to on breadboard monitor. 
 No DMP use. We just want to get out the accelerations, temperature, and gyro readings.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors worked for me. They should be on the breakout
 board.
 
 Hardware setup:
 MPU6050 Breakout --------- Bluefruit
 3.3V --------------------- 3.3V
 SDA ----------------------- 25
 SCL ----------------------- 26
 GND ---------------------- GND
 
 */
#include <bluefruit52.h>

MPU6050 IMU;

void setup()
{
  BF52.begin(true, true, true);

  Serial.println("MPU6050");
  Serial.println("6-DOF 16-bit");
  Serial.println("motion sensor");
  Serial.println("60 ug LSB");

  BF52.Lcd.fillScreen(BLACK);
  BF52.Lcd.setTextColor(WHITE); // Set pixel color; 1 on the monochrome screen
  BF52.Lcd.setTextSize(2);
  BF52.Lcd.setCursor(0,0); BF52.Lcd.println("MPU6050");
  BF52.Lcd.setTextSize(1);
  BF52.Lcd.setCursor(0, 20); BF52.Lcd.println("6-DOF 16-bit");
  BF52.Lcd.setCursor(0, 30); BF52.Lcd.println("motion sensor");
  BF52.Lcd.setCursor(20,40); BF52.Lcd.println("60 ug LSB");
  delay(1000);

  // Set up for data display
  BF52.Lcd.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  BF52.Lcd.fillScreen(BLACK);   // clears the screen and buffer

   // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = IMU.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  Serial.print("I AM ");
  Serial.print(c, HEX);  
  Serial.print(" I Should Be ");
  Serial.println(0x68, HEX); 

  BF52.Lcd.setCursor(20,0); BF52.Lcd.print("MPU6050");
  BF52.Lcd.setCursor(0,10); BF52.Lcd.print("I AM");
  BF52.Lcd.setCursor(30,10); BF52.Lcd.print(c, HEX);
  BF52.Lcd.setCursor(0,20); BF52.Lcd.print("I Should Be 0x");
  BF52.Lcd.setCursor(84,20); BF52.Lcd.print(0x68, HEX);
  delay(1000);

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU6050 is online...");
    
    IMU.MPU6050SelfTest(IMU.SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); 
    Serial.print(IMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); 
    Serial.print(IMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); 
    Serial.print(IMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); 
    Serial.print(IMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); 
    Serial.print(IMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); 
    Serial.print(IMU.SelfTest[5],1); Serial.println("% of factory value");

    if(IMU.SelfTest[0] < 1.0f && IMU.SelfTest[1] < 1.0f && IMU.SelfTest[2] < 1.0f && IMU.SelfTest[3] < 1.0f && IMU.SelfTest[4] < 1.0f && IMU.SelfTest[5] < 1.0f) 
    {
      Serial.println("Pass Selftest!");  
      
      IMU.calibrateMPU6050(IMU.gyroBias, IMU.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
      
      BF52.Lcd.fillScreen(BLACK);
      BF52.Lcd.setTextSize(1);
      BF52.Lcd.setCursor(0, 0); BF52.Lcd.print("MPU6050 bias");
      BF52.Lcd.setCursor(0, 16); BF52.Lcd.print(" x   y   z  ");
  
      BF52.Lcd.setCursor(0,  32); BF52.Lcd.print((int)(1000*IMU.accelBias[0]));
      BF52.Lcd.setCursor(32, 32); BF52.Lcd.print((int)(1000*IMU.accelBias[1]));
      BF52.Lcd.setCursor(64, 32); BF52.Lcd.print((int)(1000*IMU.accelBias[2]));
      BF52.Lcd.setCursor(96, 32); BF52.Lcd.print("mg");
  
      BF52.Lcd.setCursor(0,  48); BF52.Lcd.print(IMU.gyroBias[0], 1);
      BF52.Lcd.setCursor(32, 48); BF52.Lcd.print(IMU.gyroBias[1], 1);
      BF52.Lcd.setCursor(64, 48); BF52.Lcd.print(IMU.gyroBias[2], 1);
      BF52.Lcd.setCursor(96, 48); BF52.Lcd.print("o/s");
      delay(1000);      

      IMU.initMPU6050(); 
      Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    }
    else
    {
      Serial.print("Could not connect to MPU6050: 0x");
      Serial.println(c, HEX);
      while(1) ; // Loop forever if communication doesn't happen
    }
  }
  BF52.Lcd.setTextSize(2);
  BF52.Lcd.setTextColor(BLUE);
  BF52.Lcd.fillScreen(BLACK);  
  BF52.Lcd.setCursor(0, 0); BF52.Lcd.print("MPU6050 Date:");

  BF52.Lcd.drawChar(0, 20, 'X', GREEN, BLACK, 2);
  BF52.Lcd.drawChar(70, 20, 'Y', GREEN, BLACK, 2);
  BF52.Lcd.drawChar(140, 20, 'Z', GREEN, BLACK, 2);
  BF52.Lcd.drawChar(190, 40, 'm', GREEN, BLACK, 2);
  BF52.Lcd.drawChar(202, 40, 'g', GREEN, BLACK, 2);
  BF52.Lcd.drawChar(190, 60, 'o', GREEN, BLACK, 2);
  BF52.Lcd.drawChar(202, 60, '/', GREEN, BLACK, 2);
  BF52.Lcd.drawChar(214, 60, 's', GREEN, BLACK, 2);

  BF52.Lcd.drawChar(0, 100, 'T', RED, BLACK, 2);
  BF52.Lcd.drawChar(12, 100, 'e', RED, BLACK, 2);
  BF52.Lcd.drawChar(24, 100, 'm', RED, BLACK, 2);
  BF52.Lcd.drawChar(36, 100, 'p', RED, BLACK, 2);
  BF52.Lcd.drawChar(48, 100, ':', RED, BLACK, 2);
  BF52.Lcd.drawChar(190, 100, 'C', GREEN, BLACK, 2);

  // BF52.Lcd.setCursor(0, 32); BF52.Lcd.print(" X   Y    Z  ");
  // BF52.Lcd.setCursor(96, 48); BF52.Lcd.print("mg");
  // BF52.Lcd.setCursor(96, 64); BF52.Lcd.print("o/s");
  // BF52.Lcd.setCursor(0,  96); BF52.Lcd.print("Gyro T ");
}

void loop()
{  
  // If data ready bit set, all data registers have new data
  if(IMU.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)   // check if data ready interrupt
  {
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.aRes=IMU.getAres();
    
    // Now we'll calculate the accleration value into actual g's
    IMU.ax = (float)IMU.accelCount[0]*IMU.aRes - IMU.accelBias[0];  // get actual g value, this depends on scale being set
    IMU.ay = (float)IMU.accelCount[1]*IMU.aRes - IMU.accelBias[1];   
    IMU.az = (float)IMU.accelCount[2]*IMU.aRes - IMU.accelBias[2];  
   
    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.gRes=IMU.getGres();
 
    // Calculate the gyro value into actual degrees per second
    IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes - IMU.gyroBias[0];  // get actual gyro value, this depends on scale being set
    IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes - IMU.gyroBias[1];  
    IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes - IMU.gyroBias[2];   

    IMU.tempCount = IMU.readTempData();  // Read the x/y/z adc values
    IMU.temperature = ((float) IMU.tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
  }  
  // Must be called before updating quaternions!
  IMU.updateTime();

  IMU.delt_t = millis() - IMU.count;
  if(IMU.delt_t > 500) 
  {

    // Print acceleration values in milligs!
    Serial.print("X-acceleration: "); 
    Serial.print(1000*IMU.ax); 
    Serial.print(" mg "); 
    Serial.print("Y-acceleration: "); 
    Serial.print(1000*IMU.ay); 
    Serial.print(" mg "); 
    Serial.print("Z-acceleration: "); 
    Serial.print(1000*IMU.az); 
    Serial.println(" mg"); 

    // Print gyro values in degree/sec
    Serial.print("X-gyro rate: "); 
    Serial.print(IMU.gx, 3); 
    Serial.print(" degrees/sec "); 
    Serial.print("Y-gyro rate: "); 
    Serial.print(IMU.gy, 3); 
    Serial.print(" degrees/sec "); 
    Serial.print("Z-gyro rate: "); 
    Serial.print(IMU.gz, 3); 
    Serial.println(" degrees/sec"); 
    
    // Print temperature in degrees Centigrade      
    Serial.print("Temperature is: ");  
    Serial.print(IMU.temperature, 2);  
    Serial.println(" °C"); // Print T values to tenths of s degree C
    Serial.println("");

    IMU.count = millis();
  }

  BF52.Lcd.drawNumber(0, 40, (int)(1000*IMU.ax), 4, GREEN, BLACK, 2);
  BF52.Lcd.drawNumber(60, 40, (int)(1000*IMU.ay), 4, GREEN, BLACK, 2);
  BF52.Lcd.drawNumber(120, 40, (int)(1000*IMU.az), 4, GREEN, BLACK, 2);
  BF52.Lcd.drawNumber(0, 60, (int)(IMU.gx), 4, GREEN, BLACK, 2);
  BF52.Lcd.drawNumber(60, 60, (int)(IMU.gy), 4, GREEN, BLACK, 2);
  BF52.Lcd.drawNumber(120, 60, (int)(IMU.gz), 4, GREEN, BLACK, 2);
  BF52.Lcd.drawFloat(80, 100, IMU.temperature, 4, YELLOW, BLACK, 2);
  // BF52.Lcd.setCursor(0,  48); BF52.Lcd.print((int)(1000*IMU.ax));
  // BF52.Lcd.setCursor(32, 48); BF52.Lcd.print((int)(1000*IMU.ay));
  // BF52.Lcd.setCursor(64, 48); BF52.Lcd.print((int)(1000*IMU.az));
  // BF52.Lcd.setCursor(0,  64); BF52.Lcd.print((int)(IMU.gx));
  // BF52.Lcd.setCursor(32, 64); BF52.Lcd.print((int)(IMU.gy));
  // BF52.Lcd.setCursor(64, 64); BF52.Lcd.print((int)(IMU.gz));
  // BF52.Lcd.setCursor(50, 96); BF52.Lcd.print(IMU.temperature, 1);
  // BF52.Lcd.print(" C");
}
