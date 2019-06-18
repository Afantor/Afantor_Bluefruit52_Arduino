/* MPU6050 Basic Example with IMU
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

#define blinkPin 19  // Blink LED on Bluefruit52 when updating
boolean blinkOn = false;

void setup()
{
  BF52.begin(true, true, true);
  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = IMU.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  Serial.print("I AM ");
  Serial.print(c, HEX);
  Serial.print(" I Should Be ");
  Serial.println(0x68, HEX);

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU6050 is online...");

    IMU.MPU6050SelfTest(IMU.SelfTest); // Start by performing self test and reporting values
    //    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(IMU.SelfTest[0],1); Serial.println("% of factory value");
    //    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(IMU.SelfTest[1],1); Serial.println("% of factory value");
    //    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(IMU.SelfTest[2],1); Serial.println("% of factory value");
    //    Serial.print("x-axis self test: gyration trim within : "); Serial.print(IMU.SelfTest[3],1); Serial.println("% of factory value");
    //    Serial.print("y-axis self test: gyration trim within : "); Serial.print(IMU.SelfTest[4],1); Serial.println("% of factory value");
    //    Serial.print("z-axis self test: gyration trim within : "); Serial.print(IMU.SelfTest[5],1); Serial.println("% of factory value");

    if (IMU.SelfTest[0] < 1.0f && IMU.SelfTest[1] < 1.0f && IMU.SelfTest[2] < 1.0f && IMU.SelfTest[3] < 1.0f && IMU.SelfTest[4] < 1.0f && IMU.SelfTest[5] < 1.0f) 
    {
      Serial.println("Pass Selftest!");

      IMU.calibrateMPU6050(IMU.gyroBias, IMU.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      Serial.println("MPU6050 bias");
      Serial.println(" x\t  y\t  z  ");
      Serial.print((int)(1000 * IMU.accelBias[0])); Serial.print('\t');
      Serial.print((int)(1000 * IMU.accelBias[1])); Serial.print('\t');
      Serial.print((int)(1000 * IMU.accelBias[2]));
      Serial.println(" mg");

      Serial.print(IMU.gyroBias[0], 1); Serial.print('\t');
      Serial.print(IMU.gyroBias[1], 1); Serial.print('\t');
      Serial.print(IMU.gyroBias[2], 1);
      Serial.println(" o/s");


      IMU.initMPU6050(); 
      Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    }
    else
    {
      Serial.print("Could not connect to MPU6050: 0x");
      Serial.println(c, HEX);
      while (1) ; // Loop forever if communication doesn't happen
    }
  }
}

void loop()
{
  // If data ready bit set, all data registers have new data
  if (IMU.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { // check if data ready interrupt
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.aRes = IMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // get actual g value, this depends on scale being set
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes;
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes;

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.gRes = IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes; // get actual gyro value, this depends on scale being set
    IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;

    IMU.tempCount = IMU.readTempData();  // Read the x/y/z adc values
    IMU.temperature = ((float) IMU.tempCount) / 340.0 + 36.53; // Temperature in degrees Centigrade
  }

  IMU.updateTime();

  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx * PI / 180.0f, IMU.gy * PI / 180.0f, IMU.gz * PI / 180.0f, IMU.deltat);

  // Serial print and/or display at 0.5 s rate independent of data rates
  IMU.delt_t = millis() - IMU.count;
  if (IMU.delt_t > 500) { // update LCD once per half-second independent of read rate
    digitalWrite(blinkPin, blinkOn);
    /*
        Serial.print("ax = "); Serial.print((int)1000*IMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000*IMU.ay);
        Serial.print(" az = "); Serial.print((int)1000*IMU.az); Serial.println(" mg");

        Serial.print("gx = "); Serial.print( IMU.gx, 1);
        Serial.print(" gy = "); Serial.print( IMU.gy, 1);
        Serial.print(" gz = "); Serial.print( IMU.gz, 1); Serial.println(" deg/s");

        Serial.print("q0 = "); Serial.print(q[0]);
        Serial.print(" qx = "); Serial.print(q[1]);
        Serial.print(" qy = "); Serial.print(q[2]);
        Serial.print(" qz = "); Serial.println(q[3]);
    */
    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    IMU.yaw   = atan2(2.0f * (IMU.q[1] * IMU.q[2] + IMU.q[0] * IMU.q[3]), IMU.q[0] * IMU.q[0] + IMU.q[1] * IMU.q[1] - IMU.q[2] * IMU.q[2] - IMU.q[3] * IMU.q[3]);
    IMU.pitch = -asin(2.0f * (IMU.q[1] * IMU.q[3] - IMU.q[0] * IMU.q[2]));
    IMU.roll  = atan2(2.0f * (IMU.q[0] * IMU.q[1] + IMU.q[2] * IMU.q[3]), IMU.q[0] * IMU.q[0] - IMU.q[1] * IMU.q[1] - IMU.q[2] * IMU.q[2] + IMU.q[3] * IMU.q[3]);

    IMU.pitch *= 180.0f / PI;
    IMU.yaw   *= 180.0f / PI;
    IMU.roll  *= 180.0f / PI;

    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(IMU.yaw, 2);
    Serial.print(", ");
    Serial.print(IMU.pitch, 2);
    Serial.print(", ");
    Serial.println(IMU.roll, 2);

    //    Serial.print("average rate = "); Serial.print(1.0f/deltat, 2); Serial.println(" Hz");

    Serial.println(" x\t  y\t  z  ");

    Serial.print((int)(1000 * IMU.ax)); Serial.print('\t');
    Serial.print((int)(1000 * IMU.ay)); Serial.print('\t');
    Serial.print((int)(1000 * IMU.az));
    Serial.println(" mg");

    Serial.print((int)(IMU.gx)); Serial.print('\t');
    Serial.print((int)(IMU.gy)); Serial.print('\t');
    Serial.print((int)(IMU.gz));
    Serial.println(" o/s");

    Serial.print((int)(IMU.yaw)); Serial.print('\t');
    Serial.print((int)(IMU.pitch)); Serial.print('\t');
    Serial.print((int)(IMU.roll));
    Serial.println(" ypr");

    Serial.print("rt: "); Serial.print(1.0f / IMU.deltat, 2); Serial.println(" Hz");

    blinkOn = ~blinkOn;
    IMU.count = millis();
  }
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float deltat)
{
    float q1 = IMU.q[0], q2 = IMU.q[1], q3 = IMU.q[2], q4 = IMU.q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
            
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
          
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
            
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
            
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
            
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * IMU.zeta;
    gbiasy += gerry * deltat * IMU.zeta;
    gbiasz += gerrz * deltat * IMU.zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;
            
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(IMU.beta * hatDot1)) * deltat;
    q2 += (qDot2 -(IMU.beta * hatDot2)) * deltat;
    q3 += (qDot3 -(IMU.beta * hatDot3)) * deltat;
    q4 += (qDot4 -(IMU.beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    IMU.q[0] = q1 * norm;
    IMU.q[1] = q2 * norm;
    IMU.q[2] = q3 * norm;
    IMU.q[3] = q4 * norm;
}
