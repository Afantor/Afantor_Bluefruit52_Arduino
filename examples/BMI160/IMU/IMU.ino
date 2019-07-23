/* BMI160 Basic Example Code
 by: Afantor
 date:2019/06/02
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 This sketch uses SDA/SCL on pins 26/25, respectively.
 
 SDA and SCL has external pull-up resistors (to 3.3V).
 
 Hardware setup:
 BOARD ------------------- Bluefruit52 V3.0
 VDD ---------------------- 3.3V
 SDA ----------------------- 25
 SCL ----------------------- 26
 GND ---------------------- GND
 
 */
#include <bluefruit52.h>

#define BMI160_ADDRESS 0x69  // Device address when ADO = 1

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
#define  AFS_2G 0x03
#define  AFS_4G 0x05
#define  AFS_8G 0x08
#define  AFS_16G 0x0C

enum AODR {
  Arate0 = 0,
  Arate1Hz,  // really 25/32
  Arate2Hz,  // really 25/16
  Arate3Hz, // really 25/8
  Arate6_25Hz,  
  Arate12_5Hz,
  Arate25Hz,
  Arate50Hz,
  Arate100Hz,
  Arate200Hz,
  Arate400Hz,
  Arate800Hz,
  Arate1600Hz
};

enum ABW {
  ABW_4X = 0, // 4 times oversampling ~ 10% of ODR
  ABW_2X,     // 2 times oversampling ~ 20% of ODR
  ABW_1X      // 1 times oversampling ~ 40% of ODR
};

enum Gscale {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS
};

enum GODR {
  Grate25Hz = 6,
  Grate50Hz,
  Grate100Hz,
  Grate200Hz,
  Grate400Hz,
  Grate800Hz,
  Grate1600Hz,
  Grate3200Hz
};

enum GBW {
  GBW_4X = 0, // 4 times oversampling ~ 10% of ODR
  GBW_2X,     // 2 times oversampling ~ 20% of ODR
  GBW_1X      // 1 times oversampling ~ 40% of ODR
};


//
// Specify sensor full scale
uint8_t Gscale = GFS_2000DPS, GODR = Grate3200Hz, GBW = GBW_2X;
uint8_t Ascale = AFS_2G, AODR = Arate1600Hz, ABW = ABW_2X;
float aRes, gRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int myLed     = 19;  // LED on the Bluefruit52

// BMI160 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float Quat[4] = {0, 0, 0, 0}; // quaternion data register
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the BMX055 internal chip temperature in degrees Celsius
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll, Yaw, Pitch, Roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
uint8_t param[4];                         // used for param transfer

float ax, ay, az, gx, gy, gz;             // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

bool passThru = true;

void setup()
{
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  Wire.begin();
  delay(50);
  Serial.begin(115200);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("BMI160 6-axis motion sensor...");
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr, irq_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("BMI160,I AM 0x"); 
  Serial.println(dev_id, HEX); 
  Serial.print(" I should be "); Serial.println(0xD1, HEX);
   
  delay(1000); 

  if (dev_id == 0xD1) // WHO_AM_I should always be ACC/GYRO = 0xD1, MAG = 0x48 
  {  
    Serial.println("BMI160 are online...");
  
    delay(1000); 
     
    // get sensor resolutions, only need to do this once
    getAres();
    getGres();
    
    Serial.println(" Calibrate gyro and accel");
    accelgyrofastcalBMI160(accelBias, gyroBias); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.println("accel biases (mg)"); Serial.println(3.9*accelBias[0]); Serial.println(3.9*accelBias[1]); Serial.println(3.9*accelBias[2]);
    Serial.println("gyro biases (dps)"); Serial.println(0.061*gyroBias[0]); Serial.println(0.061*gyroBias[1]); Serial.println(0.061*gyroBias[2]);
    delay(1000);

    initBMI160(); 
    Serial.println("BMI160 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    // Check power status of BMI160
    uint8_t pwr_status = readByte(BMI160_ADDRESS, BMI160_PMU_STATUS);
    uint8_t acc_pwr = (pwr_status & 0x30) >> 4;
    if (acc_pwr == 0x00) Serial.println("Accel Suspend Mode");
    if (acc_pwr == 0x01) Serial.println("Accel Normal Mode");
    if (acc_pwr == 0x02) Serial.println("Accel Low Power Mode");
    uint8_t gyr_pwr = (pwr_status & 0x0C) >> 2;
    if (gyr_pwr == 0x00) Serial.println("Gyro Suspend Mode");
    if (gyr_pwr == 0x01) Serial.println("Gyro Normal Mode");
    if (gyr_pwr == 0x03) Serial.println("Gyro Fast Start Up Mode");
     delay(1000);

    
    BMI160SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.println("result of self test should be at least 2 g!");
    Serial.print("x-axis self test: acceleration delta is : "); Serial.print(SelfTest[0],1); Serial.println(" g");
    Serial.print("y-axis self test: acceleration delta is : "); Serial.print(SelfTest[1],1); Serial.println(" g");
    Serial.print("z-axis self test: acceleration delta is : "); Serial.print(SelfTest[2],1); Serial.println(" g");
    delay(1000);
  }
  else
  {
    Serial.print("Could not connect to BMI160: 0x");
    Serial.println(dev_id, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}


void loop()
{  
  if (readByte(BMI160_ADDRESS, BMI160_STATUS) & 0x40) {  // check if new gyro data
    readAccelGyroData(gyroCount, accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;  
    az = (float)accelCount[2]*aRes;  

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes;   
  }


 // keep track of rates
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Use NED orientaton convention where N is direction of arrow on the board
  // AK8963C has N = -y, E = -x, and D = -z if top of board == North
  // BMI160 has N = x, E - -y, and D - -z for the gyro, and opposite by convention for the accel 
  // This rotation can be modified to allow any convenient orientation convention.
  // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, deltat);
//  if(passThru)MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  -my,  -mx, -mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  if (delt_t > 500)  // update LCD once per half-second independent of read rate
  {
    // if(SerialDebug) {
    //   Serial.print("ax = "); Serial.print((int)1000*ax);  
    //   Serial.print(" ay = "); Serial.print((int)1000*ay); 
    //   Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    //   Serial.print("gx = "); Serial.print( gx, 2); 
    //   Serial.print(" gy = "); Serial.print( gy, 2); 
    //   Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
      
    //   Serial.println("Software quaternions:"); 
    //   Serial.print("q0 = "); Serial.print(q[0]);
    //   Serial.print(" qx = "); Serial.print(q[1]); 
    //   Serial.print(" qy = "); Serial.print(q[2]); 
    //   Serial.print(" qz = "); Serial.println(q[3]); 
    // }               

   //  tempCount = readTempData();  // Read the gyro adc values
   //  temperature = ((float) tempCount) / 512.0 + 23.0; // Gyro chip temperature in degrees Centigrade
   // // Print temperature in degrees Centigrade      
   //  Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
   
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    //Software AHRS:
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;
    //Hardware AHRS:
    // Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);   
    // Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
    // Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
    // Pitch *= 180.0f / PI;
    // Yaw   *= 180.0f / PI; 
    // Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    // if(Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
    // Roll  *= 180.0f / PI;
    
    // Or define output variable according to the Android system, where heading (0 to 260) is defined by the angle between the y-axis 
    // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
    // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
    // points toward the right of the device.
    //
    
    if(SerialDebug) {
      Serial.print("Software Yaw, Pitch, Roll: ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.println(roll, 2);
      
      // Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }
 
    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
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
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
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
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;
            
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

float uint32_reg_to_float (uint8_t *buf)
{
  union {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 =     (((uint32_t)buf[0]) +
               (((uint32_t)buf[1]) <<  8) +
               (((uint32_t)buf[2]) << 16) +
               (((uint32_t)buf[3]) << 24));
  return u.f;
}

void float_to_bytes (float param_val, uint8_t *buf) {
  union {
    float f;
    uint8_t comp[sizeof(float)];
  } u;
  u.f = param_val;
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = u.comp[i];
  }
  //Convert to LITTLE ENDIAN
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = buf[(sizeof(float)-1) - i];
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_125DPS:
          gRes = 125.0/32768.0;
          break;
     case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (05), 8 Gs (08), and 16 Gs  (0C). 
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelGyroData(int16_t * dest1, int16_t * dest2)
{
  uint8_t rawData[12];  // x/y/z accel register data stored here
  readBytes(BMI160_ADDRESS, BMI160_GYRO_DATA, 12, &rawData[0]);  // Read the twelve raw data registers into data array
  dest1[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn gyro MSB and LSB into a signed 16-bit value
  dest1[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  dest1[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 

  dest2[0] = ((int16_t)rawData[7] << 8) | rawData[6] ;  // Turn accel MSB and LSB into a signed 16-bit value
  dest2[1] = ((int16_t)rawData[9] << 8) | rawData[8] ;  
  dest2[2] = ((int16_t)rawData[11] << 8) | rawData[10] ; 
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(BMI160_ADDRESS, BMI160_TEMPERATURE, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a 16-bit value
}

void initBMI160()
{  
 // configure accel and gyro
  writeByte(BMI160_ADDRESS, BMI160_CMD, 0x11); // Set accel in normal mode operation
  delay(50); // Wait for accel to reset 
  writeByte(BMI160_ADDRESS, BMI160_CMD, 0x15); // Set gyro in normal mode operation
  delay(100); // Wait for gyro to reset 
 // Define accel full scale and sample rate
  writeByte(BMI160_ADDRESS, BMI160_ACC_RANGE, Ascale);
  writeByte(BMI160_ADDRESS, BMI160_ACC_CONF, ABW << 4 | AODR);
 // Define gyro full scale and sample rate
  writeByte(BMI160_ADDRESS, BMI160_GYR_RANGE, Gscale);
  writeByte(BMI160_ADDRESS, BMI160_GYR_CONF, GBW << 4 | GODR);
}


void accelgyrofastcalBMI160(float * dest1, float * dest2)
{
  uint8_t rawData[7] = {0, 0, 0, 0, 0, 0, 0};
  
  writeByte(BMI160_ADDRESS, BMI160_FOC_CONF, 0x4 | 0x30 | 0x0C | 0x01); // Enable gyro cal and accel cal with 0, 0, 1 as reference
  delay(20);
  writeByte(BMI160_ADDRESS, BMI160_CMD, 0x03); // start fast calibration
  delay(50);
  if(readByte(BMI160_ADDRESS, BMI160_ERR_REG) & 0x40) {  // check if dropped command
    Serial.println("Dropped fast offset compensation command!");
    return;
  }
 
  while(!(readByte(BMI160_ADDRESS, BMI160_STATUS) & 0x08)); // wait for fast compensation data ready bit
  if((readByte(BMI160_ADDRESS, BMI160_STATUS) & 0x08)) {
   
    readBytes(BMI160_ADDRESS, BMI160_OFFSET, 7, &rawData[0]);  // Read the seven raw data registers into data array
    dest1[0] = (float)(((int16_t)(rawData[0] << 8 ) | 0x00) >> 8);  // Turn accel offset into a signed 8-bit value
    dest1[1] = (float)(((int16_t)(rawData[1] << 8 ) | 0x00) >> 8);  
    dest1[2] = (float)(((int16_t)(rawData[2] << 8 ) | 0x00) >> 8); 

    dest2[0] = (float)((((int16_t)(rawData[7] & 0x03) << 8) | rawData[3]) >> 6);  // Turn gyro offset MSB and LSB into a signed 10-bit value
    dest2[1] = (float)((((int16_t)(rawData[7] & 0x0C) << 8) | rawData[4]) >> 6);  
    dest2[2] = (float)((((int16_t)(rawData[7] & 0x30) << 8) | rawData[5]) >> 6); 

    writeByte(BMI160_ADDRESS, BMI160_OFFSET_CONF, 0x40);  // enable use of offset registers in data output
  }
  else {
    Serial.println("couldn't get offsets!");
  }
}


void BMI160SelfTest(float * destination)
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint16_t selfTestp[3], selfTestm[3];
     
// Enable Gyro Self Test
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x10); // Enable Gyro Self Test
  delay(100);
  uint8_t result = readByte(BMI160_ADDRESS, BMI160_STATUS);
  if(result & 0x02) {
    Serial.println("Gyro Self test passed!");
  }
  else {
     Serial.println("Gyro Self test failed!");  
  }
   // disable self test
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x00 ); // disable Self Test  
  delay(100);
  
// Configure for Accel Self test
   writeByte(BMI160_ADDRESS, BMI160_ACC_RANGE, AFS_8G); // set range to 8 g
   writeByte(BMI160_ADDRESS, BMI160_ACC_CONF, 0x2C ); // Configure accel for Self Test

// Enable Accel Self Test-positive deflection
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x08 | 0x04 | 0x01 ); // Enable accel Self Test positive
  delay(100);
  readBytes(BMI160_ADDRESS, BMI160_ACCEL_DATA, 6, &rawData[0]);  // Read the six raw data registers into data array
  selfTestp[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn accel MSB and LSB into a signed 16-bit value
  selfTestp[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  selfTestp[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 

  // disable self test
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x00 ); // disable Self Test  
  delay(100);

// Enable Accel Self Test-negative deflection
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x08 | 0x01 ); // Enable accel Self Test negative
  delay(100);
  readBytes(BMI160_ADDRESS, BMI160_ACCEL_DATA, 6, &rawData[0]);  // Read the six raw data registers into data array
  selfTestm[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn accel MSB and LSB into a signed 16-bit value
  selfTestm[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  selfTestm[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 

//  Serial.println(selfTestp[0]);Serial.println(selfTestp[1]);Serial.println(selfTestp[2]);
 // Serial.println(selfTestm[0]);Serial.println(selfTestm[1]);Serial.println(selfTestm[2]);
   
  destination[0] = (float)(selfTestp[0] - selfTestm[0])*8./32768.; //construct differences between positve and negative defection, should be ~2 g
  destination[1] = (float)(selfTestp[1] - selfTestm[1])*8./32768.;
  destination[2] = (float)(selfTestp[2] - selfTestm[2])*8./32768.;

 // disable self test
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x00 ); // disable Self Test  
  delay(100);

  writeByte(BMI160_ADDRESS, BMI160_ACC_RANGE, Ascale); // set range to original value
  writeByte(BMI160_ADDRESS, BMI160_ACC_CONF, ABW << 4 | AODR); // return accel to original configuration
// End self tests
}

// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


// I2C read/write functions for the MPU6500 and AK8963 sensors
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (uint8_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
	while (Wire.available()) {
    dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

