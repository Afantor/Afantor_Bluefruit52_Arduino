/*!
  * stepCounter.ino
  *
  * I2C addr:
  *   0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
  *   0x69: set I2C address by parameter
  *
  * Through the example, you can get the sensor data which means step counter.
  * set step counter power mode by setStepPowerMode(stepNormalPowerMode means normal model,stepLowPowerMode means low power model)
  * upload interrupt number by setInt (choose int1 or int2)  
  * data from int1 and int2 read in readStepCounter
  *
  * Copyright   [Afantor](http://www.afantor.cc), 2019
  * Copyright   MIT License
  *
  * version  V1.0
  * date  2019-05-27
  */

#include <bluefruit52.h>

BMI160 bmi160;
const int8_t i2c_addr = 0x69;
bool readStep = false;


int pbIn = A6;

/*the bmi160 have two interrput interfaces*/
int int1 = 1;
int int2 = 2;

void stepChange()
{
  //once the step conter is changed, the value can be read 
  readStep = true;
}

void setup(){
  BF52.begin(true, true, false);
  
  pinMode(pbIn, INPUT_PULLUP);
  //set and init the bmi160 i2c address  
  while (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("i2c init fail");
    delay(1000); 
  }
  
  //set interrput number to int1 or int2
  if (bmi160.setInt(int1) != BMI160_OK){
    Serial.println("set interrput fail");
    while(1);
  }

  //set the bmi160 mode to step counter
  if (bmi160.setStepCounter() != BMI160_OK){
    Serial.println("set step fail");
    while(1);
  }
  
  //set the bmi160 power model,contains:stepNormalPowerMode,stepLowPowerMode
  if (bmi160.setStepPowerMode(bmi160.stepLowPowerMode) != BMI160_OK){
    Serial.println("set setStepPowerMode fail");
    while(1);
  }
  
  attachInterrupt(pbIn, stepChange, FALLING);

  Serial.println(pbIn);
}

void loop(){
  if (readStep){
    uint16_t stepCounter = 0;
    //read step counter from hardware bmi160 
    if (bmi160.readStepCounter(&stepCounter)==BMI160_OK){
      Serial.print("step counter = ");Serial.println(stepCounter);
    }
    readStep = false;
  }
}


