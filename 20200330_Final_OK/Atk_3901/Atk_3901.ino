#include <Arduino.h>

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Bitcraze_PMW3901.h>

#include <Wire.h>
#include "SparkFun_VL53L1X.h"

SFEVL53L1X distanceSensor;
Bitcraze_PMW3901 flow(10);

void setup(void) {
  
    Serial.begin(115200);
        
    pinMode(5,OUTPUT);    // deltaX
    pinMode(6,OUTPUT);    // deltaY
    pinMode(9,OUTPUT);    // range
    pinMode(LED_BUILTIN, OUTPUT);  // 삭제하면 출력값이 나오지 않음 : 이유?

    if (!flow.begin()) {
      Serial.println("Initialization of the flow sensor failed");
      while(1) { }
    }
    
    Wire.begin();
    
    while (!Serial)

    if (distanceSensor.begin() == 0) { // Begin returns 0 on a good init
        Serial.println("Sensor online!");
    }

    // distanceSensor.setDistanceModeShort();
    distanceSensor.setDistanceModeLong();
    // timing budget default 50,  70 : 정확도 증진, 속도는 저하
    distanceSensor.setTimingBudgetInMs(50);
    // measure periodically. Intermeasurement period must be >/= timing budget.
    distanceSensor.setIntermeasurementPeriod(100);
    distanceSensor.startRanging(); // Start once

}

int16_t deltaX,deltaY;

void loop(void) {

    // Get motion count since last call
    flow.readMotionCount(&deltaX, &deltaY);

    deltaX = constrain(deltaX, -200, 200);
    int outX = map(deltaX, -200, 200, 0, 255);

    deltaY = constrain(-deltaY, -200, 200);
    int outY = map(deltaY, -200, 200, 0, 255);

    analogWrite(5, outX);
    analogWrite(6, outY);

    while (!distanceSensor.checkForDataReady()) {
        delay(1);
    }

    //byte rangeStatus = distanceSensor.getRangeStatus();
    
    //Get the result of the measurement from the sensor
    unsigned int distance = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();

    distance = distance / 10;
    analogWrite(9, distance);

    Serial.print(deltaX);Serial.print("\t");
    Serial.print(deltaY);Serial.print("\t");
    Serial.println(distance);

    delay(1);
}
