/*
  MultiWiiCopter by Alexandre Dubus
  www.multiwii.com
  July  2012     V2.1  ( Feb 2020 modified for opticalflow )
*/
/*
 기본 작동 안정적 - 합격
 
 New Test - Pro Mini 에 ATK PMW3901 연결
 aux2_tmp 는 3 ~ 4,  angleTrim 은  12, -2 OK
 Hovering Throttle : 500mAh 배터리 장착 시 1414 -> 1450 배터리 전압강하에 따라 변동
 현재 통신은 115200 bps 임 ( config line 30 )

 주의!
 ATK-PMW3901 버전임 -> OpticaFlow 의 line 95, 101 바꿨음 optflow_pos[0], optflow_pos[1]
 
*/

#include <avr/io.h>
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include <avr/pgmspace.h>
#define  VERSION  211

// *********** RC alias *****************/
enum rc {           // 열거형 enum, 초기값 안 주면 자동으로 0부터 시작
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4        // 7
};

enum pid {
  PIDROLL,    // 0
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL,     // 9
  PIDITEMS    // 10
};

enum box {
#if ACC
  BOXANGLE,   // 0
  BOXHORIZON,
#endif
#if BARO
  BOXBARO,
#endif
#if MAG
  BOXMAG,
#endif

  BOXARM,     // 4

#if GPS || OPTFLOW
  BOXOPTHOLD,
#endif
#if MAG
  BOXHEADFREE,
#endif

#if MAG
  BOXHEADADJ, // 8 acquire heading for HEADFREE mode
#endif
  CHECKBOXITEMS  // 8
};

const char boxnames[] PROGMEM =     // names for dynamic generation of config GUI
#if ACC
  "ANGLE;"
  "HORIZON;"
#endif
#if BARO
  "BARO;"
#endif
#if MAG
  "MAG;"
#endif

  "ARM;"

#if GPS || OPTFLOW
  "OPTHOLD;"
#endif
#if MAG
  "HEADFREE;"
#endif

#if MAG
  "HEADADJ;"
#endif
  ;

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"     //5
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
  ;

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;
static uint16_t acc_1G;             // this is the 1G measured acceleration
static int16_t  acc_25deg;
static int16_t  headFreeModeHold;
static int16_t  gyroADC[3], accADC[3], accSmooth[3], magADC[3];
static int16_t  heading, magHold;
static uint8_t  vbat;                   // battery voltage in 0.1V steps
static uint8_t  rcOptions[CHECKBOXITEMS];

static uint8_t battery_compensation = 40;

static int32_t  BaroAlt;
static int32_t  BaroHome = 0;  // for test
static int32_t  EstAlt;             // in cm
static int16_t  BaroPID = 0;
static int32_t  AltHold;
static int16_t  errorAltitudeI = 0;
static int16_t  sonarAlt = 0; // distance, cm (0..SONAR_MAX_DISTANCE)
    
static int16_t  debug[4];

struct flags_struct {
  uint8_t OK_TO_ARM : 1 ;
  uint8_t ARMED : 1 ;
  uint8_t ACC_CALIBRATED : 1 ;
  uint8_t ANGLE_MODE : 1 ;
  uint8_t HORIZON_MODE : 1 ;
  uint8_t MAG_MODE : 1 ;
  uint8_t BARO_MODE : 1 ;
  uint8_t OPT_HOLD_MODE : 1 ;   // for test - 추가
  uint8_t HEADFREE_MODE : 1 ;
  uint8_t PASSTHRU_MODE : 1 ;
  uint8_t SMALL_ANGLES_25 : 1 ;
  uint8_t CALIBRATE_MAG : 1 ;
} f;

static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;

static uint16_t intPowerMeterSum, intPowerTrigger1;

static uint8_t  buzzerState = 0;

// ******** rc functions ********
#define MINCHECK 1050     //1115     //1100
#define MAXCHECK 1900     //1895     //1900

static int16_t rcData[RC_CHANS];   // interval [1000;2000]
static int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
static int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

// ******** gyro+acc IMU ********
static int16_t gyroData[3] = {0, 0, 0};
static int16_t gyroZero[3] = {0, 0, 0};
static int16_t angle[2]    = {0, 0};      // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800


// ******** motor and servo functions ********
static int16_t axisPID[3];
static int16_t motor[NUMBER_MOTOR];

// ******** EEPROM Layout definition ********

static uint8_t dynP8[3], dynD8[3];
static struct {
  uint8_t checkNewConf;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t accZero[3];
  int16_t magZero[3];
  int16_t angleTrim[2];
  uint16_t activate[CHECKBOXITEMS];
  uint8_t powerTrigger1;    // not used only for EEPROM save
} conf;

// ******** GPS common variables - not used only for EEPROM save 현재 사용 X

// default POSHOLD control gains         
#define POSHOLD_P              .11
#define POSHOLD_I              0.0

#define POSHOLD_RATE_P         2.0
#define POSHOLD_RATE_I         0.08      // Wind control
#define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1

// default Navigation PID gains
#define NAV_P                  1.4
#define NAV_I                  0.20      // Wind control
#define NAV_D                  0.08      //

// ******** OPTFlow 관련 변수 ********

static int8_t optflowMode = 0;              // Optical Flow mode flag
static int16_t optflow_angle[2] = { 0, 0 };    // Angles of correction
static int16_t optflow_pos[2]  = { 0, 0 };  // displacment (in mm*10 on height 1m)
static uint16_t  aux2_tmp;     // sonar 및 optflow 최적 PID 값을 찾기위해 사용
static uint16_t  aux3_tmp;
static int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

// ******** Function ( blinkLED, annexCode ) ******

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat) {
  uint8_t i, r;
  for (r = 0; r < repeat; r++) {
    for (i = 0; i < num; i++) {
      LEDPIN_TOGGLE; // switch LEDPIN state
      POWERPIN_ON;
      delay(wait);
      POWERPIN_OFF;
    }
    delay(60);
  }
}

//------------------------------ annexcode() 시작 -> rcCommand 계산

void annexCode() {    // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp, tmp2;
  uint8_t axis, prop1, prop2;
  static uint8_t  buzzerFreq;         // delay between buzzer ring
  static uint32_t buzzerTime;

  #define BREAKPOINT 1500
  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if   (rcData[THROTTLE] < BREAKPOINT) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE] < 2000) {
      prop2 = 100 - (uint16_t)conf.dynThrPID * (rcData[THROTTLE] - BREAKPOINT) / (2000 - BREAKPOINT);
    } else {
      prop2 = 100 - conf.dynThrPID;
    }
  }

  for (axis = 0; axis < 3; axis++) {
    tmp = min(abs(rcData[axis] - MIDRC), 500);
    #if defined(DEADBAND)
        if (tmp > DEADBAND) {
          tmp -= DEADBAND;
        }
        else {
          tmp = 0;
        }
    #endif
    if (axis != 2) {              //ROLL & PITCH
      tmp2 = tmp / 100;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
      prop1 = 100 - (uint16_t)conf.rollPitchRate * tmp / 500;
      prop1 = (uint16_t)prop1 * prop2 / 100;
    } else {                      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100 - (uint16_t)conf.yawRate * tmp / 500;
    }
    dynP8[axis] = (uint16_t)conf.P8[axis] * prop1 / 100;
    dynD8[axis] = (uint16_t)conf.D8[axis] * prop1 / 100;
    if (rcData[axis] < MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE], MINCHECK, 2000);
  tmp = (uint32_t)(tmp - MINCHECK) * 1000 / (2000 - MINCHECK);          // [MINCHECK;2000] -> [0;1000]
  tmp2 = tmp / 100;
  
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100; 

  //IF aux3_tmp > 1500 THEN rcCommand[THROTTLE] = Hovering Throttle, ELSE 기존 코드  
  
  if (f.HEADFREE_MODE) {                                                //to optimize
    float radDiff = (heading - headFreeModeHold) * 0.0174533f;          // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  #if defined(VBAT)
    static uint8_t vbatTimer = 0;
    static uint8_t ind = 0;
    uint16_t vbatRaw = 0;
    static uint16_t vbatRawArray[8];
    if (! (++vbatTimer % VBATFREQ)) {
      vbatRawArray[(ind++) % 8] = analogRead(V_BATPIN);
      for (uint8_t i = 0; i < 8; i++) vbatRaw += vbatRawArray[i];
      vbat = vbatRaw / (VBATSCALE / 2);                // result is Vbatt in 0.1V steps
    }
    if ( ( (vbat > VBATLEVEL1_3S)
  
         )  || (NO_VBAT > vbat)                              ) 
    { // VBAT ok AND powermeter ok, buzzer off
      buzzerFreq = 0; buzzerState = 0; POWERPIN_OFF;
  
    } else if (vbat > VBATLEVEL2_3S) buzzerFreq = 1;
    else if (vbat > VBATLEVEL3_3S)   buzzerFreq = 2;
    else                           buzzerFreq = 4;
    if (buzzerFreq) {
      if (buzzerState && (currentTime > buzzerTime + 250000) ) {
        buzzerState = 0;
        POWERPIN_OFF;
        buzzerTime = currentTime;
      } else if ( !buzzerState && (currentTime > (buzzerTime + (2000000 >> buzzerFreq))) ) {
        buzzerState = 1;
        POWERPIN_ON;
        buzzerTime = currentTime;
      }
    }
  #endif

  if ( (calibratingA > 0 && ACC ) || (calibratingG > 0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {
      LEDPIN_OFF;
    }
    if (f.ARMED) {
      LEDPIN_ON;
    }
  }

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;     //500000 ( org )
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }
  serialCom();
}

//------------------------------------- annexcode() 끝

// ******** Initial Setup *********

void setup() {

  SerialOpen(0, SERIAL0_COM_SPEED);

  CHECK_PINMODE;   // 고도 모니터링을 위해 새로 추가한 부분
  
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  readEEPROM();
  checkFirstTime();
  configureReceiver();
  initSensors();

  #if defined(SONAR)
     initSonar();
  #endif

  #if defined(OPTFLOW)
    initOptflow();   
  #endif
  
  previousTime = micros();
  calibratingG = 400;

  ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution
  f.SMALL_ANGLES_25 = 1; // important for gyro only conf
  
}

//--------------------------------- 메인 루프 시작

// ******** Main Loop *********

void loop () {
  
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  uint8_t axis, i;
  int16_t error, errorAngle;
  int16_t delta, deltaSum;
  int16_t PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
  static int16_t lastGyro[3] = {0, 0, 0};
  static int16_t delta1[3], delta2[3];
  static int16_t errorGyroI[3] = {0, 0, 0};
  static int16_t errorAngleI[2] = {0, 0};
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;

  #define RC_FREQ 50

//------------------------------------------------ rc loop

  if (currentTime > rcTime ) {      // 50Hz
    rcTime = currentTime + 20000;
    computeRC();
  
    // end of failsave routine - next change is made with RcOptions setting
    // Throttle 값이 최저일 때
    if (rcData[THROTTLE] < MINCHECK) {
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      rcDelayCommand++;

      //기동전 YAW 스틱값과 PITCH값이 1100보다 작고 0.4초 지속이면 ->  Gyro 캘리브레이션
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && !f.ARMED) {
        if (rcDelayCommand == 20) {
          calibratingG = 400;
        }
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
        if (rcDelayCommand == 20) {
          previousTime = micros();
        }
      }
      //기동전, YAW, PITCH값 1100 이하, BOXARM 이면 -> 시동
      else if (conf.activate[BOXARM] > 0) {
        if ( rcOptions[BOXARM] && f.OK_TO_ARM ) {
          f.ARMED = 1;                      // 시동
          headFreeModeHold = heading;
        } else if (f.ARMED) f.ARMED = 0;
        rcDelayCommand = 0;
        
  #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
      } else if ( (rcData[YAW] < MINCHECK )  && f.ARMED) {
        if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[YAW] > MAXCHECK ) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
        if (rcDelayCommand == 20) {
          f.ARMED = 1;
          headFreeModeHold = heading;
        }
  #endif
  
      } else
        rcDelayCommand = 0;
        
    // Throttle 최대, 시동 전 -> ACC, COMPASS 캘리브레이션, Trim    
    } else if (rcData[THROTTLE] > MAXCHECK && !f.ARMED) {
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {             // Calibrate ACC (throttle=max, yaw=left, pitch=min)
        if (rcDelayCommand == 20) calibratingA = 400;
        rcDelayCommand++;
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MINCHECK) {      // Calibrate Compass (throttle=max, yaw=right, pitch=min)
        if (rcDelayCommand == 20) f.CALIBRATE_MAG = 1;                      // MAG calibration request
        rcDelayCommand++;
      } else if (rcData[PITCH] > MAXCHECK) {          // Trim Acc Forwards (throttle=max, pitch=max)
        conf.angleTrim[PITCH] += 2; writeParams(1);

      } else if (rcData[PITCH] < MINCHECK) {          // Trim Acc Backwards (throttle=max, pitch=min)
        conf.angleTrim[PITCH] -= 2; writeParams(1);
      } else if (rcData[ROLL] > MAXCHECK) {           // Trim Acc Right (throttle=max, roll=right)
        conf.angleTrim[ROLL] += 2; writeParams(1);
      } else if (rcData[ROLL] < MINCHECK) {           // Trim Acc Left (throttle=max, roll=left)
        conf.angleTrim[ROLL] -= 2; writeParams(1);
      } else {
        rcDelayCommand = 0;
      }
    }

// AUX1 의 상태를 체크하여 BARO, MAG 등 상태를 결정한다. /////////////////

    uint16_t auxState = 0;
    for (i = 0; i < 4; i++)
      auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
    for (i = 0; i < CHECKBOXITEMS; i++)
      rcOptions[i] = (auxState & conf.activate[i]) > 0;

////////////////////////////////////////////////////////////////////

// AUX2, AUX3 의 상태를 체크하여 Optflow Scale, S/W ON Take off 결정 ///

//  aux2_tmp = map(rcData[AUX2], 1110, 1907, 1, 120);     // for Lidar test
  aux2_tmp = map(rcData[AUX2], 1110, 1907, 1, 6);       // for optflow test
//  aux3_tmp = rcData[AUX2];    // rcData[AUX3]; 수신기 6CH 까지만 지원해서 변경함

// Switch ON Arm / Takeoff 기능 테스트 부분 ///////////////////////////
// SOT (Switch ON Takeoff) 은 Throttle Hovering Position 을 찾아내서 송신기 Throttle 스틱을 위치시켜야 함
// 송신기 사용하지 않고 SOT 하려면 rcCommand[THROTTLE] 값에 송신기 값을 넣는 대신에 Hovering Throttle 값을
// 대입하는 코드를 추가해야 함 ( Line 285 ~ 288 변경 -> IF aux3_tmp > 1500 THEN rcCommand[THROTTLE] = Hovering Throttle, ELSE 기존 코드 )
// Takeoff Throttle Position -> ALT_HOLD_THROTTLE_NEUTRAL_ZONE 50 을 초과하면 Baro Mode 가 해제 되므로 사용에 주의! ( Line 654 )

// 현재 코드는 S/W ON 시 1m 까지 이륙하면 자동 BARO, MAG, OPTHOLD 로 Position Hold 되도록 함
// -> 테스트 결과 : 처음 고도 1m 도착하면 Drop, Throttle up 필요, 작동은 OK, but 다소 불안정
// 우선 수동으로 안정상태 돌입 후 S/W ON 해야 문제 없음, 이 경우 이 기능의 필요성 문제 대두
// SOT 기능, Swith ON Hovering 기능 모두 Drop
/*
  if (aux3_tmp > 1500) {
    f.ARMED = 1;
    if (sonarAlt > 100 && sonarAlt < 120) {
        rcOptions[BOXBARO] = 1;
        rcOptions[BOXMAG] = 1;
        rcOptions[BOXOPTHOLD] = 1;    // OPTFLOW Mode
    }
    } else {
    if (f.ARMED == 1 && sonarAlt < 10) {   // Switch OFF Disarm
       f.ARMED = 0;
       rcOptions[BOXBARO] = 0;
       rcOptions[BOXMAG] = 0;
       rcOptions[BOXOPTHOLD] = 0;
    }
 }
*/
  #if ACC
    if ( rcOptions[BOXANGLE] ) {
      // bumpless transfer to Level mode
      if (!f.ANGLE_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.ANGLE_MODE = 1;
      }
    } else {
      f.ANGLE_MODE = 0;
    }
    if ( rcOptions[BOXHORIZON] ) {
      if (!f.HORIZON_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.HORIZON_MODE = 1;
      }
    } else {
      f.HORIZON_MODE = 0;
    }
  #endif

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;

  #if BARO
    if (rcOptions[BOXBARO]) {         // BARO Switch ON
      if (!f.BARO_MODE) {
        f.BARO_MODE = 1;
        AltHold = EstAlt;
        initialThrottleHold = rcCommand[THROTTLE];
        errorAltitudeI = 0;
        BaroPID = 0;
      }
    } else {
      f.BARO_MODE = 0;
    }
  #endif
  #if MAG
    if (rcOptions[BOXMAG]) {
      if (!f.MAG_MODE) {
        f.MAG_MODE = 1;
        magHold = heading;
      }
    } else {
      f.MAG_MODE = 0;
    }
    if (rcOptions[BOXHEADFREE]) {
      if (!f.HEADFREE_MODE) {
        f.HEADFREE_MODE = 1;
      }
    } else {
      f.HEADFREE_MODE = 0;
    }
    if (rcOptions[BOXHEADADJ]) {
      headFreeModeHold = heading; // acquire new heading
    }
  #endif

  // alexmos: using GPSHold checkbox for opticalFlow mode, because no special box in GUI
  #if defined(OPTFLOW)
     if (rcOptions[BOXOPTHOLD]) {
        optflowMode = 1;
        f.OPT_HOLD_MODE = 1;
     } else {
        optflowMode = 0;
        f.OPT_HOLD_MODE = 0;
        nav_rated[i] = 0;       // for test
     }
  #endif

//-------------------------------------------- not in rc loop

  } else { // not in rc loop
    static uint8_t taskOrder = 0; // never call all functions in the same loop, to avoid high delay spikes
    switch (taskOrder) {
      case 0:
        taskOrder++;
        #if MAG
            Mag_getADC();
            break;
        #endif
      case 1:
        taskOrder++;
        #if BARO
            Baro_update();
            break;
        #endif
      case 2:
        taskOrder++;
        #if BARO
            getEstimatedAltitude();
            break;
        #endif
      case 3:
        taskOrder++;
        #if SONAR
            sonarUpdate();
            break;
        #endif
      case 4:
        taskOrder++; 
        #ifdef OPTFLOW
          Optflow_update();
          break;
        #endif
      default:
        taskOrder=0;
        break;
    }
  }

//--------------------------------------------

  computeIMU();    // IMU 에 있으며, annexcode() 호출

//--------------------------------------------------

  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  #if MAG
  if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
    int16_t dif = heading - magHold;
    if (dif <= - 180) dif += 360;
    if (dif >= + 180) dif -= 360;
    if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif * conf.P8[PIDMAG] / 30;  // 18 deg
  } else magHold = heading;
  #endif

  #if BARO
  if (f.BARO_MODE) {
    // THROTTLE 많이 움직이면 ( 50(20) 이상 차이 ) Baro mode 해제
    if (abs(rcCommand[THROTTLE] - initialThrottleHold) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
      f.BARO_MODE = 0;              // so that a new althold reference is defined
    } else {
      // THROTTLE값 = 고도 설정값 + BaroPID ( BARO PID 계산 결과값 )
      rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
    }
  }
  #endif
  
//  for FB9  -6, 4, 4, 8      for FB13  8, -2  ( 6, 0 ) -> 500mA BAT 10, 0

  if (sonarAlt < 50) {        // 이륙 후 기체 흐름 현상이 발생하므로 50cm 이상은 보정값으로 대체
    conf.angleTrim[0] = 0;    //-6;   // 이륙 전 초기 Roll 값
    conf.angleTrim[1] = 0;    //4;    // 이륙 전 초기 Pitch 값
  } else {

    conf.angleTrim[0] = 8;   //4;    // 0  Hovering Test 후 확정
    conf.angleTrim[1] = -2;   //8;    // 8  Hovering Test 후 확정   

  }
  
  debug[0] = aux2_tmp;        // BaroHome;        // conf.P8[PIDALT];  IMU line 235
  debug[1] = sonarAlt;        // BaroPID;

  debug[2] = optflow_angle[0];    // axisPID[0];
  debug[3] = optflow_angle[1];    // axisPID[1];
  
//  debug[2] = conf.angleTrim[0];
//  debug[3] = conf.angleTrim[1];

  // **** PITCH & ROLL & YAW PID ****

//  int16_t prop;       // HORIZON 모드 사용 시에는 주석 해제
//  prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL]));    // range [0;500]

//--------- Roll, Pitch, Yaw에 대해 annexCode 에서 계산된 rcCommand 와  optflow_angle 값으로 axisPID 값 계산

  for (axis = 0; axis < 3; axis++) {
    if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2 ) {      // MODE relying on ACC

/*
      // OK , aux2_tmp 2 필요
      #ifdef OPTFLOW
         if (sonarAlt > 50) { 
            rcCommand[axis] = rcCommand[axis] + constrain(optflow_angle[axis], -200, +200);   // 100
         } else {
            // 근거리 overshoot 방지 위해 거리 비례 constrain
            rcCommand[axis] = rcCommand[axis] + constrain(optflow_angle[axis], -(sonarAlt/2), +(sonarAlt/2));
         }
      #endif
      // 50 degrees max inclination   
      errorAngle = constrain(2 * rcCommand[axis], -500, +500) - angle[axis] + conf.angleTrim[axis];
*/

      // GPS 방식 -> OK , aux2_tmp 3 필요
      #if OPTFLOW  
         if (sonarAlt > 50) { 
            errorAngle = constrain(2 * rcCommand[axis] + constrain(optflow_angle[axis], -200, +200), -500, +500) - angle[axis] + conf.angleTrim[axis];
         } else {
            // 근거리 overshoot 방지 위해 거리 비례 constrain
            errorAngle = constrain(2 * rcCommand[axis] + constrain(optflow_angle[axis], -(sonarAlt/2), +(sonarAlt/2)), -500, +500) - angle[axis] + conf.angleTrim[axis];
         }
      #else
         errorAngle = constrain(2 * rcCommand[axis], -500, +500) - angle[axis] + conf.angleTrim[axis];
      #endif


      PTermACC = (int32_t)errorAngle * conf.P8[PIDLEVEL] / 100 ;
      PTermACC = constrain(PTermACC, -conf.D8[PIDLEVEL] * 5, +conf.D8[PIDLEVEL] * 5);

      errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000);
      ITermACC = ((int32_t)errorAngleI[axis] * conf.I8[PIDLEVEL]) >> 12;
      
    }

    if (!f.ANGLE_MODE || axis == 2 ) {          // MODE relying on GYRO or YAW axis
      if (abs(rcCommand[axis]) < 350) error = rcCommand[axis] * 10 * 8 / conf.P8[axis] ;
      else error = (int32_t)rcCommand[axis] * 10 * 8 / conf.P8[axis] ;
      error -= gyroData[axis];

      PTermGYRO = rcCommand[axis];

      errorGyroI[axis]  = constrain(errorGyroI[axis] + error, -16000, +16000);
      if (abs(gyroData[axis]) > 640) errorGyroI[axis] = 0;
      ITermGYRO = (errorGyroI[axis] / 125 * conf.I8[axis]) >> 6;
    }
   
//    if ( f.HORIZON_MODE && axis < 2) {    // HORIZON 모드 + ROLL과 PITCH에 대해 
//      PTerm = ((int32_t)PTermACC * (500 - prop) + (int32_t)PTermGYRO * prop) / 500;
//      ITerm = ((int32_t)ITermACC * (500 - prop) + (int32_t)ITermGYRO * prop) / 500;
//    } else {      // HORIZON 모드가 아니고 YAW 에 대해 
      if ( f.ANGLE_MODE && axis < 2) {      // ANGLE 모드 + ROLL과 PITCH에 대해 
        PTerm = PTermACC;
        ITerm = ITermACC;
      } else {    // YAW 에 대해
        PTerm = PTermGYRO;
        ITerm = ITermGYRO;
      }
//    }
    
    if (abs(gyroData[axis]) < 160) PTerm -= gyroData[axis] * dynP8[axis] / 10 / 8; 
    else PTerm -= (int32_t)gyroData[axis] * dynP8[axis] / 10 / 8; 

    delta          = gyroData[axis] - lastGyro[axis];
    lastGyro[axis] = gyroData[axis];
    deltaSum       = delta1[axis] + delta2[axis] + delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;

    if (abs(deltaSum) < 640) DTerm = (deltaSum * dynD8[axis]) >> 5;
    else DTerm = ((int32_t)deltaSum * dynD8[axis]) >> 5;
    
    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

//--------------------------------------- PID 계산 끝

  mixTable();
  writeMotors();

}
