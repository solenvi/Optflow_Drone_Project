
void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0, 0, 0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static uint32_t timeInterleave = 0;

  #if ACC
    ACC_getADC();
    getEstimatedAttitude();
  #endif
  #if GYRO
    Gyro_getADC();
  #endif
  for (axis = 0; axis < 3; axis++)
    gyroADCp[axis] =  gyroADC[axis];
  timeInterleave = micros();
  
  annexCode();      // main loop 에 있음

  #if BARO
      getEstimatedAltitude();
  #endif
  
  if ((micros() - timeInterleave) > 650) {
    annex650_overrun_count++;
  } else {
    while ((micros() - timeInterleave) < 650) ; //empirical, interleaving delay between 2 consecutive reads
  }
  #if GYRO
    Gyro_getADC();
  #endif
  for (axis = 0; axis < 3; axis++) {
    gyroADCinter[axis] =  gyroADC[axis] + gyroADCp[axis];
    // empirical, we take a weighted value of the current and the previous values
    gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis]) / 3;
    gyroADCprevious[axis] = gyroADCinter[axis] / 2;
    if (!ACC) accADC[axis] = 0;
  }
}

//******  advanced users settings *******************
#ifndef ACC_LPF_FACTOR
#define ACC_LPF_FACTOR 200 // Hovering more stable (default : 100)
#endif

#ifndef MG_LPF_FACTOR
//#define MG_LPF_FACTOR 4
#endif

#ifndef GYR_CMPF_FACTOR
#define GYR_CMPF_FACTOR 600.0f     // default : 400.0f
#endif

#ifndef GYR_CMPFM_FACTOR
#define GYR_CMPFM_FACTOR 200.0f
#endif
//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if GYRO
#define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result
#else
#define GYRO_SCALE (1.0f/200e6f)
#endif

// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;

int16_t _atan2(float y, float x) {
#define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100));
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ) {
    if (zi > 10)
      z = z / (1.0f + 0.28f * z * z);
    if (fp_is_neg(x)) {
      if (y_neg) z -= PI;
      else z += PI;
    }
  } else {
    z = (PI / 2.0f) - z / (z * z + 0.28f);
    if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 10);
  return z;
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}

#define ACC_LPF_FOR_VELOCITY 10
static float accLPFVel[3];
static t_fp_vector EstG;

void getEstimatedAttitude() {
  uint8_t axis;
  int32_t accMag = 0;
  #if MAG
  static t_fp_vector EstM;
  #endif
  #if defined(MG_LPF_FACTOR)
  static int16_t mgSmooth[3];
  #endif
  #if defined(ACC_LPF_FACTOR)
  static float accLPF[3];
  #endif
  static uint16_t previousT;
  uint16_t currentT = micros();
  float scale, deltaGyroAngle[3];

  scale = (currentT - previousT) * GYRO_SCALE;
  previousT = currentT;

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * scale;
  #if defined(ACC_LPF_FACTOR)
    accLPF[axis]    = accLPF[axis]    * (1.0f - (1.0f / ACC_LPF_FACTOR))       + accADC[axis] * (1.0f / ACC_LPF_FACTOR);
    accLPFVel[axis] = accLPFVel[axis] * (1.0f - (1.0f / ACC_LPF_FOR_VELOCITY)) + accADC[axis] * (1.0f / ACC_LPF_FOR_VELOCITY);
    accSmooth[axis] = accLPF[axis];
  #define ACC_VALUE accSmooth[axis]
  #else
    accSmooth[axis] = accADC[axis];
  #define ACC_VALUE accADC[axis]
  #endif
    //    accMag += (ACC_VALUE * 10 / (int16_t)acc_1G) * (ACC_VALUE * 10 / (int16_t)acc_1G);
    accMag += (int32_t)ACC_VALUE * ACC_VALUE ;
  #if MAG
  #if defined(MG_LPF_FACTOR)
    mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
  #define MAG_VALUE mgSmooth[axis]
  #else
  #define MAG_VALUE magADC[axis]
  #endif
  #endif
  }
  accMag = accMag * 100 / ((int32_t)acc_1G * acc_1G);

  rotateV(&EstG.V, deltaGyroAngle);
  #if MAG
  rotateV(&EstM.V, deltaGyroAngle);
  #endif

  if ( abs(accSmooth[ROLL]) < acc_25deg && abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0) {
    f.SMALL_ANGLES_25 = 1;
  } else {
    f.SMALL_ANGLES_25 = 0;
  }

  if ( ( 36 < accMag && accMag < 196 ) || f.SMALL_ANGLES_25 )
    for (axis = 0; axis < 3; axis++) {
      int16_t acc = ACC_VALUE;
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
    }
  #if MAG
  for (axis = 0; axis < 3; axis++)
    EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
  #endif

  // Attitude of the estimated vector
  angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z) ;
  angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z) ;

  #if MAG
  // Attitude of the cross product vector GxM
  heading = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  );
  heading += MAG_DECLINIATION * 10; //add declination
  heading = heading / 10;
  if ( heading > 180)      heading = heading - 360;
  else if (heading < -180) heading = heading + 360;
  #endif
}

#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define INIT_DELAY      4000000  // 4 sec initialization delay
#define BARO_TAB_SIZE   21

#define ACC_Z_DEADBAND (acc_1G/50)

void getEstimatedAltitude() {             // 고도
  
  static uint32_t deadLine = INIT_DELAY;

  static int16_t baroHistTab[BARO_TAB_SIZE];
  static int8_t baroHistIdx;
  static int32_t baroHigh;
  
  if (abs(currentTime - deadLine) < UPDATE_INTERVAL) return;
  uint16_t dTime = currentTime - deadLine;
  deadLine = currentTime;

  // **** Alt. Set Point stabilization PID ****

  baroHistTab[baroHistIdx] = BaroAlt / 10;
  baroHigh += baroHistTab[baroHistIdx];
  baroHigh -= baroHistTab[(baroHistIdx + 1) % BARO_TAB_SIZE];
  
  baroHistIdx++;
  if (baroHistIdx == BARO_TAB_SIZE) baroHistIdx = 0;
    
  #if defined(SONAR) && defined(SONAR_BARO_FUSION)

  if (f.ARMED && sonarAlt < 10) {    // 시동 시 지면 고도값을 BaroHome 초기값으로 ( sonarAlt 최소값은 3 )
    BaroHome = (baroHigh * 10.0f / (BARO_TAB_SIZE - 1));
  }

  if (BaroHome != 0) {     // 지면 고도값이 있다면 ( Baro 모드와 무관 )
     //EstAlt = EstAlt * 0.6f + (BaroHome + sonarAlt) * 0.4f;   // Lidar 사용
     EstAlt = EstAlt * 0.6f + (BaroHome + sonarAlt * 2) * 0.4f;   // Lidar 사용, * 2 추가 OK
     //EstAlt = EstAlt * 0.6f + (baroHigh * 10.0f / (BARO_TAB_SIZE - 1)) * 0.4f;   // Baro 사용
  }
  #endif
  
/*
  // 8.6, 0.069 28 에서 고도 안정적
  //conf.D8[PIDALT] = aux2_tmp;    // 송신기 AUX2 Dial 로 ALT PID 의 P 값을 조절해 최적값을 찾는다.
*/

  //P
  int16_t error = constrain(AltHold - EstAlt, -300, 300);
  error = applyDeadband16(error, 10);             //remove small P parameter to reduce noise near zero position
  BaroPID = constrain((conf.P8[PIDALT] * error / 50), -250, +250) + 150;     // 100   150
//  BaroPID = constrain((conf.P8[PIDALT] * error / 100), -150, +150) + 150;
     
  //I
  errorAltitudeI += error * conf.I8[PIDALT] / 50;
  errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
  BaroPID += (errorAltitudeI / 500);              //I in range +/-60

  float invG = InvSqrt(isq(EstG.V.X) + isq(EstG.V.Y) + isq(EstG.V.Z));
  int16_t accZ = (accLPFVel[ROLL] * EstG.V.X + accLPFVel[PITCH] * EstG.V.Y + accLPFVel[YAW] * EstG.V.Z) * invG - acc_1G;
  accZ = applyDeadband16(accZ, ACC_Z_DEADBAND);

  static float vel = 0.0f;
  static float accVelScale = 9.80665f / acc_1G / 10000.0f;

  // Integrator - velocity, cm/sec
  vel += accZ * accVelScale * dTime;

  static int32_t lastBaroAlt = EstAlt;
  float baroVel = (EstAlt - lastBaroAlt) / (dTime / 1000000.0f);
  baroVel = constrain(baroVel, -300, 300);              // constrain baro velocity +/- 300cm/s
  baroVel = applyDeadbandFloat(baroVel, 10);            // to reduce noise near zero
  lastBaroAlt = EstAlt;

  vel = vel * 0.985f + baroVel * 0.015f;
    
  //D
  BaroPID -= constrain(conf.D8[PIDALT] * applyDeadbandFloat(vel, 5) / 20, -150, 150);
  
}

int16_t applyDeadband16(int16_t value, int16_t deadband) {
  if (abs(value) < deadband) {
    value = 0;
  } else if (value > 0) {
    value -= deadband;
  } else if (value < 0) {
    value += deadband;
  }
  return value;
}

float applyDeadbandFloat(float value, int16_t deadband) {
  if (abs(value) < deadband) {
    value = 0;
  } else if (value > 0) {
    value -= deadband;
  } else if (value < 0) {
    value += deadband;
  }
  return value;
}

float InvSqrt(float x) {
  union {
    int32_t i;
    float   f;
  } conv;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

int32_t isq(int32_t x) {
  return x * x;
}

// warp 함수 :  0 이하 입력은 출력값이 없고, 0 이상은 ->  0 ~ 254 까지만 출력함
// 0 ~ 10 입력 -> 0 ~ 10 출력, 250 ~ 254 입력 -> 250 ~ 254 출력
// 255 ~ 255 이상 입력 -> 모두 0 ~ 254 출력
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
}
