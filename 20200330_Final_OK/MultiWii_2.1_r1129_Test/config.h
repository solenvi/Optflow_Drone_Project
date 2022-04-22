
#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4     //For PPM

// MINTHROTTLE 값으로 시동 시 4개의 모터 회전 여부가 결정됨
// 송신기 값과 달라도 됨. MINCMD 보다는 높게해야 시동시 모터회전함
#define MINTHROTTLE 1050    //1110 for FB9
 
#define MAXTHROTTLE 1900    //1900 for FB9

#define MINCOMMAND  1000

#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

#define GY_86               // Chinese 10 DOF with  MPU6050 HMC5883L MS5611, LLC

#define YAW_DIRECTION 1
//#define DEADBAND 5          // test

#define SONAR_BARO_FUSION              //mix sonar value with baro
#define SONAR_BARO_FUSION_LC  130      //130 for FB9   //cm, low cut, below = full sonar
#define SONAR_BARO_FUSION_HC  300      //200    //300       //cm, high cut, above = full baro
#define SONAR_BARO_FUSION_RATIO 0.0    //0.0-1.0 amount of each sensor value, 0 = proportionnel between LC and HC

#define OF_DEADBAND 50     // 15
#define OF_ROTATE_I
//#define NAV_SLEW_RATE 50   // 출력 신호가 입력 신호에 따라가는 비율 - 너무 높으면 Overshoot

#define ALLOW_ARM_DISARM_VIA_TX_YAW

#define SERIAL0_COM_SPEED 115200

#define MAG_DECLINIATION  0.0f

//#define MPU6050_LPF_256HZ       // for FB 13 // Less Stable for FB9 - Baro 시 Drop or Up
//#define MPU6050_LPF_188HZ
//#define MPU6050_LPF_98HZ      // Good for FB9
#define MPU6050_LPF_42HZ      // Good more Stable for FB9
//#define MPU6050_LPF_20HZ      // slightly Unstable
//#define MPU6050_LPF_10HZ      // Oscillation - Unstable !!!

#define VBAT              // uncomment this line to activate the vbat code
#define VBATSCALE     127 // change this value if readed Battery voltage is different than real voltage
#define VBATNOMINAL   126 // 12,6V full battery nominal voltage
#define VBATLEVEL1_3S 107 // 10,7V
#define VBATLEVEL2_3S 103 // 10,3V
#define VBATLEVEL3_3S 99  // 9.9V
#define VBATLEVEL4_3S 93  // 9.3V - if vbat ever goes below this value, permanent alarm is triggered
#define NO_VBAT       16 // Avoid beeping without any battery

//#define MOTOR_STOP

#define MIDRC 1500

#define ESC_CALIB_LOW  MINCOMMAND
#define ESC_CALIB_HIGH 2000
//#define ESC_CALIB_CANNOT_FLY  // for PWM, not for PPM

#define VBATFREQ 6       // to read battery voltage - keep equal to PSENSORFREQ unless you know what you are doing
