
#define CHECK_PINMODE              pinMode (7, OUTPUT);

#define LEDPIN_PINMODE             pinMode (13, OUTPUT);
#define LEDPIN_TOGGLE              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
#define LEDPIN_OFF                 PORTB &= ~(1<<5);
#define LEDPIN_ON                  PORTB |= (1<<5);

#define POWERPIN_PINMODE           pinMode (8, OUTPUT);  // Red //pinMode (12, OUTPUT);
#define POWERPIN_ON                PORTB |= 1;    //PORTB |= 1<<4;
#define POWERPIN_OFF               PORTB &= ~1;   //PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12

#define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);

//RX PIN assignment inside the port //for PORTD
#define THROTTLEPIN                2      //13

#define V_BATPIN                   A3     // Analog PIN 3

#if defined(GY_86)
#define MPU6050
#define HMC5883
#define MS561101BA
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
#define MPU6050_I2C_AUX_MASTER // MAG connected to the AUX I2C bus of MPU6050
#endif

#define GYRO 1
#define ACC 1
#define BARO 1
#define MAG 1
#define GPS   0         // not used, only for serial com
#define SONAR  1
#define OPTFLOW   1

#define MULTITYPE 3

#define RC_CHANS 8
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 50    //20

#define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  
#define NUMBER_MOTOR     4
