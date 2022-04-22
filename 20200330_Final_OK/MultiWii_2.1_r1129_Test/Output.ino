
/***************                  Motor Pin order                  ********************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins
// its not possible to change a PWM output pin just by changing the order

uint8_t PWM_PIN[8] = {9, 10, 11, 3, 6, 5, A2, 12};

/************  Writes the Motors values to the PWM compare register  ******************/
void writeMotors() {       // [1000;2000] => [125;250]

#if (NUMBER_MOTOR > 0)
  OCR1A = motor[0] >> 3;      //  pin 9
#endif
#if (NUMBER_MOTOR > 1)
  OCR1B = motor[1] >> 3;      //  pin 10
#endif
#if (NUMBER_MOTOR > 2)
  OCR2A = motor[2] >> 3;      //  pin 11
#endif
#if (NUMBER_MOTOR > 3)
  OCR2B = motor[3] >> 3;      //  pin 3
#endif

}

/************          Writes the mincommand to all Motors           ******************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i = 0; i < NUMBER_MOTOR; i++) {
    motor[i] = mc;
  }
  writeMotors();
}

/************        Initialize the PWM Timers and Registers         ******************/
void initOutput() {

  /****************            mark all PWM pins as Output             ******************/
  for (uint8_t i = 0; i < NUMBER_MOTOR; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
  }

  /********  Specific PWM Timers & Registers for the atmega328P (NANO)   ************/

#if (NUMBER_MOTOR > 0)
  TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
#endif
#if (NUMBER_MOTOR > 1)
  TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
#endif
#if (NUMBER_MOTOR > 2)
  TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
#endif
#if (NUMBER_MOTOR > 3)
  TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
#endif

  /********  To calibrate all attached ESCs ************/
#if defined(ESC_CALIB_CANNOT_FLY)

  writeAllMotors(ESC_CALIB_HIGH);
  blinkLED(2, 20, 2);
  delay(4000);
  writeAllMotors(ESC_CALIB_LOW);
  blinkLED(3, 20, 2);
  while (1) {
    delay(5000);
    blinkLED(4, 20, 2);
  }
  exit; // statement never reached
#endif

  writeAllMotors(MINCOMMAND);
  delay(300);

}

/********** Mixes the Computed stabilize values to the Motors  ***************/
void mixTable() {
  int16_t maxMotor;
  uint8_t i;

#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

#if NUMBER_MOTOR > 3
  // prevent "yaw jump" during yaw correction
  axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
#endif

  /****************      main Mix Table      ******************/

  motor[0] = PIDMIX(-1, +1, -1); //REAR_R
  motor[1] = PIDMIX(-1, -1, +1); //FRONT_R
  motor[2] = PIDMIX(+1, +1, +1); //REAR_L
  motor[3] = PIDMIX(+1, -1, -1); //FRONT_L

// 테스트 : 배터리 전압강하에 따른 모터 속도 보상 ( 큰 차이는 없음 ) 
  if (vbat < 12.40 && vbat > 6.0) { 
    motor[0] += (12.40 - vbat) * battery_compensation;    //Compensate the esc-1 pulse for voltage drop.
    motor[1] += (12.40 - vbat) * battery_compensation;    //Compensate the esc-2 pulse for voltage drop.
    motor[2] += (12.40 - vbat) * battery_compensation;    //Compensate the esc-3 pulse for voltage drop.
    motor[3] += (12.40 - vbat) * battery_compensation;    //Compensate the esc-4 pulse for voltage drop.
  }
  
  /****************     Filter the Motors values     ******************/
  maxMotor = motor[0];
  for (i = 1; i < NUMBER_MOTOR; i++)
    if (motor[i] > maxMotor) maxMotor = motor[i];
  for (i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);
    if ((rcData[THROTTLE]) < MINCHECK)
#ifndef MOTOR_STOP
      motor[i] = MINTHROTTLE;
#else
      motor[i] = MINCOMMAND;
#endif
    if (!f.ARMED)
      motor[i] = MINCOMMAND;
  }

}
