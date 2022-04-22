/* Optical flow	sensor reading and calculations	*/
/* (c) alexmos 2012	*/

#ifdef OPTFLOW

static int16_t sum_dx  = 0, sum_dy = 0; // sensor's row data accumulators

inline void initOptflow() {     // Pro Mini 로부터 Optflow PWM 값 Read
  pinMode(5,INPUT);
  pinMode(6,INPUT);
}
   
/* PID calculations. Outputs optflow_angle[ROLL],  optflow_angle[PITCH] */
inline void Optflow_update() {
  static int16_t optflowErrorI[2] = { 0, 0 };
  static int16_t prevHeading = 0;
  static int8_t optflowUse = 0;
  int8_t axis;
    
  // enable OPTFLOW only in LEVEL mode and if GPS is not used
  if(f.ANGLE_MODE ==  1 && optflowMode == 1) {
    // init first time mode enabled
    if(!optflowUse) {
      optflowErrorI[0] = 0; optflowErrorI[1] = 0;
      prevHeading = heading;
      optflowUse = 1;
      return;
    }
    
    // Read sensors
    Optflow_read();                     // OPTFLOW 센서 값 읽어들임

    // Rotate I to follow global axis   // Heading Rotation 보상
    #ifdef OF_ROTATE_I
      int16_t dif = heading - prevHeading;
      if (dif <= - 180) dif += 360;
      else if (dif >= + 180) dif -= 360;

      if(abs(dif) > 5) { // rotate by 5-degree steps
        rotate16(optflowErrorI, dif * 10);
        prevHeading = heading;
      }
    #endif

    float sin_yaw_y = sin(heading * 0.0174532925f);
    float cos_yaw_x = cos(heading * 0.0174532925f);
    
    // Use sensor only inside DEADBAND - 즉, Roll과 Pitch 스틱이 중립 위치인 경우 ( < 15 )
    if(abs(rcCommand[ROLL]) < OF_DEADBAND && abs(rcCommand[PITCH]) < OF_DEADBAND) {       // 50    15
      // calculate velocity
      for(axis=0; axis<2; axis++) {
       
        optflow_pos[axis] = optflow_pos[axis] * (OF_DEADBAND - abs(rcCommand[axis])) / OF_DEADBAND; // 16 bit ok: 100*100 = 10000

        optflowErrorI[axis] +=  optflow_pos[axis]; 
        optflowErrorI[axis] = constrain(optflowErrorI[axis], -20000, 20000);

     //   optflow_angle[axis] = optflow_pos[axis];   // Drift 감소하나 Overshoot 증가

       }

     // GPS 참조해 새로 시도하는 부분 ( 좀 더 부드럽게 Postion Hold. but 고도에 영향을 주는 것 같음 ) //////
     // 약 1.5m 높이에서는 aux2_tmp 값 4가 적당함 ///////////////////// 
     #if defined(NAV_SLEW_RATE)     // SR (I/O 반응률) 적용 -> Position Hold 비교적 양호
        nav_rated[1] += constrain(wrap_18000(optflow_pos[1] - nav_rated[1]), -NAV_SLEW_RATE, NAV_SLEW_RATE);
        nav_rated[0] += constrain(wrap_18000(optflow_pos[0] - nav_rated[0]), -NAV_SLEW_RATE, NAV_SLEW_RATE);
        optflow_angle[0]  = (nav_rated[1] * cos_yaw_x - nav_rated[0] * sin_yaw_y);
        optflow_angle[1]  = -(nav_rated[1] * sin_yaw_y + nav_rated[0] * cos_yaw_x);
     #else   // 기체 회전 시에도 PosHold ( 롤링, 피칭 상하 Overshoot 감소 - Toilet Bowl 현상 있음)
        optflow_angle[0]  = (optflow_pos[1] * cos_yaw_x - optflow_pos[0] * sin_yaw_y);
        optflow_angle[1]  = -(optflow_pos[1] * sin_yaw_y + optflow_pos[0] * cos_yaw_x);
     #endif

    } else {
      optflow_angle[0] = 0; optflow_angle[1] = 0;
    }
        
    // Apply I-term unconditionally
    for(axis=0; axis<2; axis++) {
      optflow_angle[axis] = constrain(optflow_angle[axis] + (int16_t)((int32_t)optflowErrorI[axis] * 5  / 5000), -300,  300);
    }
    
  } else if(optflowUse) { // switch mode off
    optflow_angle[0] = 0; optflow_angle[1] = 0;
    optflowUse = 0;
  }  
}

inline void	Optflow_read() {
  
  unsigned long valX = pulseIn(5, HIGH, 20000);   // pulse Read
  int16_t outX = map(valX, 0, 2000, 0, 255);    // convert pulse to PWM 

  outX = outX - 65;
  optflow_pos[0] = aux2_tmp * outX;             // Pitch, 위도(LAT)

  unsigned long valY = pulseIn(6, HIGH, 20000);   // pulse Read
  int16_t outY = map(valY, 0, 2000, 0, 255);    // default 255 -> - 값 수렴

  outY = outY - 65;
  optflow_pos[1] = aux2_tmp * outY;             // Roll, 경도(LON)

}

inline void rotate16(int16_t *V, int16_t delta) {
  int16_t tmp = V[0];
  V[0]-= (int16_t)( ((int32_t)delta) * V[1] / 573);
  V[1]+= (int16_t)( ((int32_t)delta) * tmp / 573); 
}

#endif
