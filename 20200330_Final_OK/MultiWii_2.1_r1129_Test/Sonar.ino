
#ifdef SONAR     // -> Lidar로 변경

void initSonar() {     // Pro Mini 로부터 Lidar PWM 값 Read
  
  pinMode(4,INPUT);
  
}

inline void sonarUpdate() {

  unsigned long range = pulseIn(4, HIGH, 20000);    // 펄스 Read
  int dist = map(range, 0, 2000, 0, 255);           // 읽힌 펄스의 값을 PWM범위로 다시 변환

  sonarAlt = dist;

  if ( sonarAlt > 80 && sonarAlt < 110 ) {
    digitalWrite (7, HIGH);
  } else {
    digitalWrite (7, LOW);  
  }

}

#endif
