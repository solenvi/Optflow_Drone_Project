
#define UART_NUMBER 1
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX[UART_NUMBER], serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER], serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];

// Multiwii Serial Protocol 0
#define MSP_VERSION              0

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         1 altitude
#define MSP_BAT                  110   //out message         vbat, powermetersum
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113   //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114   //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_WP_SET               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

#define CURRENTPORT 0

uint32_t read32() {
  uint32_t t = read16();
  t += (uint32_t)read16() << 16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t += (uint16_t)read8() << 8;
  return t;
}
uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT] & 0xff;
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum[CURRENTPORT] = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP[CURRENTPORT]);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum[CURRENTPORT]); UartSendData();
}

void serializeNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}

void serialCom() {
  uint8_t c, n;
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER];// = IDLE;

  for (n = 0; n < UART_NUMBER; n++) {

    while (SerialAvailable(CURRENTPORT)) {
      uint8_t bytesTXBuff = ((uint8_t)(serialHeadTX[CURRENTPORT] - serialTailTX[CURRENTPORT])) % TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = SerialRead(CURRENTPORT);

      if (c_state[CURRENTPORT] == IDLE) {
        c_state[CURRENTPORT] = (c == '$') ? HEADER_START : IDLE;
        if (c_state[CURRENTPORT] == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
      } else if (c_state[CURRENTPORT] == HEADER_START) {
        c_state[CURRENTPORT] = (c == 'M') ? HEADER_M : IDLE;
      } else if (c_state[CURRENTPORT] == HEADER_M) {
        c_state[CURRENTPORT] = (c == '<') ? HEADER_ARROW : IDLE;
      } else if (c_state[CURRENTPORT] == HEADER_ARROW) {
        if (c > INBUF_SIZE) {  // now we are expecting the payload size
          c_state[CURRENTPORT] = IDLE;
          continue;
        }
        dataSize[CURRENTPORT] = c;
        offset[CURRENTPORT] = 0;
        checksum[CURRENTPORT] = 0;
        indRX[CURRENTPORT] = 0;
        checksum[CURRENTPORT] ^= c;
        c_state[CURRENTPORT] = HEADER_SIZE;  // the command is to follow
      } else if (c_state[CURRENTPORT] == HEADER_SIZE) {
        cmdMSP[CURRENTPORT] = c;
        checksum[CURRENTPORT] ^= c;
        c_state[CURRENTPORT] = HEADER_CMD;
      } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] < dataSize[CURRENTPORT]) {
        checksum[CURRENTPORT] ^= c;
        inBuf[offset[CURRENTPORT]++][CURRENTPORT] = c;
      } else if (c_state[CURRENTPORT] == HEADER_CMD && offset[CURRENTPORT] >= dataSize[CURRENTPORT]) {
        if (checksum[CURRENTPORT] == c) {  // compare calculated and transferred checksum
          evaluateCommand();  // we got a valid packet, evaluate it
        }
        c_state[CURRENTPORT] = IDLE;
      }
    }
  }
}

void evaluateCommand() {
  switch (cmdMSP[CURRENTPORT]) {
    case MSP_SET_RAW_RC:
      for (uint8_t i = 0; i < 8; i++) {
        rcData[i] = read16();
      }
      headSerialReply(0);
      break;
    case MSP_SET_PID:
      for (uint8_t i = 0; i < PIDITEMS; i++) {
        conf.P8[i] = read8();
        conf.I8[i] = read8();
        conf.D8[i] = read8();
      }
      headSerialReply(0);
      break;
    case MSP_SET_BOX:
      for (uint8_t i = 0; i < CHECKBOXITEMS; i++) {
        conf.activate[i] = read16();
      }
      headSerialReply(0);
      break;
    case MSP_SET_RC_TUNING:
      conf.rcRate8 = read8();
      conf.rcExpo8 = read8();
      conf.rollPitchRate = read8();
      conf.yawRate = read8();
      conf.dynThrPID = read8();
      conf.thrMid8 = read8();
      conf.thrExpo8 = read8();
      headSerialReply(0);
      break;
    case MSP_SET_MISC:
      headSerialReply(0);
      break;
    case MSP_IDENT:
      headSerialReply(7);
      serialize8(VERSION);   // multiwii version
      serialize8(MULTITYPE); // type of multicopter
      serialize8(MSP_VERSION);         // MultiWii Serial Protocol Version
      serialize32(0);        // "capability"
      break;
    case MSP_STATUS:
      headSerialReply(10);
      serialize16(cycleTime);
      serialize16(i2c_errors_count);
      serialize16(ACC | BARO << 1 | MAG << 2 | GPS << 3 | SONAR << 4);
      serialize32(
#if ACC
        f.ANGLE_MODE << BOXANGLE |
        f.HORIZON_MODE << BOXHORIZON |
#endif
#if BARO
        f.BARO_MODE << BOXBARO |
#endif
#if MAG
        f.MAG_MODE << BOXMAG | f.HEADFREE_MODE << BOXHEADFREE | rcOptions[BOXHEADADJ] << BOXHEADADJ |
#endif

#if OPTFLOW
        f.OPT_HOLD_MODE << BOXOPTHOLD |
#endif
        f.ARMED << BOXARM);
      break;
    
    case MSP_RAW_IMU:
      headSerialReply(18);
      for (uint8_t i = 0; i < 3; i++) serialize16(accSmooth[i]);
      for (uint8_t i = 0; i < 3; i++) serialize16(gyroData[i]);
      for (uint8_t i = 0; i < 3; i++) serialize16(magADC[i]);
      break;
    case MSP_SERVO:
      headSerialReply(16);
      for (uint8_t i = 0; i < 8; i++)
#if defined(SERVO)
        serialize16(servo[i]);
#else
        serialize16(0);
#endif
      break;
    case MSP_MOTOR:
      headSerialReply(16);
      for (uint8_t i = 0; i < 8; i++) {
        serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
      }
      break;
    case MSP_RC:
      headSerialReply(16);
      for (uint8_t i = 0; i < 8; i++) serialize16(rcData[i]);
      break;
    case MSP_ATTITUDE:
      headSerialReply(8);
      for (uint8_t i = 0; i < 2; i++) serialize16(angle[i]);
      serialize16(heading);
      serialize16(headFreeModeHold);
      break;
    case MSP_ALTITUDE:
      headSerialReply(4);
      serialize32(EstAlt);
      break;
    case MSP_BAT:
      headSerialReply(3);
      serialize8(vbat);
      serialize16(intPowerMeterSum);
      break;
    case MSP_RC_TUNING:
      headSerialReply(7);
      serialize8(conf.rcRate8);
      serialize8(conf.rcExpo8);
      serialize8(conf.rollPitchRate);
      serialize8(conf.yawRate);
      serialize8(conf.dynThrPID);
      serialize8(conf.thrMid8);
      serialize8(conf.thrExpo8);
      break;
    case MSP_PID:
      headSerialReply(3 * PIDITEMS);
      for (uint8_t i = 0; i < PIDITEMS; i++) {
        serialize8(conf.P8[i]);
        serialize8(conf.I8[i]);
        serialize8(conf.D8[i]);
      }
      break;
    case MSP_BOX:
      headSerialReply(2 * CHECKBOXITEMS);
      for (uint8_t i = 0; i < CHECKBOXITEMS; i++) {
        serialize16(conf.activate[i]);
      }
      break;
    case MSP_BOXNAMES:
      headSerialReply(strlen_P(boxnames));
      serializeNames(boxnames);
      break;
    case MSP_PIDNAMES:
      headSerialReply(strlen_P(pidnames));
      serializeNames(pidnames);
      break;
    case MSP_MISC:
      headSerialReply(2);
      serialize16(intPowerTrigger1);
      break;
    case MSP_MOTOR_PINS:
      headSerialReply(8);
      for (uint8_t i = 0; i < 8; i++) {
        serialize8(PWM_PIN[i]);
      }
      break;
    case MSP_RESET_CONF:
      if (!f.ARMED) {
        conf.checkNewConf++;
        checkFirstTime();
      }
      headSerialReply(0);
      break;
    case MSP_ACC_CALIBRATION:
      if (!f.ARMED) calibratingA = 400;
      headSerialReply(0);
      break;
    case MSP_MAG_CALIBRATION:
      if (!f.ARMED) f.CALIBRATE_MAG = 1;
      headSerialReply(0);
      break;
    case MSP_EEPROM_WRITE:
      writeParams(0);
      headSerialReply(0);
      break;
    case MSP_DEBUG:
      headSerialReply(8);
      for (uint8_t i = 0; i < 4; i++) {
        serialize16(debug[i]); // 4 variables are here for general monitoring purpose
      }
      break;
    default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
      headSerialError(0);
      break;
  }
  tailSerialReply();
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
  switch (sr) {
      // Note: we may receive weird characters here which could trigger unwanted features during flight.
      //       this could lead to a crash easily.
      //       Please use if (!f.ARMED) where neccessary
  }
}

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a >> 8) & 0xFF);
  serialize8((a >> 16) & 0xFF);
  serialize8((a >> 24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a >> 8) & 0xFF);
}

void serialize8(uint8_t a) {
  uint8_t t = serialHeadTX[CURRENTPORT];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][CURRENTPORT] = a;
  checksum[CURRENTPORT] ^= a;
  serialHeadTX[CURRENTPORT] = t;
}

ISR(USART_UDRE_vect) {            // Serial 0 on a PROMINI
  uint8_t t = serialTailTX[0];
  if (serialHeadTX[0] != t) {
    if (++t >= TX_BUFFER_SIZE) t = 0;
    UDR0 = serialBufferTX[t][0];  // Transmit next byte in the ring
    serialTailTX[0] = t;
  }
  if (t == serialHeadTX[0]) UCSR0B &= ~(1 << UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
}

void UartSendData() {
  UCSR0B |= (1 << UDRIE0);
}

void SerialOpen(uint8_t port, uint32_t baud) {
  //static void inline SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud - 1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud - 1) / 2);
  switch (port) {
    case 0: UCSR0A  = (1 << U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); break;
  }
}

static void inline SerialEnd(uint8_t port) {
  switch (port) {
    case 0: UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0)); break;
  }
}

static void inline store_uart_in_buf(uint8_t data, uint8_t portnum) {
  uint8_t h = serialHeadRX[portnum];
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX[portnum]) return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX[portnum]][portnum] = data;
  serialHeadRX[portnum] = h;
}

ISR(USART_RX_vect)  {
  store_uart_in_buf(UDR0, 0);
}

uint8_t SerialRead(uint8_t port) {
  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

uint8_t SerialAvailable(uint8_t port) {
  return (serialHeadRX[port] - serialTailRX[port]) % RX_BUFFER_SIZE;
}
