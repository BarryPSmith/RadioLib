#include "SX126x.h"

volatile bool SX126x::_interruptFlag = false;

#if 0
SX126x::SX126x(Module* mod) : PhysicalLayer(SX126X_CRYSTAL_FREQ, SX126X_DIV_EXPONENT, SX126X_MAX_PACKET_LENGTH) {
#else
SX126x::SX126x(Module* mod) {
#endif
  _mod = mod;
}

int16_t SX126x::begin(float bw, uint8_t sf, uint8_t cr, uint16_t syncWord, float currentLimit, uint16_t preambleLength, float tcxoVoltage) {
  return(begin_i(bw * 10 + 0.1, sf, cr, syncWord, currentLimit / 2.5, preambleLength, tcxoVoltage * 10 + 0.5));
}

int16_t SX126x::begin_i(uint16_t bwkHz_x10, uint8_t sf, uint8_t cr, uint16_t syncWord,
  uint8_t currentLimit_mA_div2_5, uint16_t preambleLength, uint8_t tcxoVoltage_x10) {
  // set module properties
  _mod->init(RADIOLIB_USE_SPI, RADIOLIB_INT_BOTH);

  attachInterrupt(digitalPinToInterrupt(_mod->getInt0()), interruptActionStatic, RISING);

  // BW in kHz and SF are required in order to calculate LDRO for setModulationParams
  _bwkHz_x10 = bwkHz_x10;
  _sf = sf;

  // initialize configuration variables (will be overwritten during public settings configuration)
  _bw = SX126X_LORA_BW_125_0;
  _cr = SX126X_LORA_CR_4_7;
  _ldro = 0x00;
  _crcType = SX126X_LORA_CRC_ON;
  _preambleLength = preambleLength;
  _tcxoDelay = 0;

  // set mode to standby
  int16_t state = standby(SX126X_STANDBY_RC);
  if(state != ERR_NONE) {
    return(state);
  }

  // configure settings not accessible by API
  state = config(SX126X_PACKET_TYPE_LORA);
  if(state != ERR_NONE) {
    return(state);
  }

  // set TCXO control, if requested
  if(tcxoVoltage_x10 > 0) {
    state = setTCXO_i(tcxoVoltage_x10);
    if(state != ERR_NONE) {
      return(state);
    }
  }

  // configure publicly accessible settings
  state = setSpreadingFactor(sf);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setBandwidth_i(bwkHz_x10);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setCodingRate(cr);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setSyncWord(syncWord);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setCurrentLimit_i(currentLimit_mA_div2_5);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setPreambleLength(preambleLength);
  if(state != ERR_NONE) {
    return(state);
  }

  // set publicly accessible settings that are not a part of begin method
  state = setDio2AsRfSwitch(false);

  return(state);
}

int16_t SX126x::beginFSK(float br, float freqDev, float rxBw, float currentLimit, uint16_t preambleLength, float dataShaping, float tcxoVoltage) {
  return(beginFSK_i(br * 1000 + 0.5, freqDev * 1000 + 0.5, rxBw * 10 + 0.5, currentLimit / 2.5 + 0.5, preambleLength, dataShaping * 10 + 0.5, tcxoVoltage * 10 + 0.5));
}

int16_t SX126x::beginFSK_i(uint32_t br_bps, uint32_t freqDev_Hz, uint16_t rxBw_kHz_x10,
  uint8_t currentLimit_mA_div2_5, uint16_t preambleLength, uint8_t dataShaping_x10,
  uint8_t tcxoVoltage_x10) {
  // set module properties
  _mod->init(RADIOLIB_USE_SPI, RADIOLIB_INT_BOTH);

  // initialize configuration variables (will be overwritten during public settings configuration)
  _br = 21333;                                  // 48.0 kbps
  _freqDev = 52428;                             // 50.0 kHz
  _rxBw = SX126X_GFSK_RX_BW_156_2;
  _rxBwKhz_x10 = 1562;
  _pulseShape = SX126X_GFSK_FILTER_GAUSS_0_5;
  _crcTypeFSK = SX126X_GFSK_CRC_2_BYTE_INV;     // CCIT CRC configuration
  _preambleLengthFSK = preambleLength;
  _addrComp = SX126X_GFSK_ADDRESS_FILT_OFF;

  // set mode to standby
  int16_t state = standby(SX126X_STANDBY_RC);
  if(state != ERR_NONE) {
    return(state);
  }

  // configure settings not accessible by API
  state = config(SX126X_PACKET_TYPE_GFSK);
  if(state != ERR_NONE) {
    return(state);
  }

  // set TCXO control, if requested
  if(tcxoVoltage_x10 > 0) {
    state = setTCXO_i(tcxoVoltage_x10);
    if(state != ERR_NONE) {
      return(state);
    }
  }

  // configure publicly accessible settings
  state = setBitRate_i(br_bps);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setFrequencyDeviation_i(freqDev_Hz);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setRxBandwidth_i(rxBw_kHz_x10);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setCurrentLimit_i(currentLimit_mA_div2_5);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setDataShaping_i(dataShaping_x10);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setPreambleLength(preambleLength);
  if(state != ERR_NONE) {
    return(state);
  }

  // set publicly accessible settings that are not a part of begin method
  uint8_t sync[] = {0x2D, 0x01};
  state = setSyncWord(sync, 2);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setWhitening(true, 0x0100);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setDio2AsRfSwitch(false);

  return(state);
}

int16_t SX126x::processLoop()
{
#ifdef  NO_DIO1_INTERRUPT
  static bool lastSignalled = false;
  bool curVal = digitalRead(_mod->getInt0());
  if (curVal && !lastSignalled)
    _interruptFlag = true;
  lastSignalled = curVal;
#endif //  NO_DIO1_INTERRUPT


  // check if an interrupt has occurred
  auto sreg = SREG;
  cli();
  bool localFlag = _interruptFlag;
  _interruptFlag = false;
  SREG = sreg;
  if (!localFlag) {
    return(ERR_NONE);
  }
  
  switch (_curStatus)
  {
  case SX126X_STATUS_MODE_TX:
    //Handle the interrupt in TX mode
    if (_txDoneFunc)
      _txDoneFunc();
    _curStatus = SX126X_STATUS_MODE_STDBY_RC;
    break;
  case SX126X_STATUS_MODE_RX:
    //Handle the interrupt in RX mode
    return(rxInterruptAction());
  }
  
  return ERR_NONE;
}

int16_t SX126x::transmit(uint8_t* data, size_t len, uint8_t addr) {
  // set mode to standby
  int16_t state = standby();
  if(state != ERR_NONE) {
    return(state);
  }

  // check packet length
  if(len > SX126X_MAX_PACKET_LENGTH) {
    return(ERR_PACKET_TOO_LONG);
  }

  uint32_t timeout = 0;

  // get currently active modem
  uint8_t modem = getPacketType();
  if(modem == SX126X_PACKET_TYPE_LORA) {
    // calculate timeout (150% of expected time-on-air)
    // Buggerit. getTimeOnAir is an expensive function. Just use 1 second...
    timeout = 1000000; // (getTimeOnAir(len) * 3) / 2;

  } else if(modem == SX126X_PACKET_TYPE_GFSK) {
    // calculate timeout (500% of expected time-on-air)
    timeout = getTimeOnAir(len) * 5;

  } else {
    return(ERR_UNKNOWN);
  }

  RADIOLIB_DEBUG_PRINT(F("Timeout in "));
  RADIOLIB_DEBUG_PRINT(timeout);
  RADIOLIB_DEBUG_PRINTLN(F(" us"));

  // start transmission
  state = startTransmit(data, len, addr);
  if(state != ERR_NONE) {
    return(state);
  }

  // wait for packet transmission or timeout
  uint32_t start = micros();
  while(!(digitalRead(_mod->getInt0()))) {
    yield();
    if(micros() - start > timeout) {
      clearIrqStatus();
      return(ERR_TX_TIMEOUT);
    }
  }
  uint32_t elapsed_us = micros() - start;

  // update data rate
  _dataRate = (len * 8 * usPerSecond) / elapsed_us ;

  // clear interrupt flags
  state = clearIrqStatus();
  if(state != ERR_NONE) {
    return(state);
  }

  // set mode to standby to disable transmitter
  state = standby();

  return(state);
}

int16_t SX126x::receive(uint8_t* data, size_t len) {
  // set mode to standby
  int16_t state = standby();
  if(state != ERR_NONE) {
    return(state);
  }

  uint32_t timeout = 0;

  // get currently active modem
  uint8_t modem = getPacketType();
  if(modem == SX126X_PACKET_TYPE_LORA) {
    // calculate timeout (100 LoRa symbols, the default for SX127x series)
    uint32_t symbolLength_us = ((10 * usPerSecond / kilo) << _sf) / (_bwkHz_x10);
    timeout = symbolLength_us * 100;

  } else if(modem == SX126X_PACKET_TYPE_GFSK) {
    // calculate timeout (500 % of expected time-one-air)
    size_t maxLen = len;
    if(len == 0) {
      maxLen = 0xFF;
    }
    // uint32_t brBps = (SX126X_CRYSTAL_FREQ * MHz * 32) / _br;
    // timeout = (uint32_t)(maxLen * 8 * microsPerSecond * 5) / brBps;
    // the below is equivalent to the above but does fewer operations:
    timeout = (maxLen * 8 * 5 * _br) / (SX126X_CRYSTAL_FREQ * 32);

  } else {
    return(ERR_UNKNOWN);
  }

  RADIOLIB_DEBUG_PRINT(F("Timeout in "));
  RADIOLIB_DEBUG_PRINT(timeout);
  RADIOLIB_DEBUG_PRINTLN(F(" us"));

  // start reception
  uint32_t timeoutValue = (timeout * 8) / 125; // equivalent to timeout / 15.625
  state = startReceive(timeoutValue);
  if(state != ERR_NONE) {
    return(state);
  }

  // wait for packet reception or timeout
  uint32_t start = micros();
  while(!digitalRead(_mod->getInt0())) {
    if(micros() - start > timeout) {
      fixImplicitTimeout();
      clearIrqStatus();
      return(ERR_RX_TIMEOUT);
    }
  }

  // timeout fix is recommended after any reception with active timeout
  state = fixImplicitTimeout();
  if(state != ERR_NONE) {
    return(state);
  }

  // read the received data
  return(readData(data, len));
}

int16_t SX126x::transmitDirect(uint32_t frf) {
  // user requested to start transmitting immediately (required for RTTY)
  int16_t state = ERR_NONE;
  if(frf != 0) {
    state = setRfFrequency(frf);
  }
  if(state != ERR_NONE) {
    return(state);
  }

  // start transmitting
  uint8_t data[] = {SX126X_CMD_NOP};
  return(SPIwriteCommand(SX126X_CMD_SET_TX_CONTINUOUS_WAVE, data, 1));
}

int16_t SX126x::receiveDirect() {
  // SX126x is unable to ouput received data directly
  return(ERR_UNKNOWN);
}

int16_t SX126x::scanChannel() {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_LORA) {
    return(ERR_WRONG_MODEM);
  }

  // set mode to standby
  int16_t state = standby();
  if(state != ERR_NONE) {
    return(state);
  }
  
  state = setDioIrqParams(SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE, SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_CAD_DONE);
  if(state != ERR_NONE) {
    return(state);
  }

  // clear interrupt flags
  state = clearIrqStatus();
  if(state != ERR_NONE) {
    return(state);
  }

  // set mode to CAD
  state = setCad();
  if(state != ERR_NONE) {
    return(state);
  }

  
  uint32_t symbolLength = ((uint32_t)(10 * 1000) << _sf) / (_bwkHz_x10);
  uint32_t timeout = 18 * symbolLength;
  uint32_t start = micros();
  // wait for channel activity detected or timeout
  while(!digitalRead(_mod->getInt0())) {
    if (micros() - start > timeout) {
      return(ERR_RX_TIMEOUT);
    yield();
    }
  }

  // check CAD result
  uint16_t cadResult;
  state = getIrqStatus(&cadResult);
  if (state != ERR_NONE)
    return state;

  if(cadResult & SX126X_IRQ_CAD_DETECTED) {
    // detected some LoRa activity
    clearIrqStatus();
    return(LORA_DETECTED);
  } else if(cadResult & SX126X_IRQ_CAD_DONE) {
    // channel is free
    clearIrqStatus();
    return(CHANNEL_FREE);
  }

  return(ERR_UNKNOWN);
}

int16_t SX126x::isChannelBusy(bool scanIfInRx, bool reenterAfterScan) {
  //uint8_t status;
  int16_t state;// = getStatus(&status);
  /*if(state != ERR_NONE) {
    return(state);
  }*/
  bool wasRx = _curStatus == SX126X_STATUS_MODE_RX; // ((status & SX126X_STATUS_MODE_MASK) == SX126X_STATUS_MODE_RX);
  if(wasRx) {
    RADIOLIB_DEBUG_PRINTLN(F("isChannelBusy queried in receive."));
    RADIOLIB_DEBUG_PRINT(F("maybe receiving: "));
    RADIOLIB_DEBUG_PRINTLN(_maybeReceiving);
    if(_maybeReceiving) {
      // We might have detected a preamble but never got the message
      // so we need some timeout logic.
      // This estimates a timeout at least as long as our longest packet we've ever received,
      // and at least 3 times the average packet length.
      // We might want to tweak this algorithm (what exists in literature?)
      uint32_t preambleDetectTimeout = (2 * _avgPacketMicros + _longestPacketMicros);
      if (preambleDetectTimeout > maxBusyTimeout) {
        preambleDetectTimeout = maxBusyTimeout;
      }
      uint32_t curMicros = micros();
      RADIOLIB_DEBUG_PRINT(F("dm: "));
      RADIOLIB_DEBUG_PRINT(curMicros - _lastPreambleDetMicros);
      RADIOLIB_DEBUG_PRINT(F(" apm: "));
      RADIOLIB_DEBUG_PRINT(_avgPacketMicros);
      RADIOLIB_DEBUG_PRINT(F(" lpm: "));
      RADIOLIB_DEBUG_PRINTLN(_longestPacketMicros);
      if((curMicros - _lastPreambleDetMicros) < preambleDetectTimeout) {
        return(LORA_DETECTED);
      }
    }
    if(!scanIfInRx) {
      return(CHANNEL_FREE);
    }
  }

  int16_t ret = scanChannel();
  if (wasRx &&
    (reenterAfterScan || ret != CHANNEL_FREE))
  {
    state = startReceive();
    if (state != ERR_NONE) {
      return(state);
    }
  }
  return(ret);
}

int16_t SX126x::sleep() {
  uint8_t data[] = {SX126X_SLEEP_START_WARM | SX126X_SLEEP_RTC_OFF};
  int16_t state = SPIwriteCommand(SX126X_CMD_SET_SLEEP, data, 1, false);
  if (state == ERR_NONE) {
    _curStatus = 0;
  }

  // wait for SX126x to safely enter sleep mode
  delayMicroseconds(500);

  return(state);
}

int16_t SX126x::standby() {
   return(SX126x::standby(SX126X_STANDBY_XOSC));
}

int16_t SX126x::standby(uint8_t mode) {
  uint8_t data[] = {mode};
  int16_t state = SPIwriteCommand(SX126X_CMD_SET_STANDBY, data, 1);
  if (state == ERR_NONE) {
    _curStatus = mode == SX126X_STANDBY_RC ? SX126X_STATUS_MODE_STDBY_RC : SX126X_STATUS_MODE_STDBY_XOSC;
  }
  return(state);
}

#if 0
void SX126x::setDio1Action(void (*func)(void)) {
  attachInterrupt(digitalPinToInterrupt(_mod->getInt0()), func, RISING);
}
#endif

void SX126x::setTxDoneAction(void (*func)(void)) {
  _txDoneFunc = func;
}

void SX126x::setRxDoneAction(void (*func)(void)) {
  _rxDoneFunc = func;
}

int16_t SX126x::startTransmit(uint8_t* data, size_t len, uint8_t addr) {
  // suppress unused variable warning
  (void)addr;

  // check packet length
  if(len > SX126X_MAX_PACKET_LENGTH) {
    return(ERR_PACKET_TOO_LONG);
  }

  // maximum packet length is decreased by 1 when address filtering is active
  if((_addrComp != SX126X_GFSK_ADDRESS_FILT_OFF) && (len > SX126X_MAX_PACKET_LENGTH - 1)) {
    return(ERR_PACKET_TOO_LONG);
  }

  // set packet Length
  int16_t state = ERR_NONE;
  uint8_t modem = getPacketType();
  if(modem == SX126X_PACKET_TYPE_LORA) {
    state = setPacketParams(_preambleLength, _crcType, len);
  } else if(modem == SX126X_PACKET_TYPE_GFSK) {
    state = setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, _packetType, len);
  } else {
    return(ERR_UNKNOWN);
  }
  if(state != ERR_NONE) {
    return(state);
  }

  //Ensure our Rx action isn't active - it clears the IRQ flags as it goes.
  // set DIO mapping
  state = setDioIrqParams(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT, SX126X_IRQ_TX_DONE);
  if(state != ERR_NONE) {
    return(state);
  }

  // set buffer pointers
  state = setBufferBaseAddress();
  if(state != ERR_NONE) {
    return(state);
  }

  // write packet to buffer
  state = writeBuffer(data, len);
  if(state != ERR_NONE) {
    return(state);
  }

  // clear interrupt flags
  state = clearIrqStatus();
  if(state != ERR_NONE) {
    return(state);
  }

  // fix sensitivity
  state = fixSensitivity();
  if(state != ERR_NONE) {
    return(state);
  }

  // start transmission
  state = setTx(SX126X_TX_TIMEOUT_NONE);
  if(state != ERR_NONE) {
    return(state);
  }

  // wait for BUSY to go low (= PA ramp up done)
  while(digitalRead(_mod->getInt1()));

  return(state);
}

int16_t SX126x::startReceive(uint32_t timeout) {
  int16_t state = startReceiveCommon();
  if(state != ERR_NONE) {
    return(state);
  }

  // set mode to receive
  state = setRx(timeout);

  return(state);
}

int16_t SX126x::startReceiveDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod) {
  // datasheet claims time to go to sleep is ~500us, same to wake up, compensate for that with 1 ms + TCXO delay
  uint32_t transitionTime = _tcxoDelay + 1000;
  sleepPeriod -= transitionTime;

  uint32_t sleepPeriodRaw = (sleepPeriod * 8) / 125;
  // this check of the high byte also catches underflow when we subtracted transitionTime
  if((sleepPeriodRaw & 0xFF000000) || (sleepPeriodRaw == 0)) {
    return(ERR_INVALID_SLEEP_PERIOD);
  }

  // divide by 15.625
  uint32_t rxPeriodRaw = (rxPeriod * 8) / 125;
  // check 24 bit limit and zero value (likely not intended)
  if((rxPeriodRaw & 0xFF000000) || (rxPeriodRaw == 0)) {
    return(ERR_INVALID_RX_PERIOD);
  }

  int16_t state = startReceiveCommon();
  if(state != ERR_NONE) {
    return(state);
  }

  uint8_t data[6] = {(uint8_t)((rxPeriodRaw >> 16) & 0xFF), (uint8_t)((rxPeriodRaw >> 8) & 0xFF), (uint8_t)(rxPeriodRaw & 0xFF),
                     (uint8_t)((sleepPeriodRaw >> 16) & 0xFF), (uint8_t)((sleepPeriodRaw >> 8) & 0xFF), (uint8_t)(sleepPeriodRaw & 0xFF)};
  state = SPIwriteCommand(SX126X_CMD_SET_RX_DUTY_CYCLE, data, 6);

  if (state == ERR_NONE) {
    _curStatus = SX126X_STATUS_MODE_RX;
  }
  return(state);
}

int16_t SX126x::startReceiveDutyCycleAuto(uint16_t senderPreambleLength, uint16_t minSymbols) {
  if(senderPreambleLength == 0) {
    senderPreambleLength = _preambleLength;
  }

  // worst case is that the sender starts transmiting when we're just less than minSymbols from going back to sleep.
  // in this case, we don't catch minSymbols before going to sleep,
  // so we must be awake for at least that long before the sender stops transmitting.
  uint16_t sleepSymbols = senderPreambleLength - 2 * minSymbols;

  // if we're not to sleep at all, just use the standard startReceive.
  if(2 * minSymbols > senderPreambleLength) {
    return(startReceive());
  }

  uint32_t symbolLength = ((uint32_t)(10 * 1000) << _sf) / (_bwkHz_x10);
  uint32_t sleepPeriod = symbolLength * sleepSymbols;
  RADIOLIB_DEBUG_PRINT(F("Auto sleep period: "));
  RADIOLIB_DEBUG_PRINTLN(sleepPeriod);

  // when the unit detects a preamble, it starts a timer that will timeout if it doesn't receive a header in time.
  // the duration is sleepPeriod + 2 * wakePeriod.
  // The sleepPeriod doesn't take into account shutdown and startup time for the unit (~1ms)
  // We need to ensure that the timout is longer than senderPreambleLength.
  // So we must satisfy: wakePeriod > (preamblePeriod - (sleepPeriod - 1000)) / 2. (A)
  // we also need to ensure the unit is awake to see at least minSymbols. (B)
  uint32_t wakePeriod = max(
    (symbolLength * (senderPreambleLength + 1) - (sleepPeriod - 1000)) / 2, // (A)
    symbolLength * (minSymbols + 1)); //(B)
  RADIOLIB_DEBUG_PRINT(F("Auto wake period: "));
  RADIOLIB_DEBUG_PRINTLN(wakePeriod);

  //If our sleep period is shorter than our transition time, just use the standard startReceive
  if(sleepPeriod < _tcxoDelay + 1016) {
    return(startReceive());
  }

  return(startReceiveDutyCycle(wakePeriod, sleepPeriod));
}

int16_t SX126x::startReceiveCommon() {
  // set DIO mapping
  uint16_t irqMask = SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR | SX126X_IRQ_HEADER_ERR,
           dio1Mask = SX126X_IRQ_RX_DONE ;

  //if (_enableIsChannelBusy) {
    irqMask |= SX126X_IRQ_PREAMBLE_DETECTED;
    dio1Mask |= SX126X_IRQ_PREAMBLE_DETECTED;
  //}

  int16_t state = setDioIrqParams(irqMask, dio1Mask);
  if(state != ERR_NONE) {
    return(state);
  }

  // set buffer pointers
  //state = setBufferBaseAddress();
  if(state != ERR_NONE) {
    return(state);
  }

  // clear interrupt flags
  state = clearIrqStatus();
  if(state != ERR_NONE) {
    return(state);
  }

  // _maybeReceiving might have been left over from last time we were in startReceive
  _maybeReceiving = false;

  return(state);
}

void SX126x::interruptActionStatic() {
  _interruptFlag = true;
}

int16_t SX126x::rxInterruptAction() {
  // freeze micros at entry.
  uint32_t entryMicros = micros();

  if (_curStatus != SX126X_STATUS_MODE_RX)
    return(ERR_UNKNOWN);

  uint8_t count = 0;
  /* I wish there was an atomic get & clear IRQ status in one go,
     so we don't have to worry about what happens if another flag is signalled between these two calls.
     But there isn't, so this is what we get:

     We're guaranteed to run until the IRQ register is empty.
     If DIO1 goes low and high again and the SX126x triggers additional interrupts on the MCU while we're in this function, 
     we'll run again but the register will be empty so no harm 
    */
  
  uint16_t irqStatus;
  do {
    if (count++ > 3) {
      RADIOLIB_DEBUG_PRINTLN(F("Bailed from rxInterruptAction"));
      return(ERR_UNKNOWN);
    }
        

    int16_t state = getIrqStatus(&irqStatus);
    if (state != ERR_NONE) {
      return(state);
    }
    irqStatus = irqStatus & (SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_RX_DONE);

    state = clearIrqStatus(irqStatus);
    if (state != ERR_NONE) {
      return(state);
    }
    if (irqStatus) {
      rxInterruptAction(irqStatus, entryMicros);
    }
  } while (irqStatus);

  return (ERR_NONE);
}



void SX126x::rxInterruptAction(uint16_t irqStatus, uint32_t entryMicros)
{
  if(irqStatus & SX126X_IRQ_PREAMBLE_DETECTED) {
    _maybeReceiving = true;
    _lastPreambleDetMicros = entryMicros;
  }
  if (irqStatus & SX126X_IRQ_RX_DONE) {
    _maybeReceiving = false;
    // update our averages
    // we're in an ISR here, don't waste cycles with floating point or division operations.
    // exponential moving average weighted for the last N values is approximately
    // Avg(n) = 1 / N * Val(n) + (N - 1) / N * Avg(n - 1)
    uint32_t pktMicros = entryMicros - _lastPreambleDetMicros;
    _avgPacketMicros = 
      (pktMicros >> SX126X_PACKET_AVG_SHIFT_COUNT)
      + 
      ((_avgPacketMicros * SX126X_PACKET_AVG_NUM) >> SX126X_PACKET_AVG_SHIFT_COUNT);
    if(pktMicros > _longestPacketMicros) {
      _longestPacketMicros = pktMicros;
    }
    if(_rxDoneFunc)
      _rxDoneFunc();
  }
}

int16_t SX126x::readData(uint8_t* data, size_t len) {
  int16_t state;
  // set mode to standby
  // BS: Why?
  /* state = standby();
  if(state != ERR_NONE) {
    return(state);
  }*/

  // check integrity CRC
  uint16_t irq;
  state = getIrqStatus(&irq);
  if (state != ERR_NONE) {
    return state;
  }
  if((irq & SX126X_IRQ_CRC_ERR) || (irq & SX126X_IRQ_HEADER_ERR)) {
    clearIrqStatus();
    return(ERR_CRC_MISMATCH);
  }

  // get packet length
  size_t length = len;

  uint8_t packetLen, startAddr;
  state = getBufferStatus(&packetLen, &startAddr);
  if (state != ERR_NONE) {
    return(state);
  }

  if (length > packetLen) {
    length = packetLen;
  }

  // read packet data
  state = readBuffer(data, length, startAddr);
  if(state != ERR_NONE) {
    return(state);
  }

  // clear interrupt flags
  state = clearIrqStatus();

  return(state);
}

int16_t SX126x::setBandwidth(float bw) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_LORA) {
    return(ERR_WRONG_MODEM);
  }
  // ensure we don't truncate when changing to int16
  // check with ! and interior comparisons to catch NaN and infinity
  if (!(bw > 0 && bw < 501)) {
    return(ERR_INVALID_BANDWIDTH);
  }

  //note we add 0.1 here to ensure we end up with 312 instead of 313 if supplied 31.25
  return(setBandwidth_i((uint16_t)(bw * 10 + 0.1)));
}

int16_t SX126x::setBandwidth_i(uint16_t bwkHz_x10) {
  // check alowed bandwidth values
  switch (bwkHz_x10) {
  case 78:
      _bw = SX126X_LORA_BW_7_8;
      break;
    case 114:
      _bw = SX126X_LORA_BW_10_4;
      break;
    case 156:
      _bw = SX126X_LORA_BW_15_6;
      break;
    case 208:
      _bw = SX126X_LORA_BW_20_8;
      break;
    case 312:
      _bw = SX126X_LORA_BW_31_25;
      break;
    case 417:
      _bw = SX126X_LORA_BW_41_7;
      break;
    case 625:
      _bw = SX126X_LORA_BW_62_5;
      break;
    case 1250:
      _bw = SX126X_LORA_BW_125_0;
      break;
    case 2500:
      _bw = SX126X_LORA_BW_250_0;
      break;
    case 5000:
      _bw = SX126X_LORA_BW_500_0;
      break;
    default:
      return(ERR_INVALID_BANDWIDTH);
      break;
  }
  _bwkHz_x10 = bwkHz_x10;

  // update modulation parameters
  return(setModulationParams(_sf, _bw, _cr));
}

int16_t SX126x::setSpreadingFactor(uint8_t sf) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_LORA) {
    return(ERR_WRONG_MODEM);
  }

  // check allowed spreading factor values
  if(!((sf >= 5) && (sf <= 12))) {
    return(ERR_INVALID_SPREADING_FACTOR);
  }

  // update modulation parameters
  _sf = sf;
  return(setModulationParams(_sf, _bw, _cr));
}

int16_t SX126x::setCodingRate(uint8_t cr) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_LORA) {
    return(ERR_WRONG_MODEM);
  }

  // check allowed spreading factor values
  if(!((cr >= 5) && (cr <= 8))) {
    return(ERR_INVALID_CODING_RATE);
  }

  // update modulation parameters
  _cr = cr - 4;
  return(setModulationParams(_sf, _bw, _cr));
}

int16_t SX126x::setSyncWord(uint16_t syncWord) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_LORA) {
    return(ERR_WRONG_MODEM);
  }

  // update register
  uint8_t data[2] = {(uint8_t)((syncWord >> 8) & 0xFF), (uint8_t)(syncWord & 0xFF)};
  return(writeRegister(SX126X_REG_LORA_SYNC_WORD_MSB, data, 2));
}

int16_t SX126x::setCurrentLimit(float currentLimit) {
  // ensure conversion to byte doesn't overflow.
  // this isn't checking that the limit is in a valid range for the device.
  if (!(currentLimit < 0 && currentLimit < 400)) {
    return(ERR_INVALID_CURRENT_LIMIT);
  }
  // calculate raw value
  uint8_t rawLimit = (uint8_t)(currentLimit / 2.5);
  return(setCurrentLimit_i(rawLimit));
}

int16_t SX126x::setCurrentLimit_i(uint8_t limit_div2_5) {
  // update register
  return(writeRegister(SX126X_REG_OCP_CONFIGURATION, &limit_div2_5, 1));
}

int16_t SX126x::setPreambleLength(uint16_t preambleLength) {
  uint8_t modem = getPacketType();
  if(modem == SX126X_PACKET_TYPE_LORA) {
    _preambleLength = preambleLength;
    return(setPacketParams(_preambleLength, _crcType));
  } else if(modem == SX126X_PACKET_TYPE_GFSK) {
    _preambleLengthFSK = preambleLength;
    return(setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, _packetType));
  }

  return(ERR_UNKNOWN);
}

int16_t SX126x::setFrequencyDeviation(float freqDev) {
  // check alowed frequency deviation values
  if (!(0 < freqDev  && freqDev < 200.0)) {
    return(ERR_INVALID_FREQUENCY_DEVIATION);
  }

  uint32_t freqDev_Hz = freqDev * 1000;
  return setFrequencyDeviation_i(freqDev_Hz);
}

int16_t SX126x::setFrequencyDeviation_i(uint32_t freqDev_Hz) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // calculate raw frequency deviation value
  uint32_t freqDevRaw = getRfFreq(freqDev_Hz);

  // check modulation parameters
  /*if(2 * freqDevRaw + _br > _rxBwKhz * 1000.0) {
    return(ERR_INVALID_MODULATION_PARAMETERS);
  }*/
  _freqDev = freqDevRaw;

  // update modulation parameters
  return(setModulationParamsFSK(_br, _pulseShape, _rxBw, _freqDev));
}

int16_t SX126x::setBitRate(float br) {
  // check alowed bit rate values
  if(!((br >= 0.6) && (br <= 300.0))) {
    return(ERR_INVALID_BIT_RATE);
  }

  return(setBitRate_i((uint32_t)(br * kilo + 0.5)));
}

int16_t SX126x::setBitRate_i(uint32_t br_bps)
{
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // calculate raw bit rate value
  uint32_t brRaw = (32 * SX126X_CRYSTAL_FREQ * MHz) / br_bps;

  // check modulation parameters
  /*if(2 * _freqDev + brRaw > _rxBwKhz * 1000.0) {
    return(ERR_INVALID_MODULATION_PARAMETERS);
  }*/
  _br = brRaw;

  // update modulation parameters
  return(setModulationParamsFSK(_br, _pulseShape, _rxBw, _freqDev));
}

int16_t SX126x::setRxBandwidth(float rxBw) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // ensure we don't overflow converting to integer:
  if (!(0 < rxBw && rxBw < 500)) {
    return(ERR_INVALID_RX_BANDWIDTH);
  }

  uint8_t rxBwi = (uint8_t)(rxBw * 10 + 0.5);
  return setRxBandwidth_i(rxBwi);
}

int16_t SX126x::setRxBandwidth_i(uint16_t rxBw_kHz_x10)
{
  // check modulation parameters
  /*if(2 * _freqDev + _br > rxBw * 1000.0) {
    return(ERR_INVALID_MODULATION_PARAMETERS);
  }*/

  // check alowed receiver bandwidth values
  // (the compiler should optimise out these constant expressions involving FP)
  switch (rxBw_kHz_x10) {
  case 48:
      _rxBw = SX126X_GFSK_RX_BW_4_8;
      break;
  case 58:
      _rxBw = SX126X_GFSK_RX_BW_5_8;
      break;
  case 73:
      _rxBw = SX126X_GFSK_RX_BW_7_3;
      break;
  case 97:
      _rxBw = SX126X_GFSK_RX_BW_9_7;
      break;
  case 117:
      _rxBw = SX126X_GFSK_RX_BW_11_7;
      break;
    case 146:
      _rxBw = SX126X_GFSK_RX_BW_14_6;
      break;
    case 195:
      _rxBw = SX126X_GFSK_RX_BW_19_5;
      break;
    case 234:
      _rxBw = SX126X_GFSK_RX_BW_23_4;
      break;
    case 293:
      _rxBw = SX126X_GFSK_RX_BW_29_3;
      break;
    case 390:
      _rxBw = SX126X_GFSK_RX_BW_39_0;
      break;
    case 469:
      _rxBw = SX126X_GFSK_RX_BW_46_9;
      break;
    case 586:
      _rxBw = SX126X_GFSK_RX_BW_58_6;
      break;
    case 782:
      _rxBw = SX126X_GFSK_RX_BW_78_2;
      break;
    case 938:
      _rxBw = SX126X_GFSK_RX_BW_93_8;
      break;
    case 1173:
      _rxBw = SX126X_GFSK_RX_BW_117_3;
      break;
    case 1562:
      _rxBw = SX126X_GFSK_RX_BW_156_2;
      break;
    case 1872:
      _rxBw = SX126X_GFSK_RX_BW_187_2;
      break;
    case 2343:
      _rxBw = SX126X_GFSK_RX_BW_234_3;
      break;
    case 3120:
      _rxBw = SX126X_GFSK_RX_BW_312_0;
      break;
    case 3736:
      _rxBw = SX126X_GFSK_RX_BW_373_6;
      break;
    case 4670:
      _rxBw = SX126X_GFSK_RX_BW_467_0;
      break;
    default:
      return(ERR_INVALID_RX_BANDWIDTH);
      break;
  }

  // set member variable only after we ensure it is valid.
  _rxBwKhz_x10 = rxBw_kHz_x10;

  // update modulation parameters
  return(setModulationParamsFSK(_br, _pulseShape, _rxBw, _freqDev));
}

int16_t SX126x::setDataShaping(float sh) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // ensure we don't overflow converting to byte
  if (!(0 <= sh && sh <= 20)) {
    return(ERR_INVALID_DATA_SHAPING);
  }

  return(setDataShaping_i((uint8_t)(sh * 10 + 0.5)));
}

int16_t SX126x::setDataShaping_i(uint8_t sh_x10) {
  // check allowed values
  switch (sh_x10)
  {
    case 0:
      _pulseShape = SX126X_GFSK_FILTER_NONE;
      break;
    case 3:
      _pulseShape = SX126X_GFSK_FILTER_GAUSS_0_3;
      break;
    case 5:
      _pulseShape = SX126X_GFSK_FILTER_GAUSS_0_5;
      break;
    case 7:
      _pulseShape = SX126X_GFSK_FILTER_GAUSS_0_7;
      break;
    case 10:
      _pulseShape = SX126X_GFSK_FILTER_GAUSS_1;
      break;
    default:
      return(ERR_INVALID_DATA_SHAPING);
  }

  // update modulation parameters
  return(setModulationParamsFSK(_br, _pulseShape, _rxBw, _freqDev));
}

int16_t SX126x::setSyncWord(uint8_t* syncWord, uint8_t len) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // check sync word Length
  if(len > 8) {
    return(ERR_INVALID_SYNC_WORD);
  }

  // write sync word
  int16_t state = writeRegister(SX126X_REG_SYNC_WORD_0, syncWord, len);
  if(state != ERR_NONE) {
    return(state);
  }

  // update packet parameters
  _syncWordLength = len * 8;
  state = setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, _packetType);

  return(state);
}

int16_t SX126x::setSyncBits(uint8_t *syncWord, uint8_t bitsLen) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // check sync word Length
  if(bitsLen > 0x40) {
    return(ERR_INVALID_SYNC_WORD);
  }

  uint8_t bytesLen = bitsLen / 8;
  if ((bitsLen % 8) != 0) {
    bytesLen++;
  }

  // write sync word
  int16_t state = writeRegister(SX126X_REG_SYNC_WORD_0, syncWord, bytesLen);
  if(state != ERR_NONE) {
    return(state);
  }

  // update packet parameters
  _syncWordLength = bitsLen;
  state = setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, _packetType);

  return(state);
}

int16_t SX126x::setNodeAddress(uint8_t nodeAddr) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // enable address filtering (node only)
  _addrComp = SX126X_GFSK_ADDRESS_FILT_NODE;
  int16_t state = setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, _packetType);
  if(state != ERR_NONE) {
    return(state);
  }

  // set node address
  state = writeRegister(SX126X_REG_NODE_ADDRESS, &nodeAddr, 1);

  return(state);
}

int16_t SX126x::setBroadcastAddress(uint8_t broadAddr) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // enable address filtering (node and broadcast)
  _addrComp = SX126X_GFSK_ADDRESS_FILT_NODE_BROADCAST;
  int16_t state = setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, _packetType);
  if(state != ERR_NONE) {
    return(state);
  }

  // set broadcast address
  state = writeRegister(SX126X_REG_BROADCAST_ADDRESS, &broadAddr, 1);

  return(state);
}

int16_t SX126x::disableAddressFiltering() {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // disable address filtering
  _addrComp = SX126X_GFSK_ADDRESS_FILT_OFF;
  return(setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening));
}

int16_t SX126x::setCRC(uint8_t len, uint16_t initial, uint16_t polynomial, bool inverted) {
  // check active modem
  uint8_t modem = getPacketType();

  if(modem == SX126X_PACKET_TYPE_GFSK) {
    // update packet parameters
    switch(len) {
      case 0:
        _crcTypeFSK = SX126X_GFSK_CRC_OFF;
        break;
      case 1:
        if(inverted) {
          _crcTypeFSK = SX126X_GFSK_CRC_1_BYTE_INV;
        } else {
          _crcTypeFSK = SX126X_GFSK_CRC_1_BYTE;
        }
        break;
      case 2:
        if(inverted) {
          _crcTypeFSK = SX126X_GFSK_CRC_2_BYTE_INV;
        } else {
          _crcTypeFSK = SX126X_GFSK_CRC_2_BYTE;
        }
        break;
      default:
        return(ERR_INVALID_CRC_CONFIGURATION);
    }

    int16_t state = setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, _packetType);
    if(state != ERR_NONE) {
      return(state);
    }

    // write initial CRC value
    uint8_t data[2] = {(uint8_t)((initial >> 8) & 0xFF), (uint8_t)(initial & 0xFF)};
    state = writeRegister(SX126X_REG_CRC_INITIAL_MSB, data, 2);
    if(state != ERR_NONE) {
      return(state);
    }

    // write CRC polynomial value
    data[0] = (uint8_t)((polynomial >> 8) & 0xFF);
    data[1] = (uint8_t)(polynomial & 0xFF);
    state = writeRegister(SX126X_REG_CRC_POLYNOMIAL_MSB, data, 2);

    return(state);

  } else if(modem == SX126X_PACKET_TYPE_LORA) {
    // LoRa CRC doesn't allow to set CRC polynomial, inital value, or inversion

    // update packet parameters
    if(len) {
      _crcType = SX126X_LORA_CRC_ON;
    } else {
      _crcType = SX126X_LORA_CRC_OFF;
    }

    return(setPacketParams(_preambleLength, _crcType));
  }

  return(ERR_UNKNOWN);
}

int16_t SX126x::setWhitening(bool enabled, uint16_t initial) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  int16_t state = ERR_NONE;
  if(enabled != true) {
    // disable whitening
    _whitening = SX126X_GFSK_WHITENING_OFF;

    state = setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, _packetType);
    if(state != ERR_NONE) {
      return(state);
    }
  } else {
    // enable whitening
    _whitening = SX126X_GFSK_WHITENING_ON;

    // write initial whitening value
    // as per note on pg. 65 of datasheet v1.2: "The user should not change the value of the 7 MSB's of this register"
    uint8_t data[2];
    // first read the actual value and mask 7 MSB which we can not change
    // if different value is written in 7 MSB, the Rx won't even work (tested on HW)
    state = readRegister(SX126X_REG_WHITENING_INITIAL_MSB, data, 1);
    if(state != ERR_NONE) {
      return(state);
    }
    data[0] = (data[0] & 0xFE) | (uint8_t)((initial >> 8) & 0x01);
    data[1] = (uint8_t)(initial & 0xFF);
    state = writeRegister(SX126X_REG_WHITENING_INITIAL_MSB, data, 2);

    if(state != ERR_NONE) {
      return(state);
    }

    state = setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, _packetType);
    if(state != ERR_NONE) {
      return(state);
    }
  } 
  return(state);
}

uint16_t SX126x::getDataRate() {
  return(_dataRate);
}


float SX126x::getRSSI() {
  // get last packet RSSI from packet status
  uint32_t packetStatus = getPacketStatus();
  uint8_t rssiPkt = packetStatus & 0xFF;
  return(-1.0 * rssiPkt/2.0);
}

float SX126x::getSNR() {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_LORA) {
    return(ERR_WRONG_MODEM);
  }

  // get last packet SNR from packet status
  uint32_t packetStatus = getPacketStatus();
  uint8_t snrPkt = (packetStatus >> 8) & 0xFF;
  return(snrPkt/4.0);
}

int16_t SX126x::getBufferStatus(uint8_t* packetLen, uint8_t* startAddr) {
  uint8_t rxBufStatus[2];
  int16_t state = SPIreadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, rxBufStatus, 2);
  if (state != ERR_NONE) {
    return(state);
  }
  if (packetLen) {
    *packetLen = rxBufStatus[0];
  }
  if (startAddr) {
    *startAddr = rxBufStatus[1];
  }
  return(state);
}

size_t SX126x::getPacketLength(bool update) {
  (void)update;
  uint8_t packetLen = 0;
  int16_t state = getBufferStatus(&packetLen, NULL);
  if (state != ERR_NONE) {
    return(0);
  }
  return(packetLen);
}

int16_t SX126x::fixedPacketLengthMode(uint8_t len) {
  return(setPacketMode(SX126X_GFSK_PACKET_FIXED, len));
}

int16_t SX126x::variablePacketLengthMode(uint8_t maxLen) {
  return(setPacketMode(SX126X_GFSK_PACKET_VARIABLE, maxLen));
}


uint32_t SX126x::getTimeOnAir(size_t len) {
  // everything is in microseconds to allow integer arithmetic
  // some constants have .25, these are multiplied by 4, and have _x4 postfix to indicate that fact
  if(getPacketType() == SX126X_PACKET_TYPE_LORA) {
    uint32_t symbolLength_us = ((10 * usPerSecond / kilo) << _sf) / _bwkHz_x10 ;
    uint8_t sfCoeff1_x4 = 17; // (4.25 * 4)
    uint8_t sfCoeff2 = 8;
    if(_sf == 5 || _sf == 6) {
      sfCoeff1_x4 = 25; // 6.25 * 4
      sfCoeff2 = 0;
    }
    uint8_t sfDivisor = 4*_sf;
    if(symbolLength_us >= 16000) {
      sfDivisor = 4*(_sf - 2);
    }
    const int8_t bitsPerCrc = 16;
    const int8_t N_symbol_header = 20;

    // numerator of equation in section 6.1.4 of SX1268 datasheet v1.1 (might not actually be bitcount, but it has len * 8)
    int16_t bitCount = (int16_t) 8 * len + _crcType * bitsPerCrc - 4 * _sf  + sfCoeff2 + N_symbol_header;
    if(bitCount < 0) {
      bitCount = 0;
    }
    // add (sfDivisor) - 1 to the numerator to give integer CEIL(...)
    uint16_t nPreCodedSymbols = (bitCount + (sfDivisor - 1)) / (sfDivisor);

    // preamble can be 65k, therefore nSymbol_x4 needs to be 32 bit
    uint32_t nSymbol_x4 = (_preambleLength + 8) * 4 + sfCoeff1_x4 + nPreCodedSymbols * (_cr + 4) * 4;

    return((symbolLength_us * nSymbol_x4) / 4);
  } else {
    return((len * 8 * _br) / (SX126X_CRYSTAL_FREQ * 32));
  }
}

int16_t SX126x::setTCXO(float voltage, uint32_t delay) {
  //Ensure we don't overflow converting to integer:
  if (!(0 < voltage && voltage < 4)) {
    return(ERR_INVALID_TCXO_VOLTAGE);
  }
  return(setTCXO_i((uint8_t)(voltage * 10 + 0.5), delay));
}

int16_t SX126x::setTCXO_i(uint8_t voltage_x10, uint32_t delay_us)
{
  // set mode to standby
  standby(SX126X_STANDBY_RC);

  // check SX126X_XOSC_START_ERR flag and clear it
  if(getDeviceErrors() & SX126X_XOSC_START_ERR) {
    clearDeviceErrors();
  }

  if (!(0 < voltage_x10 && voltage_x10 < 40)) {
    return(ERR_INVALID_TCXO_VOLTAGE);
  }

  // check alowed voltage values
  uint8_t data[4];
  switch (voltage_x10)
  {
    case 16:
      data[0] = SX126X_DIO3_OUTPUT_1_6;
      break;
    case 17:
      data[0] = SX126X_DIO3_OUTPUT_1_7;
      break;
    case 18:
      data[0] = SX126X_DIO3_OUTPUT_1_8;
      break;
    case 22:
      data[0] = SX126X_DIO3_OUTPUT_2_2;
      break;
    case 24:
      data[0] = SX126X_DIO3_OUTPUT_2_4;
      break;
    case 27:
      data[0] = SX126X_DIO3_OUTPUT_2_7;
      break;
    case 30:
      data[0] = SX126X_DIO3_OUTPUT_3_0;
      break;
    case 33:
      data[0] = SX126X_DIO3_OUTPUT_3_3;
      break;
    default:
      return(ERR_INVALID_TCXO_VOLTAGE);
  }

  // calculate delay
  uint32_t delayValue = delay_us * 8 / 125; // equivalent to "/ 15.625"
  data[1] = (uint8_t)((delayValue >> 16) & 0xFF);
  data[2] = (uint8_t)((delayValue >> 8) & 0xFF);
  data[3] = (uint8_t)(delayValue & 0xFF);

  _tcxoDelay = delay_us;

  // enable TCXO control on DIO3
  return(SPIwriteCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, data, 4));
}

int16_t SX126x::setDio2AsRfSwitch(bool enable) {
  uint8_t data = 0;
  if(enable) {
    data = SX126X_DIO2_AS_RF_SWITCH;
  } else {
    data = SX126X_DIO2_AS_IRQ;
  }
  return(SPIwriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1));
}

int16_t SX126x::setFrequency_i(uint32_t freq_Hz, bool calibrate) {
  int16_t state = ERR_NONE;

  // calibrate image
  if(calibrate) {
    uint8_t data[2];
    if(freq_Hz > 900 * MHz) {
      data[0] = SX126X_CAL_IMG_902_MHZ_1;
      data[1] = SX126X_CAL_IMG_902_MHZ_2;
    } else if(freq_Hz > 850 * MHz) {
      data[0] = SX126X_CAL_IMG_863_MHZ_1;
      data[1] = SX126X_CAL_IMG_863_MHZ_2;
    } else if(freq_Hz > 770 * MHz) {
      data[0] = SX126X_CAL_IMG_779_MHZ_1;
      data[1] = SX126X_CAL_IMG_779_MHZ_2;
    } else if(freq_Hz > 460 * MHz) {
      data[0] = SX126X_CAL_IMG_470_MHZ_1;
      data[1] = SX126X_CAL_IMG_470_MHZ_2;
    } else {
      data[0] = SX126X_CAL_IMG_430_MHZ_1;
      data[1] = SX126X_CAL_IMG_430_MHZ_2;
    }
    state = SX126x::calibrateImage(data);
    if(state != ERR_NONE) {
      return(state);
    }
  }

  // set frequency
  return(SX126x::setFrequencyRaw(freq_Hz));
}


int16_t SX126x::setTx(uint32_t timeout) {
  uint8_t data[3] = {(uint8_t)((timeout >> 16) & 0xFF), (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF)};
  int16_t state = SPIwriteCommand(SX126X_CMD_SET_TX, data, 3);
  if (state == ERR_NONE) {
    _curStatus = SX126X_STATUS_MODE_TX;
  }
  return(state);
}

int16_t SX126x::setRx(uint32_t timeout) {
  uint8_t data[3] = {(uint8_t)((timeout >> 16) & 0xFF), (uint8_t)((timeout >> 8) & 0xFF), (uint8_t)(timeout & 0xFF)};
  int16_t state = SPIwriteCommand(SX126X_CMD_SET_RX, data, 3);
  if (state == ERR_NONE) {
    _curStatus = SX126X_STATUS_MODE_RX;
  }
  return(state);
}

int16_t SX126x::setCad() {
  int16_t state = SPIwriteCommand(SX126X_CMD_SET_CAD, NULL, 0);
  if (state == ERR_NONE) {
    _curStatus = 0; // technically we're in Rx, but we don't want to trigger the RX callbacks
  }
  return(state);
}

int16_t SX126x::setPaConfig(uint8_t paDutyCycle, uint8_t deviceSel, uint8_t hpMax, uint8_t paLut) {
  uint8_t data[4] = {paDutyCycle, hpMax, deviceSel, paLut};
  return(SPIwriteCommand(SX126X_CMD_SET_PA_CONFIG, data, 4));
}

int16_t SX126x::writeRegister(uint16_t addr, uint8_t* data, uint8_t numBytes) {
  uint8_t cmd[] = { SX126X_CMD_WRITE_REGISTER, (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF) };
  int16_t state = SPIwriteCommand(cmd, 3, data, numBytes);
  return(state);
}

int16_t SX126x::readRegister(uint16_t addr, uint8_t* data, uint8_t numBytes) {
  uint8_t cmd[] = {SX126X_CMD_READ_REGISTER, (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF)};
  return(SX126x::SPItransfer(cmd, 3, false, NULL, data, numBytes, true));
}

int16_t SX126x::writeBuffer(uint8_t* data, uint8_t numBytes, uint8_t offset) {
  uint8_t cmd[] = { SX126X_CMD_WRITE_BUFFER, offset };
  int16_t state = SPIwriteCommand(cmd, 2, data, numBytes);

  return(state);
}

int16_t SX126x::readBuffer(uint8_t* data, uint8_t numBytes, uint8_t offset) {
  uint8_t cmd[] = { SX126X_CMD_READ_BUFFER, offset };
  int16_t state = SPIreadCommand(cmd, 2, data, numBytes);
  return(state);
}

int16_t SX126x::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
  uint8_t data[8] = {(uint8_t)((irqMask >> 8) & 0xFF), (uint8_t)(irqMask & 0xFF),
                     (uint8_t)((dio1Mask >> 8) & 0xFF), (uint8_t)(dio1Mask & 0xFF),
                     (uint8_t)((dio2Mask >> 8) & 0xFF), (uint8_t)(dio2Mask & 0xFF),
                     (uint8_t)((dio3Mask >> 8) & 0xFF), (uint8_t)(dio3Mask & 0xFF)};
  return(SPIwriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, data, 8));
}

int16_t SX126x::getIrqStatus(uint16_t* status) {
  uint8_t data[2];
  int16_t state = SPIreadCommand(SX126X_CMD_GET_IRQ_STATUS, data, 2);
  *status = ((uint16_t)(data[0]) << 8) | data[1];
  return state;
}

int16_t SX126x::clearIrqStatus(uint16_t clearIrqParams) {
  uint8_t data[2] = {(uint8_t)((clearIrqParams >> 8) & 0xFF), (uint8_t)(clearIrqParams & 0xFF)};
  return(SPIwriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, data, 2));
}

int16_t SX126x::setRfFrequency(uint32_t frf) {
  uint8_t data[4] = {(uint8_t)((frf >> 24) & 0xFF), (uint8_t)((frf >> 16) & 0xFF), (uint8_t)((frf >> 8) & 0xFF), (uint8_t)(frf & 0xFF)};
  return(SPIwriteCommand(SX126X_CMD_SET_RF_FREQUENCY, data, 4));
}

int16_t SX126x::calibrateImage(uint8_t* data) {
  return(SPIwriteCommand(SX126X_CMD_CALIBRATE_IMAGE, data, 2));
}

uint8_t SX126x::getPacketType() {
  //We only use LoRa 
  return SX126X_PACKET_TYPE_LORA;

  uint8_t data = 0xFF;
  SPIreadCommand(SX126X_CMD_GET_PACKET_TYPE, &data, 1);
  return(data);
}

int16_t SX126x::setTxParams(uint8_t power, uint8_t rampTime) {
  uint8_t data[2] = {power, rampTime};
  return(SPIwriteCommand(SX126X_CMD_SET_TX_PARAMS, data, 2));
}

int16_t SX126x::setOptimalHiPowerPaConfig(int8_t * inOutPower) {
  // set PA config for optimal consumption as described in section 13-21 of SX1268 datasheet v1.1
  // the final column of Table 13-21 suggests that the value passed in SetTxParams
  // is actually scaled depending on the parameters of setPaConfig
  int16_t state;
  if (*inOutPower >= 21) {
    state = SX126x::setPaConfig(0x04, SX126X_PA_CONFIG_SX1262_8, SX126X_PA_CONFIG_HP_MAX/*0x07*/);
  }
  else if (*inOutPower >= 18) {
    state = SX126x::setPaConfig(0x03, SX126X_PA_CONFIG_SX1262_8, 0x05);
    // datasheet instructs request 22 dBm for 20 dBm actual output power
    *inOutPower += 2;
  } else if (*inOutPower >= 15) {
    state = SX126x::setPaConfig(0x02, SX126X_PA_CONFIG_SX1262_8, 0x03);
    // datasheet instructs request 22 dBm for 17 dBm actual output power
    *inOutPower += 5;
  } else {
    state = SX126x::setPaConfig(0x02, SX126X_PA_CONFIG_SX1262_8, 0x02);
    // datasheet instructs request 22 dBm for 14 dBm actual output power.
    *inOutPower += 8;
  }
  return state;
}

int16_t SX126x::setPacketMode(uint8_t mode, uint8_t len) {
  // check active modem
  if(getPacketType() != SX126X_PACKET_TYPE_GFSK) {
    return(ERR_WRONG_MODEM);
  }

  // set requested packet mode
  int16_t state = setPacketParamsFSK(_preambleLengthFSK, _crcTypeFSK, _syncWordLength, _addrComp, _whitening, mode, len);
  if(state != ERR_NONE) {
    return(state);
  }

  // update cached value
  _packetType = mode;
  return(state);
}

int16_t SX126x::setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro) {
  // calculate symbol length and enable low data rate optimization, if needed
  if(ldro == 0xFF) {
    uint32_t symbolLength_us = ((10 * usPerSecond / kilo) << _sf) / _bwkHz_x10;
    RADIOLIB_DEBUG_PRINT(F("Symbol length: "));
    RADIOLIB_DEBUG_PRINT(symbolLength_us);
    RADIOLIB_DEBUG_PRINTLN(F(" \xe6s")); //microseconds
    if(symbolLength_us >= 16000) {
      _ldro = SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_ON;
    } else {
      _ldro = SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF;
    }
  } else {
    _ldro = ldro;
  }

  uint8_t data[4] = {sf, bw, cr, _ldro};
  return(SPIwriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 4));
}

int16_t SX126x::setModulationParamsFSK(uint32_t br, uint8_t pulseShape, uint8_t rxBw, uint32_t freqDev) {
  uint8_t data[8] = {(uint8_t)((br >> 16) & 0xFF), (uint8_t)((br >> 8) & 0xFF), (uint8_t)(br & 0xFF),
                     pulseShape, rxBw,
                     (uint8_t)((freqDev >> 16) & 0xFF), (uint8_t)((freqDev >> 8) & 0xFF), (uint8_t)(freqDev & 0xFF)};
  return(SPIwriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 8));
}

int16_t SX126x::setPacketParams(uint16_t preambleLength, uint8_t crcType, uint8_t payloadLength, uint8_t headerType, uint8_t invertIQ) {
  fixInvertedIQ(invertIQ);
  uint8_t data[6] = {(uint8_t)((preambleLength >> 8) & 0xFF), (uint8_t)(preambleLength & 0xFF), headerType, payloadLength, crcType, invertIQ};
  return(SPIwriteCommand(SX126X_CMD_SET_PACKET_PARAMS, data, 6));
}

int16_t SX126x::setPacketParamsFSK(uint16_t preambleLength, uint8_t crcType, uint8_t syncWordLength, uint8_t addrComp, uint8_t whitening, uint8_t packetType, uint8_t payloadLength, uint8_t preambleDetectorLength) {
  uint8_t data[9] = {(uint8_t)((preambleLength >> 8) & 0xFF), (uint8_t)(preambleLength & 0xFF),
                     preambleDetectorLength, syncWordLength, addrComp,
                     packetType, payloadLength, crcType, whitening};
  return(SPIwriteCommand(SX126X_CMD_SET_PACKET_PARAMS, data, 9));
}

int16_t SX126x::setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
  uint8_t data[2] = {txBaseAddress, rxBaseAddress};
  return(SPIwriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, data, 2));
}

int16_t SX126x::getStatus(uint8_t* status) {
  uint8_t cmd = SX126X_CMD_GET_STATUS;
  return SX126x::SPItransfer(&cmd, 1, false, NULL, status, 1, true, 5000, false);
}

uint32_t SX126x::getPacketStatus() {
  uint8_t data[3] = {0, 0, 0};
  SPIreadCommand(SX126X_CMD_GET_PACKET_STATUS, data, 3);
  return((((uint32_t)data[0]) << 16) | (((uint32_t)data[1]) << 8) | (uint32_t)data[2]);
}

uint16_t SX126x::getDeviceErrors() {
  uint8_t data[2] = {0, 0};
  SPIreadCommand(SX126X_CMD_GET_DEVICE_ERRORS, data, 2);
  uint16_t opError = (((uint16_t)data[0] & 0xFF) << 8) & ((uint16_t)data[1]);
  return(opError);
}

int16_t SX126x::clearDeviceErrors() {
  uint8_t data[2] = {SX126X_CMD_NOP, SX126X_CMD_NOP};
  return(SPIwriteCommand(SX126X_CMD_CLEAR_DEVICE_ERRORS, data, 2));
}

int16_t SX126x::setFrequencyRaw(uint32_t freq_Hz) {
  // calculate raw value
  uint32_t frf = getRfFreq(freq_Hz);
  setRfFrequency(frf);
  return(ERR_NONE);
}

int16_t SX126x::fixSensitivity() {
  // fix receiver sensitivity for 500 kHz LoRa
  // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.1 for details

  // read current sensitivity configuration
  uint8_t sensitivityConfig = 0;
  int16_t state = readRegister(SX126X_REG_SENSITIVITY_CONFIG, &sensitivityConfig, 1);
  if(state != ERR_NONE) {
    return(state);
  }

  // fix the value for LoRa with 500 kHz bandwidth
  if((getPacketType() == SX126X_PACKET_TYPE_LORA) && (_rxBwKhz_x10 == 5000)) {
    sensitivityConfig &= 0xFB;
  } else {
    sensitivityConfig |= 0x04;
  }
  return(writeRegister(SX126X_REG_SENSITIVITY_CONFIG, &sensitivityConfig, 1));
}

int16_t SX126x::fixPaClamping() {
  // fixes overly eager PA clamping
  // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.2 for details

  // read current clamping configuration
  uint8_t clampConfig = 0;
  int16_t state = readRegister(SX126X_REG_TX_CLAMP_CONFIG, &clampConfig, 1);
  if (state != ERR_NONE) {
    return state;
  }

  // update with the new value
  clampConfig |= 0x1E;
  return(writeRegister(SX126X_REG_TX_CLAMP_CONFIG, &clampConfig, 1));
}

int16_t SX126x::fixImplicitTimeout() {
  // fixes timeout in implicit header mode
  // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.3 for details

  // stop RTC counter
  uint8_t rtcStop = 0x00;
  int16_t state = writeRegister(SX126X_REG_RTC_STOP, &rtcStop, 1);
  if(state != ERR_NONE) {
    return(state);
  }

  // read currently active event
  uint8_t rtcEvent = 0;
  state = readRegister(SX126X_REG_RTC_EVENT, &rtcEvent, 1);
  if(state != ERR_NONE) {
    return(state);
  }

  // clear events
  rtcEvent |= 0x02;
  return(writeRegister(SX126X_REG_RTC_EVENT, &rtcEvent, 1));
}

int16_t SX126x::fixInvertedIQ(uint8_t iqConfig) {
  return ERR_NONE;
  // fixes IQ configuration for inverted IQ
  // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for details

  // read current IQ configuration
  uint8_t iqConfigCurrent = 0;
  int16_t state = readRegister(SX126X_REG_IQ_CONFIG, &iqConfigCurrent, 1);
  if(state != ERR_NONE) {
    return(state);
  }

  // set correct IQ configuration
  if(iqConfig == SX126X_LORA_IQ_STANDARD) {
    iqConfigCurrent &= 0xFB;
  } else {
    iqConfigCurrent |= 0x04;
  }

  // update with the new value
  return(writeRegister(SX126X_REG_IQ_CONFIG, &iqConfigCurrent, 1));
}

int16_t SX126x::config(uint8_t modem) {
  // set regulator mode
  uint8_t data[7];
  data[0] = SX126X_REGULATOR_DC_DC;
  int16_t state = SPIwriteCommand(SX126X_CMD_SET_REGULATOR_MODE, data, 1);
  if(state != ERR_NONE) {
    return(state);
  }

  // reset buffer base address
  state = setBufferBaseAddress();
  if(state != ERR_NONE) {
    return(state);
  }

  // set modem
  data[0] = modem;
  state = SPIwriteCommand(SX126X_CMD_SET_PACKET_TYPE, data, 1);
  if(state != ERR_NONE) {
    return(state);
  }

  // set Rx/Tx fallback mode to FS. This allows quicker switching between modes with a TCXO.
  data[0] = SX126X_RX_TX_FALLBACK_MODE_FS;
  state = SPIwriteCommand(SX126X_CMD_SET_RX_TX_FALLBACK_MODE, data, 1);
  if(state != ERR_NONE) {
    return(state);
  }

  // set CAD parameters
  data[0] = SX126X_CAD_ON_8_SYMB;
  data[1] = _sf + 13;
  data[2] = 10;
  data[3] = SX126X_CAD_GOTO_STDBY;
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  state = SPIwriteCommand(SX126X_CMD_SET_CAD_PARAMS, data, 7);
  if(state != ERR_NONE) {
    return(state);
  }

  // clear IRQ
  state = clearIrqStatus();
  state |= setDioIrqParams(SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  if(state != ERR_NONE) {
    return(state);
  }

  // calibrate all blocks
  data[0] = SX126X_CALIBRATE_ALL;
  state = SPIwriteCommand(SX126X_CMD_CALIBRATE, data, 1);
  if(state != ERR_NONE) {
    return(state);
  }

  // wait for calibration completion
  delay(5);
  while(digitalRead(_mod->getInt1()));

  return(ERR_NONE);
}

int16_t SX126x::SPIwriteCommand(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  return(SX126x::SPItransfer(cmd, cmdLen, true, data, NULL, numBytes, waitForBusy));
}

int16_t SX126x::SPIwriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  return(SX126x::SPItransfer(&cmd, 1, true, data, NULL, numBytes, waitForBusy));
}

int16_t SX126x::SPIreadCommand(uint8_t* cmd, uint8_t cmdLen, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  return(SX126x::SPItransfer(cmd, cmdLen, false, NULL, data, numBytes, waitForBusy));
}

int16_t SX126x::SPIreadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  return(SX126x::SPItransfer(&cmd, 1, false, NULL, data, numBytes, waitForBusy));
}

int16_t SX126x::SPItransfer(uint8_t* cmd, uint8_t cmdLen, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy, uint32_t timeout,
  bool rdSkipFirstByte) {
  // get pointer to used SPI interface and the settings
  SPIClass* spi = _mod->getSpi();
  SPISettings spiSettings = _mod->getSpiSettings();

  #ifdef RADIOLIB_VERBOSE
    uint8_t debugBuff[256];
  #endif

  // pull NSS low
  digitalWrite(_mod->getCs(), LOW);

  // ensure BUSY is low (state meachine ready)
  RADIOLIB_VERBOSE_PRINTLN(F("Wait for BUSY ... "));
  uint32_t start = millis();
  while(digitalRead(_mod->getInt1())) {
    if(_bailIfBusy || (millis() - start >= timeout)) {
      return(ERR_SPI_CMD_TIMEOUT);
    }
  }

  // start transfer
  spi->beginTransaction(spiSettings);

  // send command byte(s)
  for(uint8_t n = 0; n < cmdLen; n++) {
    spi->transfer(cmd[n]);
  }

  // variable to save error during SPI transfer
  uint8_t status = 0;

  // send/receive all bytes
  if(write) {
    for(uint8_t n = 0; n < numBytes; n++) {
      // send byte
      uint8_t in = spi->transfer(dataOut[n]);
      #ifdef RADIOLIB_VERBOSE
        debugBuff[n] = in;
      #endif

      // check status
      if(((in & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT) ||
         ((in & 0b00001110) == SX126X_STATUS_CMD_INVALID) ||
         ((in & 0b00001110) == SX126X_STATUS_CMD_FAILED)) {
        status = in & 0b00001110;
        break;
      } else if(in == 0x00 || in == 0xFF) {
        status = SX126X_STATUS_SPI_FAILED;
        break;
      }
    }

  } else {
    // skip the first byte for read-type commands (status-only)
    uint8_t in = spi->transfer(SX126X_CMD_NOP);
    #ifdef RADIOLIB_VERBOSE
      debugBuff[0] = in;
    #endif

    if(!rdSkipFirstByte) {
      dataIn[0] = in;
      dataIn++;
      numBytes--;
    }

    // check status
    if(((in & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT) ||
       ((in & 0b00001110) == SX126X_STATUS_CMD_INVALID) ||
       ((in & 0b00001110) == SX126X_STATUS_CMD_FAILED)) {
      status = in & 0b00001110;
    } else if(in == 0x00 || in == 0xFF) {
      status = SX126X_STATUS_SPI_FAILED;
    } else {
      for(uint8_t n = 0; n < numBytes; n++) {
        dataIn[n] = spi->transfer(SX126X_CMD_NOP);
      }
    }
  }

  // stop transfer
  spi->endTransaction();
  digitalWrite(_mod->getCs(), HIGH);

  // wait for BUSY to go high and then low
  if(waitForBusy && !_bailIfBusy) {
    delayMicroseconds(1);
    start = millis();
    while(digitalRead(_mod->getInt1())) {
        if(millis() - start >= timeout) {
          status = SX126X_STATUS_CMD_TIMEOUT;
          break;
        }
    }
  }

  // print debug output
  #ifdef RADIOLIB_VERBOSE
    // print command byte(s)
    RADIOLIB_VERBOSE_PRINT(F("CMD\t"));
    for(uint8_t n = 0; n < cmdLen; n++) {
      RADIOLIB_VERBOSE_PRINT(cmd[n], HEX);
      RADIOLIB_VERBOSE_PRINT('\t');
    }
    RADIOLIB_VERBOSE_PRINTLN();

    // print data bytes
    RADIOLIB_VERBOSE_PRINT(F("DAT"));
    if(write) {
      RADIOLIB_VERBOSE_PRINT(F("W\t"));
      for(uint8_t n = 0; n < numBytes; n++) {
        RADIOLIB_VERBOSE_PRINT(dataOut[n], HEX);
        RADIOLIB_VERBOSE_PRINT('\t');
        RADIOLIB_VERBOSE_PRINT(debugBuff[n], HEX);
        RADIOLIB_VERBOSE_PRINT('\t');
      }
      RADIOLIB_VERBOSE_PRINTLN();
    } else {
      RADIOLIB_VERBOSE_PRINT(F("R\t"));
      // skip the first byte for read-type commands (status-only)
      if (!rdSkipFirstByte)
      RADIOLIB_VERBOSE_PRINT(SX126X_CMD_NOP, HEX);
      RADIOLIB_VERBOSE_PRINT('\t');
      RADIOLIB_VERBOSE_PRINT(debugBuff[0], HEX);
      RADIOLIB_VERBOSE_PRINT('\t')

      for(uint8_t n = 0; n < numBytes; n++) {
        RADIOLIB_VERBOSE_PRINT(SX126X_CMD_NOP, HEX);
        RADIOLIB_VERBOSE_PRINT('\t');
        RADIOLIB_VERBOSE_PRINT(dataIn[n], HEX);
        RADIOLIB_VERBOSE_PRINT('\t');
      }
      RADIOLIB_VERBOSE_PRINTLN();
    }
    RADIOLIB_VERBOSE_PRINTLN();
  #endif

  // parse status
  switch(status) {
    case SX126X_STATUS_CMD_TIMEOUT:
      return(ERR_SPI_CMD_TIMEOUT);
    case SX126X_STATUS_CMD_INVALID:
      return(ERR_SPI_CMD_INVALID);
    case SX126X_STATUS_CMD_FAILED:
      return(ERR_SPI_CMD_FAILED);
    case SX126X_STATUS_SPI_FAILED:
      return(ERR_CHIP_NOT_FOUND);
    default:
      return(ERR_NONE);
  }
}
