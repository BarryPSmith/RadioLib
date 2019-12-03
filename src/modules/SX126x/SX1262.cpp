#include "SX1262.h"

SX1262::SX1262(Module* mod) : SX126x(mod) {

}

int16_t SX1262::begin(float freq, float bw, uint8_t sf, uint8_t cr, uint16_t syncWord, int8_t power, float currentLimit, uint16_t preambleLength, float tcxoVoltage) {
  return(begin_i(freq * MHz, bw * 10 + 0.5, sf, cr, syncWord, power, currentLimit / 2.5 + 0.5, preambleLength, tcxoVoltage * 10 + 0.5));
}

int16_t SX1262::begin_i(uint32_t freq_Hz, uint16_t bwkHz_x10, uint8_t sf, uint8_t cr,
      uint16_t syncWord, int8_t power, uint8_t currentLimit_mA_div2_5,
      uint16_t preambleLength, uint8_t tcxoVoltage_x10) {
  // execute common part
  int16_t state = SX126x::begin_i(bwkHz_x10, sf, cr, syncWord, currentLimit_mA_div2_5, preambleLength, tcxoVoltage_x10);
  if(state != ERR_NONE) {
    return(state);
  }

  // configure publicly accessible settings
  state = setFrequency_i(freq_Hz, true);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setOutputPower(power);
  if(state != ERR_NONE) {
    return(state);
  }

  state = SX126x::fixPaClamping();
  if (state != ERR_NONE) {
    return state;
  }

  return(state);
}

int16_t SX1262::beginFSK(float freq, float br, float freqDev, float rxBw, int8_t power, float currentLimit, uint16_t preambleLength, float dataShaping, float tcxoVoltage) {
  return(SX1262::beginFSK_i(freq * MHz, br * kilo + 0.5, freqDev * kilo + 0.5, rxBw * 10 + 0.5, power, currentLimit / 2.5 + 0.5, preambleLength, dataShaping * 10 + 0.5, tcxoVoltage * 10 + 0.5));
}

int16_t SX1262::beginFSK_i(uint32_t freq_Hz, uint32_t br_bps, uint32_t freqDev_Hz, uint16_t rxBw_kHz_x10,
      int8_t power, uint8_t currentLimit_mA_div2_5, uint16_t preambleLength,
      uint8_t dataShaping_x10, uint8_t tcxoVoltage_x10)
{
  // execute common part
  int16_t state = SX126x::beginFSK_i(br_bps, freqDev_Hz, rxBw_kHz_x10, currentLimit_mA_div2_5, preambleLength, dataShaping_x10, tcxoVoltage_x10);
  if(state != ERR_NONE) {
    return(state);
  }

  // configure publicly accessible settings
  state = SX1262::setFrequency_i(freq_Hz, true);
  if(state != ERR_NONE) {
    return(state);
  }

  state = setOutputPower(power);
  if(state != ERR_NONE) {
    return(state);
  }

  state = SX126x::fixPaClamping();
  if (state != ERR_NONE) {
    return state;
  }

  return(state);
}

int16_t SX1262::setFrequency(float freq, bool calibrate) {
  // check for overflow:
  if(!(freq > 0) && (freq < 1000)) {
    return(ERR_INVALID_FREQUENCY);
  }
  return(SX1262::setFrequency_i((uint32_t)(freq * 1000000), calibrate));
}

int16_t SX1262::setFrequency_i(uint32_t freq_Hz, bool calibrate) {
  // check for valid range:
  if (freq_Hz < 150 * MHz || freq_Hz > 960 * MHz) {
    return(ERR_INVALID_FREQUENCY);
  }
  return(SX126x::setFrequency_i(freq_Hz, calibrate));
}


int16_t SX1262::setOutputPower(int8_t power) {
  // check allowed power range
  if (!((power >= -17) && (power <= 22))) {
    return(ERR_INVALID_OUTPUT_POWER);
  }

  // get current OCP configuration
  uint8_t ocp = 0;
  int16_t state = readRegister(SX126X_REG_OCP_CONFIGURATION, &ocp, 1);
  if (state != ERR_NONE) {
    return(state);
  }

  // this function sets the optimal PA settings
  // and adjusts power based on the PA settings chosen
  // so that output power matches requested power.
  state = SX126x::setOptimalHiPowerPaConfig(&power);
  if (state != ERR_NONE) {
    return(state);
  }

  // set output power
  // TODO power ramp time configuration
  state = SX126x::setTxParams(power);
  if (state != ERR_NONE) {
    return(state);
  }

  // restore OCP configuration
  return writeRegister(SX126X_REG_OCP_CONFIGURATION, &ocp, 1);
}
