#include "JDY08.h"

JDY08::JDY08(Module* mod) : ISerial(mod) {
  
}

void JDY08::begin(long speed) {
  // set module properties
  _mod->AtLineFeed = "";
  _mod->baudrate = speed;
  _mod->initUART(RADIOLIB_INT_NONE);
}
