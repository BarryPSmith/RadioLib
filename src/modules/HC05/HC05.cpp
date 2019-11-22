#include "HC05.h"

HC05::HC05(Module* mod) : ISerial(mod) {
  
}

void HC05::begin(long speed) {
  // set module properties
  _mod->baudrate = speed;
  _mod->initUART(RADIOLIB_INT_NONE);
}
