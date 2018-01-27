#ifndef SPIHW_H_
#define SPIHW_H_

#include "hal/esp32-hal-spi.h"

void SpiHW_Config(int pnSCK, int pnMOSI, int pnMISO, int freq);
void SpiHW_SetFreq(int freq);

extern spi_t* _spi;

#define SpiHW_Transfer(b) spiTransferByte(_spi, b)


#endif