#include "common.h"
#include "config.h"
#include "spihw.h"


int8_t 	_spi_num = 0;
spi_t * _spi = NULL;
bool 	_use_hw_ss;
int8_t 	_sck;
int8_t 	_miso;
int8_t 	_mosi;
int8_t 	_ss;
uint32_t _div;
uint32_t _freq;
bool _inTransaction;

#define TAG "SpiHW"

// use IO_MUX pad pins for high clock speed
// 	HSPI(SPI2)	VSPI(SPI3) 
// SCLK	14		18
// MOSI	13		23
// MISO	12		19
// CS	   15		5

// SPI clocks divided from APB (80MHz)
// 40MHz, 26.6, 20, 16, 13.3, 11.4, 10
// for writing, can use any of these
// for reading and IO_MUX pad, 20MHz works.
// for GPIOmatrix pins, <= 10MHz required

void SpiHW_Config(int pnSCK, int pnMOSI, int pnMISO, int freq) {
	_spi_num = VSPI;
	_freq = freq;
	_div = spiFrequencyToClockDiv(_freq);
	_spi = spiStartBus(_spi_num, _div, SPI_MODE0, SPI_MSBFIRST);
	if(!_spi) {
		ESP_LOGE(TAG,"spiStartBus error");
		return;
		}

    _sck = pnSCK;
    _miso = pnMISO;
    _mosi = pnMOSI;
    _ss = -1;

    spiAttachSCK(_spi, _sck);
    spiAttachMISO(_spi, _miso);
    spiAttachMOSI(_spi, _mosi);
}


void SpiHW_SetFreq(int freq) {
	_freq = freq;
	_div = spiFrequencyToClockDiv(_freq);
	spiSetClockDiv(_spi, _div);
	}
	
	
	
