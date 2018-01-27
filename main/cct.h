#ifndef CCT_H_
#define CCT_H_

#include "sdkconfig.h"

#define  CCT_TICKS_PER_US 	CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ 

void 		cct_DelayUs(uint32_t us);
uint32_t  	cct_IntervalUs(uint32_t before, uint32_t after);
float  		cct_IntervalSecs(uint32_t before, uint32_t after);
void 		cct_SetMarker(void);
uint32_t 	cct_ElapsedUs(void);

#endif
