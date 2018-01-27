#include "common.h"
#include "config.h"
#include "cct.h"
#include "ms5611.h"
#include "spihw.h"

float zCmAvg;
float zCmSample;
float paSample;
int   celsiusSample;

static uint8_t 	prom_[16];
static uint16_t   cal_[6];
static int64_t 	tref_;
static int64_t 	offT1_;
static int64_t 	sensT1_;	
static int32_t 	tempCx100_;
static uint32_t   D1_;
static uint32_t   D2_;
static int64_t 	dT_;	
static int 		   sensorState_;

#define TAG "MS5611"


int MS5611_Configure(void) {
	BAROCS_1();
	paSample = 0.0f;
	zCmSample = 0.0f;
	celsiusSample = 0;
	zCmAvg = 0.0f;
	MS5611_Reset();
	if (!MS5611_ReadPROM()) {
		ESP_LOGE(TAG, "Error reading calibration PROM");
		return 0;
		}
	MS5611_GetCalibrationParameters();
	return 1;
	}


void MS5611_InitializeSampleStateMachine(void) {
   MS5611_TriggerTemperatureSample();
   sensorState_ = MS5611_READ_TEMPERATURE;
   }


int MS5611_SampleStateMachine(void) {
   if (sensorState_ == MS5611_READ_TEMPERATURE) {
      D2_ = MS5611_ReadSample();
      MS5611_TriggerPressureSample();
      MS5611_CalculateTemperatureCx10();
      //celsiusSample_ = (tempCx100_ >= 0? (tempCx100_+50)/100 : (tempCx100_-50)/100);
      paSample = MS5611_CalculatePressurePa();
	   zCmSample = MS5611_Pa2Cm(paSample);
	   sensorState_ = MS5611_READ_PRESSURE;
      return 1;  // new altitude sample is available
      }
   else
   if (sensorState_ == MS5611_READ_PRESSURE) {
      D1_ = MS5611_ReadSample();
      MS5611_TriggerTemperatureSample();
      sensorState_ = MS5611_READ_TEMPERATURE;
      return 0; // intermediate state, no new result available
      }
   return 0;    
   }

#ifdef MS5611_TEST

#define MAX_TEST_SAMPLES    100
extern char gszBuf[];
static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void MS5611_Test(int nSamples) {
	int32_t n;
    float paMean, zMean, zVariance, paVariance;
    paMean = 0.0f;
    zMean = 0.0f;
    paVariance = 0.0f;
    zVariance = 0.0f;
    for (n = 0; n < nSamples; n++) {
	    MS5611_TriggerTemperatureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
	    D2_ = MS5611_ReadSample();
	    MS5611_CalculateTemperatureCx10();
		MS5611_TriggerPressureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D1_ = MS5611_ReadSample();
		pa[n] = MS5611_CalculatePressurePa();
        z[n] =  MS5611_Pa2Cm(pa[n]);
        paMean += pa[n];
        zMean += z[n];
        }
    paMean /= nSamples;
    zMean /= nSamples;
    for (n = 0; n < nSamples; n++) {
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zVariance += (z[n]-zMean)*(z[n]-zMean);
        ESP_LOGI(TAG,"%f %f\r\n",pa[n],z[n]);
       }
    paVariance /= (nSamples-1);
    zVariance /= (nSamples-1);
    ESP_LOGI(TAG,"paVariance %f  zVariance %f",paVariance,zVariance);    
	}
#endif


void MS5611_AveragedSample(int nSamples) {
	int32_t tc,tAccum,n;
    float pa,pAccum;
	pAccum = 0.0f;
    tAccum = 0;
	n = nSamples;
    while (n--) {
		MS5611_TriggerTemperatureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D2_ = MS5611_ReadSample();
		MS5611_CalculateTemperatureCx10();
		MS5611_TriggerPressureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D1_ = MS5611_ReadSample();
		pa = MS5611_CalculatePressurePa();
		pAccum += pa;
		tAccum += tempCx100_;
		}
	tc = tAccum/nSamples;
	celsiusSample = (tc >= 0 ?  (tc+50)/100 : (tc-50)/100);
	paSample = (pAccum+nSamples/2)/nSamples;
	zCmAvg = zCmSample = MS5611_Pa2Cm(paSample);
	}
	
	

/// Fast Lookup+Interpolation method for converting pressure readings to altitude readings.
#include "pztbl.txt"

float MS5611_Pa2Cm(float paf)  {
   	int32_t pa,inx,pa1,z1,z2;
    float zf;
    pa = (int32_t)paf;

   	if (pa > PA_INIT) {
      	zf = (float)(gPZTbl[0]);
      	}
   	else {
      	inx = (PA_INIT - pa)>>10;
      	if (inx >= PZLUT_ENTRIES-1) {
         	zf = (float)(gPZTbl[PZLUT_ENTRIES-1]);
         	}
      	else {
         	pa1 = PA_INIT - (inx<<10);
         	z1 = gPZTbl[inx];
         	z2 = gPZTbl[inx+1];
         	zf = (float)z1 + ((((float)pa1)-paf)*(float)(z2-z1))/1024.0f;
         	}
      	}
   	return zf;
   	}


void MS5611_TriggerPressureSample(void) {
	spiSimpleTransaction(_spi);
   BAROCS_0();
	spiTransferByteNL(_spi, MS5611_CMD_CONVERT_D1|MS5611_CMD_ADC_4096);
	BAROCS_1();
	spiEndTransaction(_spi);
	}


void MS5611_TriggerTemperatureSample(void) {
	spiSimpleTransaction(_spi);
   BAROCS_0();
   spiTransferByteNL(_spi, MS5611_CMD_CONVERT_D2|MS5611_CMD_ADC_4096);
   BAROCS_1();
	spiEndTransaction(_spi);
	}


uint32_t MS5611_ReadSample(void)	{
	uint32_t w, b0, b1, b2;
	spiSimpleTransaction(_spi);
	BAROCS_0();
	spiTransferByteNL(_spi, MS5611_CMD_ADC_READ);
	b0 = (uint32_t)spiTransferByteNL(_spi, 0);
	b1 = (uint32_t)spiTransferByteNL(_spi, 0);
	b2 = (uint32_t)spiTransferByteNL(_spi, 0);
	w = ((b0<<16) | (b1<<8) | b2);
	BAROCS_1();
	spiEndTransaction(_spi);
	return w;
	}


void MS5611_CalculateTemperatureCx10(void) {
	dT_ = (int64_t)D2_ - tref_;
	tempCx100_ = 2000 + ((dT_*((int32_t)cal_[5]))>>23);
	}


float MS5611_CalculatePressurePa(void) {
	float pa;
    int64_t offset, sens,offset2,sens2,t2;
	offset = offT1_ + ((((int64_t)cal_[3])*(int64_t)dT_)>>7);
	sens = sensT1_ + ((((int64_t)cal_[2])*(int64_t)dT_)>>8);
    if (tempCx100_ < 2000) {
        t2 = ((dT_*dT_)>>31); 
        offset2 = (5*(tempCx100_-2000)*(tempCx100_-2000))/2;
        sens2 = offset2/2;
        } 
    else {
        t2 = 0;
        sens2 = 0;
        offset2 = 0;
        }
    tempCx100_ -= t2;
    offset -= offset2;
    sens -= sens2;
	pa = ((float)((int64_t)D1_ * sens)/2097152.0f - (float)(offset)) / 32768.0f;
	return pa;
	}



void MS5611_Reset(void) {
	BAROCS_0();
	spiTransferByteNL(_spi, MS5611_CMD_RESET);
	BAROCS_1();
	spiEndTransaction(_spi);
	cct_DelayUs(4000); // > 3mS as per app note AN520	
   }

   	
void MS5611_GetCalibrationParameters(void)  {
   for (int inx = 0; inx < 6; inx++) {
		int promIndex = 2 + inx*2; 
		cal_[inx] = (((uint16_t)prom_[promIndex])<<8) | (uint16_t)prom_[promIndex+1];
		}
   ESP_LOGI(TAG,"Calib Coeffs : %d %d %d %d %d %d",cal_[0],cal_[1],cal_[2],cal_[3],cal_[4],cal_[5]);
   tref_ = ((int64_t)cal_[4])<<8;
   offT1_ = ((int64_t)cal_[1])<<16;
   sensT1_ = ((int64_t)cal_[0])<<15;		
   }
   
int MS5611_ReadPROM(void)    {
   for (int inx = 0; inx < 8; inx++) {
    	BAROCS_0();
		spiTransferByteNL(_spi, 0xA0 + inx*2);
		prom_[inx*2] = spiTransferByteNL(_spi, 0);
		prom_[inx*2+1] = spiTransferByteNL(_spi, 0);
		BAROCS_1();
		}			
   uint8_t crcPROM = prom_[15] & 0x0F;
	uint8_t crcCalculated = MS5611_CRC4(prom_);
	return (crcCalculated == crcPROM ? 1 : 0);
	}
	
	
uint8_t MS5611_CRC4(uint8_t prom[] ) {
   int cnt, nbit; 
   uint16_t crcRemainder; 
   uint8_t crcSave = prom[15]; // crc byte in PROM
   ESP_LOGI(TAG,"PROM CRC = 0x%x", prom[15] & 0x0F);
   crcRemainder = 0x0000;
   prom[15] = 0; //CRC byte is replaced by 0

   for (cnt = 0; cnt < 16; cnt++)  {
		crcRemainder ^= (uint16_t) prom[cnt];
		for (nbit = 8; nbit > 0; nbit--) {
			if (crcRemainder & (0x8000)) {
				crcRemainder = (crcRemainder << 1) ^ 0x3000; 
				}
			else {
				crcRemainder = (crcRemainder << 1);
				}
			}
		}
   crcRemainder= (0x000F & (crcRemainder >> 12)); // final 4-bit reminder is CRC code
   prom[15] = crcSave; // restore the crc byte
   ESP_LOGI(TAG, "Calculated CRC = 0x%x",  crcRemainder ^ 0x0);
   return (uint8_t)(crcRemainder ^ 0x0);
   } 

