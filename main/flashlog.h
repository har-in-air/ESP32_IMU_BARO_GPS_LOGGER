#ifndef FLASHLOG_H_
#define FLASHLOG_H_

#include "spiflash.h"

#define FLASHLOG_RECORD_MAXBYTES		80
#define FLASHLOG_MAX_ADDR		      (FLASH_SIZE_BYTES - FLASHLOG_RECORD_MAXBYTES) 

// 128Mbit flash can hold 16777216 bytes

typedef struct LOG_HDR_ {
	uint16_t	magic; // 0xA55A
   uint8_t  gpsFlags; 
   uint8_t  baroFlags;
} LOG_HDR;

typedef struct IMU_RECORD_ {
	float 	gxNEDdps;
	float 	gyNEDdps;
	float 	gzNEDdps;
	float 	axNEDmG;
	float 	ayNEDmG;
	float 	azNEDmG;
	float 	mxNED;
	float 	myNED;
	float 	mzNED;
   } IMU_RECORD;

typedef struct BARO_RECORD_ {
   int32_t  heightMSLcm;
} BARO_RECORD;

typedef struct GPS_RECORD_ {
	uint32_t timeOfWeekmS;
	int32_t 	lonDeg7;
	int32_t 	latDeg7;	
	int32_t	heightMSLmm;
	uint32_t	vertAccuracymm;
	int32_t	velNorthmmps;
	int32_t	velEastmmps;
	int32_t	velDownmmps;
	uint32_t	velAccuracymmps;
} GPS_RECORD;

typedef struct FLASHLOG_RECORD_ {
   LOG_HDR     hdr;
   IMU_RECORD  imu;
   BARO_RECORD baro;
   GPS_RECORD  gps;
} FLASHLOG_RECORD;
 

extern FLASHLOG_RECORD     FlashLogRecord;
extern SemaphoreHandle_t   FlashLogMutex;
extern int                 IsLogging;
extern uint32_t            FlashLogFreeAddress;

int   FlashLog_Init(void);
void  FlashLog_GetFreeAddress(int* pNumRecords);
void 	FlashLog_EraseChip(void);
void 	FlashLog_Erase(void);
void 	FlashLog_WriteRecord(FLASHLOG_RECORD* pRecord);
void 	FlashLog_ReadRecord(uint32_t addr, FLASHLOG_RECORD* pRecord);


#endif


