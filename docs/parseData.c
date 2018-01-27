#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define FLASHLOG_RECORD_NBYTES		80
#define FLASHLOG_MAX_ADDR		      (FLASH_SIZE_BYTES - FLASHLOG_RECORD_NBYTES) 


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

// if there is only an IMU record, size of record = HDR + IMU
// if there is a BARO record, size of record = HDR + IMU + BARO
// if there is a GPS record, size of record = HDR + IMU + BARO + GPS even if BARO record is invalid

int getNumRecords(char* szFileName) {
   int imuRecordCounter = 0;
   int baroRecordCounter = 0;
   int gpsRecordCounter = 0;
	FILE* fp = fopen(szFileName, "rb");
	if (fp == NULL) {
		printf("error opening %s", szFileName);
		return(-1);
		}

	while (!feof(fp)) {	
      LOG_HDR hdr;
      hdr.magic = 0;
		int hdrSize = fread(&hdr,1, sizeof(LOG_HDR), fp);
      int numRecordBytes = 0;
		if (hdrSize == sizeof(LOG_HDR)) {
         if (hdr.magic == 0xA55A) {
            numRecordBytes = sizeof(IMU_RECORD);
            if (hdr.baroFlags || hdr.gpsFlags) numRecordBytes += sizeof(BARO_RECORD);
            if (hdr.gpsFlags) numRecordBytes += sizeof(GPS_RECORD);
            if (fseek(fp, numRecordBytes, SEEK_CUR)) {
               printf("Fseek past end of file\r\n");
               break;
               }
            imuRecordCounter++;
            if (hdr.baroFlags) baroRecordCounter++;
            if (hdr.gpsFlags) gpsRecordCounter++;
            }
         else {
            printf("Error : magic not found\r\n");
            break;
            }
         }
      else {
         printf("Did not read hdr len bytes\r\n");
         break;
         }
      }
   fclose(fp);

   printf("Found %d imu records\r\n", imuRecordCounter);
   printf("Found %d baro records\r\n", baroRecordCounter);
   printf("Found %d gps records\r\n\r\n", gpsRecordCounter);
   }



int printRecords(char* szFileName) {
   int imuRecordCounter = 0;
   int baroRecordCounter = 0;
   int gpsRecordCounter = 0;
	FILE* fp = fopen(szFileName, "rb");
	if (fp == NULL) {
		printf("error opening %s", szFileName);
		return(-1);
		}

   LOG_HDR hdr;
   IMU_RECORD imu;
   BARO_RECORD baro;
   GPS_RECORD gps;

	while (!feof(fp)) {	
      hdr.magic = 0;
		int hdrSize = fread(&hdr,1, sizeof(LOG_HDR), fp);
      int numRecordBytes = 0;
		if (hdrSize == sizeof(LOG_HDR)) {
         if (hdr.magic == 0xA55A) {
      		int imuSize = fread(&imu,1, sizeof(IMU_RECORD), fp);
            if (imuSize == sizeof(IMU_RECORD)) {
               printf("IMU : %f %f %f %f %f %f\r\n", 
                  imu.gxNEDdps, imu.gyNEDdps, imu.gzNEDdps, 
                  imu.axNEDmG, imu.ayNEDmG, imu.azNEDmG);
               } 
            if (hdr.baroFlags || hdr.gpsFlags) {
      		   int baroSize = fread(&baro,1, sizeof(BARO_RECORD), fp);
               if (baroSize == sizeof(BARO_RECORD)) {
                  printf("BARO : %d\r\n",baro.heightMSLcm); 
                  } 
               }
            if (hdr.gpsFlags) {
      		   int gpsSize = fread(&gps,1, sizeof(GPS_RECORD), fp);
               if (gpsSize == sizeof(GPS_RECORD)) {
                  printf("GPS : %d %f %f %d\r\n", gps.timeOfWeekmS, (float)gps.lonDeg7/10000000.0f,(float)gps.latDeg7/10000000.0f,
                     gps.heightMSLmm/10); 
                  } 
               }
            }
         else {
            printf("\r\nError : magic not found\r\n");
            break;
            }
         }
      else {
         printf("\r\nDid not read hdr len bytes\r\n");
         break;
         }
      }
   fclose(fp);
   }


// Structs seem to be packed without using any special compiler directives
// Using gcc, ubuntu 16.04, AMD x64 
// gcc -o parseData parseData.c
//  
// And ESP32 is also little endian, so no problem reading the structures from the binary file.

int main(int argc, char* argv[]) {
   if (argc != 2) {
      printf("usage : %s <binaryLogFileName>\r\n", argv[0]);
      return -1;
      }

   printf("sizeof LOG_HDR = %lu\r\n",  sizeof(LOG_HDR));
   printf("sizeof IMU_RECORD = %lu\r\n",  sizeof(IMU_RECORD));
   printf("sizeof FLASHLOG_RECORD = %lu\r\n\r\n",  sizeof(FLASHLOG_RECORD));

   getNumRecords(argv[1]);

   printRecords(argv[1]);
   return 0;
	}

