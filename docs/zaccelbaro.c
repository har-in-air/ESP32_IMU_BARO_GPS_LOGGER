#include "common.h"
#include "config.h"
#include "imu.h"
#include "flashlog.h"
#include "kalmanfilter2.h"
#include "kalmanfilter3.h"
#include <math.h>
#include "assert.h"



float getzaccel(float gx, float gy, float gz, float ax, float ay, float az ) {
   float asqd = ax*ax + ay*ay + az*az;
	int bUseAccel = ((asqd > 490000.0) && (asqd < 1690000.0)) ? 1 : 0;	
   int bUseMag = 0;
	imu_MahonyAHRSupdate9DOF(bUseAccel, bUseMag,0.002, DEG2RAD(gx), DEG2RAD(gy), DEG2RAD(gz), ax, ay, az, 0.0, 0.0, 0.0);
   float gravityCompensatedAccel = imu_GravityCompensatedAccel(ax, ay, az, q0, q1, q2, q3);
   return gravityCompensatedAccel;
   }

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
   return imuRecordCounter;
   }




void getStartParameters(char* szFileName, int numRecords, int* pzInitialCm, float* pzAccelVarianceCps) {
	FILE* fp = fopen(szFileName, "rb");
	if (fp == NULL) {
		printf("error opening %s", szFileName);
		return;
		}

   LOG_HDR hdr;
   IMU_RECORD imu;
   BARO_RECORD baro;
   GPS_RECORD gps;
   float* zAccelBuffer = (float*)malloc(sizeof(float) * (numRecords+1));
   assert(zAccelBuffer != NULL);
   int imuCounter = 0;
   float zAccelAccum = 0.0;
   *pzInitialCm = 0;
	while (!feof(fp)) {	
      hdr.magic = 0;
		int hdrSize = fread(&hdr,1, sizeof(LOG_HDR), fp);
      int numRecordBytes = 0;
		if (hdrSize == sizeof(LOG_HDR)) {
         if (hdr.magic == 0xA55A) {
      		int imuSize = fread(&imu,1, sizeof(IMU_RECORD), fp);
            if (imuSize == sizeof(IMU_RECORD)) {
               float zacc = getzaccel(imu.gxNEDdps, imu.gyNEDdps, imu.gzNEDdps, imu.axNEDmG, imu.ayNEDmG, imu.azNEDmG);
               //printf("ZACCEL : %d\r\n", (int)zacc); 
               zAccelBuffer[imuCounter] = zacc;
               zAccelAccum += zacc;
               imuCounter++;
               } 
            if (hdr.baroFlags || hdr.gpsFlags) {
      		   int baroSize = fread(&baro,1, sizeof(BARO_RECORD), fp);
               if ((baroSize == sizeof(BARO_RECORD)) && (baro.heightMSLcm != 0)) {
                  if (*pzInitialCm == 0) *pzInitialCm = baro.heightMSLcm;
                  //printf("BARO : %d\r\n",baro.heightMSLcm); 
                  } 
               }
            if (hdr.gpsFlags) { // read past gps record
      		   int gpsSize = fread(&gps,1, sizeof(GPS_RECORD), fp);
               }
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

   float zAccelMean = zAccelAccum/numRecords;
   float zAccelVarAccum = 0.0;
   for (int inx = 0; inx < numRecords; inx++) {
      float v = zAccelBuffer[inx] - zAccelMean;
      zAccelVarAccum +=  (v*v);     
      }
   float zAccelVar = zAccelVarAccum/(numRecords-1);
   float zAccelSD = sqrt(zAccelVar);
   //printf("Acceleration variance = %f mG*mG, standard deviation = %fmG\r\n", zAccelVar, zAccelSD);
   float zAccelCpsSD = zAccelSD*0.98; // 1000mG = 980cm/s*s
   float zAccelCpsVar = zAccelCpsSD*zAccelCpsSD;
   //printf("Acceleration variance = %f cps*cps, standard deviation = %fcps\r\n", zAccelCpsVar, zAccelCpsSD);
   *pzAccelVarianceCps = zAccelCpsVar;
}


void processData(char* szFileName, int numRecords, int zInitialCm, float zAccelVariance) {

   kalmanFilter2_Configure(KF_ZMEAS_VARIANCE_DEFAULT,zAccelVariance, (float)zInitialCm, 0.0);

   kalmanFilter3_Configure(KF_ZMEAS_VARIANCE_DEFAULT,zAccelVariance, KF_ACCELBIAS_VARIANCE, (float)zInitialCm, 0.0, 0.0);

	FILE* fp = fopen(szFileName, "rb");
	if (fp == NULL) {
		printf("error opening %s", szFileName);
		return;
		}

   LOG_HDR hdr;
   IMU_RECORD imu;
   BARO_RECORD baro;
   GPS_RECORD gps;
   float zAccelAccum = 0.0;
   int imuCounter = 0;
   float kf2Z, kf2V;
   float kf3Z, kf3V;
	while (!feof(fp)) {	
      hdr.magic = 0;
		int hdrSize = fread(&hdr,1, sizeof(LOG_HDR), fp);
      int numRecordBytes = 0;
		if (hdrSize == sizeof(LOG_HDR)) {
         if (hdr.magic == 0xA55A) {
      		int imuSize = fread(&imu,1, sizeof(IMU_RECORD), fp);
            if (imuSize == sizeof(IMU_RECORD)) {
               float zacc = getzaccel(imu.gxNEDdps, imu.gyNEDdps, imu.gzNEDdps, imu.axNEDmG, imu.ayNEDmG, imu.azNEDmG);
               zAccelAccum += zacc;
               imuCounter++;
               } 
            if (hdr.baroFlags || hdr.gpsFlags) {
      		   int baroSize = fread(&baro,1, sizeof(BARO_RECORD), fp);
               if ((baroSize == sizeof(BARO_RECORD)) && (baro.heightMSLcm != 0)) {
                  float zAverageAccel = zAccelAccum/(float)imuCounter;
                  kalmanFilter2_Update((float)baro.heightMSLcm, zAccelVariance,0.020, &kf2Z, &kf2V);
                  kalmanFilter3_Update((float)baro.heightMSLcm, zAverageAccel,0.020, &kf3Z, &kf3V);
                  //printf("%d %d : %d\r\n", baro.heightMSLcm, (int) kf3Z, (int)kf3V);
                  printf("%d, %d, %d\r\n", baro.heightMSLcm, (int) kf2Z, (int) kf3Z);
                  imuCounter = 0;
                  zAccelAccum = 0.0;
                  } 
               }
            if (hdr.gpsFlags) { // read past gps record
      		   int gpsSize = fread(&gps,1, sizeof(GPS_RECORD), fp);
               }
            }
         else {
            printf("Error : magic not found\r\n");
            break;
            }
         }
      else {
//         printf("Did not read hdr len bytes\r\n");
         break;

         }
      }
   fclose(fp);
   }

   



// Structs seem to be packed without using any special compiler directives
// Using gcc, ubuntu 16.04, AMD x64 
// gcc -o zaccelbaro zaccelbaro.c imu.c kalmanfilter2.c kalmanfilter3.c -lm
//  
// And ESP32 is also little endian, so no problem reading the structures from the binary file.

int main(int argc, char* argv[]) {
   if (argc != 2) {
      printf("usage : %s <binaryLogFileName>\r\n", argv[0]);
      return -1;
      }

   int numRecords = getNumRecords(argv[1]);
   printf("Found %d imu records\r\n", numRecords);
   int zInitialCm;
   float zAccelVarCps;
   getStartParameters(argv[1],numRecords, &zInitialCm, &zAccelVarCps);
   printf("Initial altitude = %dcm\r\n", zInitialCm);
   printf("Acceleration variance = %fcps^2\r\n", zAccelVarCps);

   processData(argv[1], numRecords, zInitialCm, zAccelVarCps);
   return 0;
	}

