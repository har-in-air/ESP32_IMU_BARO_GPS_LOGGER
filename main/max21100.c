#include "common.h"
#include "config.h"
#include "spihw.h"
#include "cct.h"
#include "max21100.h"
#include <math.h>

// gyro bias values read from/saved to nvs
int16_t gxBias;
int16_t gyBias;
int16_t gzBias;
// gyro scale value from datasheet
float 	gScale_;


// accel bias values read from/saved to nvs
int16_t axBias;
int16_t ayBias;
int16_t azBias;
// accel scale value from datasheet
float   aScale_;


#ifdef USE_MAG

// magnetometer biases and sensitivities read from/saved to nvs
int16_t mxBias;
int16_t myBias;
int16_t mzBias;
int16_t mxSensitivity;
int16_t mySensitivity;
int16_t mzSensitivity;
// magnetometer scales computed from sensitivities
float  mxScale_;
float  myScale_;
float  mzScale_;

#endif

#define TAG "max21100"

void MAX21100_InitCalibrationParams(void) {
#if 0
   // valid for +/-4G FS
	axBias = -491;
	ayBias = 542;
	azBias = 3935;	
#endif
	aScale_ = 1.0f/MAX21100_4G_SENSITIVITY; // accel in milli-gs
#if 0    
   // valid for FS=1000dps, BW = 100Hz
   gxBias = 7; 
   gyBias = 0; 
   gzBias = -29; 
#endif	
   gScale_  = 1.0f/MAX21100_1000DPS_SENSITIVITY;   // rotation rate in deg/second
	
#ifdef USE_MAG
	// HMC5883L accessed via MAX21100 master I2C interface
   // valid for FS = 4Gauss

#if 0	 
   mxBias = 441;
   myBias = -144;
   mzBias = 33;
	mxSensitivity = 527;
	mySensitivity = 417;
	mzSensitivity = 404;
#endif
	
	mxScale_ = mxSensitivity ? 1000.0f/(float)mxSensitivity : 1.0f;
	myScale_ = mySensitivity ? 1000.0f/(float)mySensitivity : 1.0f;
	mzScale_ = mzSensitivity ? 1000.0f/(float)mzSensitivity : 1.0f;
#endif
	}

#ifdef USE_MAG


void MAX21100_ConfigureGyroAccelMag(void) {
   delayMs(50);

    MAX21100_Write8(MAX21100_BANK_SELECT, 0);

    uint8_t regVal = MAX21100_Read8(MAX21100_I2C_CFG);
    regVal |= 0x01; 
    MAX21100_Write8(MAX21100_I2C_CFG, regVal);// disable slave i2c
    MAX21100_Write8(MAX21100_POWER_CFG, 0x7F); //  operating mode : accel+gyro low noise, all gyro axes enabled

    MAX21100_Write8(MAX21100_IO_CFG, 0xAB); // pulldown aux+int1+int2, disconnect internal i2c pullups
    MAX21100_Write8(MAX21100_I2C_PAD, 0x06); // more i2c driver current
    MAX21100_Write8(MAX21100_GYRO_CFG1, (GBW_150Hz << 2) | GFS_1000DPS); // bw = 150Hz, fs = 1000dps

    MAX21100_Write8(MAX21100_GYRO_CFG2, GODR_500Hz); // odr = 500Hz
    MAX21100_Write8(MAX21100_PWR_ACC_CFG, (AFS_4G << 6) | 0x07); // fs = 4G
    MAX21100_Write8(MAX21100_ACC_CFG1, (ABW_div3 << 4) | AODR_500Hz); // bw = odr/3, odr = 500Hz
   // MAX21100_Write8(MAX21100_ACC_CFG2, 0x02); // 500/2^1 = 250Hz magnetometer sampling, no accel HPF
    MAX21100_Write8(MAX21100_ACC_CFG2, 0x04); // 500/2^2 = 125Hz magnetometer sampling, no accel HPF
	
    // configure HMC5883L magnetometer connected to aux i2c bus
    MAX21100_Write8(MAX21100_MAG_SLV_ADD, 0x1E); // set hmc5883L 7bit i2c slave address (8bit addr = 0x3C )
    MAX21100_Write8(MAX21100_I2C_MST_ADD, 0x00);  // select register : config_a
    MAX21100_Write8(MAX21100_I2C_MST_RW, 0x78);  // Set ODR at 75Hz
    MAX21100_Write8(MAX21100_DR_CFG, 0x19); // i2c one-shot byte write
    MAX21100_Write8(MAX21100_DR_CFG, 0x09);
   delayMs(20);

    MAX21100_Write8(MAX21100_I2C_MST_ADD, 0x01); // select register : config_b
    MAX21100_Write8(MAX21100_I2C_MST_RW, 0x80); // set full scale to 4 gauss
    MAX21100_Write8(MAX21100_DR_CFG, 0x19); // i2c one-shot byte write
    MAX21100_Write8(MAX21100_DR_CFG, 0x09);
   delayMs(20);

    MAX21100_Write8(MAX21100_I2C_MST_ADD, 0x02); // hmc5883L select register : mode 
    MAX21100_Write8(MAX21100_I2C_MST_RW, 0x00); // set normal working mode
    MAX21100_Write8(MAX21100_DR_CFG, 0x19); // i2c one-shot byte write
    MAX21100_Write8(MAX21100_DR_CFG, 0x09);  // 1001, data ready cleared when status register read
 	 delayMs(20);

    MAX21100_Write8(MAX21100_MAG_SLV_REG, 0x03); // set hmc5883l sample data register address
    MAX21100_Write8(MAX21100_MAG_MAP_REG, 0x08 ); // XZY mapping, no axis inversions 
   // Enable magnetometer sampling, msb first, 400kHz i2c clock, read 6bytes of magnetometer data
    MAX21100_Write8(MAX21100_MAG_SLV_CFG, 0x86); 
	
    MAX21100_Write8(MAX21100_BANK_SELECT, 1);

    MAX21100_Write8(MAX21100_INT_SRC_SEL, 0x20); // gyro data ready generates drdy interrupt
    MAX21100_Write8(MAX21100_INT_MSK, 0x80); // drdy interrupt on INT1
    MAX21100_Write8(MAX21100_INT_TM0, 0x00); // INT1 unlatched
    MAX21100_Write8(MAX21100_INT_CFG2, 0x20); // INT1 enable
	ESP_LOGI(TAG, "interrupt mask = %02X", MAX21100_Read8(MAX21100_INT_MSK));
	ESP_LOGI(TAG, "interrupt src sel = %02X", MAX21100_Read8(MAX21100_INT_SRC_SEL));
		
   MAX21100_Write8(MAX21100_BANK_SELECT, 0);
	delayMs(50);
   }

	
void MAX21100_GetGyroAccelMagData(float* pGyroData, float* pAccelData, float* pMagData) {
	uint8_t buf[20]; // gyro+accel+mag+temp 
	int16_t raw[3];
	MAX21100_ReadBuf(MAX21100_GYRO_X_H, AUTO_INCR, 20, buf);
	raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	raw[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	raw[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	pGyroData[0] = (float)(raw[0] - gxBias) * gScale_;
   pGyroData[1] = (float)(raw[1] - gyBias) * gScale_;
   pGyroData[2] = (float)(raw[2] - gzBias) * gScale_;
	raw[0] = (int16_t)(((uint16_t)buf[6] << 8) | (uint16_t)buf[7]);
	raw[1] = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
	raw[2] = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);	
	pAccelData[0] = (float)(raw[0] - axBias) * aScale_;
	pAccelData[1] = (float)(raw[1] - ayBias) * aScale_;
	pAccelData[2] = (float)(raw[2] - azBias) * aScale_;
	raw[0] = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);
	raw[1] = (int16_t)(((uint16_t)buf[14] << 8) | (uint16_t)buf[15]);
	raw[2] = (int16_t)(((uint16_t)buf[16] << 8) | (uint16_t)buf[17]);	
	pMagData[0] = (float)(raw[0] - mxBias) * mxScale_;
	pMagData[1] = (float)(raw[1] - myBias) * myScale_;
	pMagData[2] = (float)(raw[2] - mzBias) * mzScale_;
	}


void MAX21100_GetGyroAccelMagDataRaw(int16_t* pGyroData, int16_t* pAccelData, int16_t* pMagData) {
	uint8_t buf[20]; // gyro+accel+mag+temp 
	MAX21100_ReadBuf(MAX21100_GYRO_X_H, AUTO_INCR, 20, buf);
	pGyroData[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	pGyroData[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	pGyroData[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	pAccelData[0] = (int16_t)(((uint16_t)buf[6] << 8) | (uint16_t)buf[7]);
	pAccelData[1] = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
	pAccelData[2] = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);	
	pMagData[0] = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);
	pMagData[1] = (int16_t)(((uint16_t)buf[14] << 8) | (uint16_t)buf[15]);
	pMagData[2] = (int16_t)(((uint16_t)buf[16] << 8) | (uint16_t)buf[17]);	
	}
		
#endif	

#if 0	
void MAX21100_ConfigureGyroAccel(void) {
    IMUCS_1();
    delayMs(10);

    MAX21100_Write8(MAX21100_BANK_SELECT, 0); 
    MAX21100_Write8(MAX21100_I2C_CFG, 0x29); // spi 4-pin slave interface, big-endian, disable slave I2C interface

    MAX21100_Write8(MAX21100_POWER_CFG, 0x7F); // operating mode : accel+gyro low noise, all gyro axes enabled
    MAX21100_Write8( MAX21100_GYRO_CFG1, (GBW_150Hz << 2) | GFS_1000DPS);
    MAX21100_Write8( MAX21100_GYRO_CFG2, GODR_500Hz);
    MAX21100_Write8( MAX21100_PWR_ACC_CFG, (AFS_4G << 6) | 0x07);
    MAX21100_Write8( MAX21100_ACC_CFG1, (ABW_div3 << 4) | AODR_500Hz);
	
    MAX21100_Write8(MAX21100_BANK_SELECT, 2); 
    MAX21100_Write8(MAX21100_BANK_SELECT, 0); 
    MAX21100_Write8( MAX21100_DR_CFG, 0x89);// bypass i2c master pads, clear DRDY on all data read, enable temp
    delayMs(100);
    }
	
void MAX21100_GetGyroAccelData(float* pGyroData, float* pAccelData) {
	uint8_t buf[20]; // gyro+accel+mag+temp 
	int16_t raw[3];
	MAX21100_ReadBuf(MAX21100_GYRO_X_H, AUTO_INCR, 20, buf);
	raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	raw[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	raw[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
   pGyroData[0] = (float)(raw[0] - gxBias) * gScale_;
   pGyroData[1] = (float)(raw[1] - gyBias) * gScale_;
   pGyroData[2] = (float)(raw[2] - gzBias) * gScale_;
	raw[0] = (int16_t)(((uint16_t)buf[6] << 8) | (uint16_t)buf[7]);
	raw[1] = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
	raw[2] = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);	
	pAccelData[0] = (float)(raw[0] - axBias) * aScale_;
	pAccelData[1] = (float)(raw[1] - ayBias) * aScale_;
	pAccelData[2] = (float)(raw[2] - azBias) * aScale_;
	}
#endif


// place unit so that the sensor board accelerometer z axis points 
// vertically (up or down). This is where the sensor z axis sees a static 
// acceleration of 1g (or -1g). In this orientation the ax and ay values are 
// the offsets (biases) for a 0g environment. 
// Repeat this calibration a few times with the debug serial monitor to check the 
// consistency of the calibration offsets. The board MUST be in a 1g static acceleration 
// environment for this calibration, i.e. at rest, no vibrations etc.

#define ACCEL_NUM_AVG_SAMPLES	50

void MAX21100_CalibrateAccel(void){
	int16_t ax,ay,az,az1g;
	int32_t axAccum, ayAccum, azAccum;
   ESP_LOGI(TAG, "Calibrating acceleration");
	axAccum = ayAccum = azAccum = 0;
	for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
      int accdr = 0;
      do {
         uint8_t status = MAX21100_Read8(MAX21100_SYSTEM_STATUS);
         if ((status & STATUS_ACC_DR) && !(status & STATUS_ACC_ERR)) accdr = 1;
         if (!accdr) taskYIELD();
         } while (!accdr);
      MAX21100_GetVector(MAX21100_ACC_X_H,&ax, &ay, &az);
		axAccum += (int32_t) ax;
		ayAccum += (int32_t) ay;
		azAccum += (int32_t) az;
		}
	axBias = (int16_t)(axAccum / ACCEL_NUM_AVG_SAMPLES);
	ayBias = (int16_t)(ayAccum / ACCEL_NUM_AVG_SAMPLES);
	az1g = (int16_t)(azAccum / ACCEL_NUM_AVG_SAMPLES);

   azBias = az1g > 0 ? az1g - (int16_t)(1000.0f*MAX21100_4G_SENSITIVITY) : az1g + (int16_t)(1000.0f*MAX21100_4G_SENSITIVITY);
   ESP_LOGI(TAG, "axBias = %d\r\nayBias = %d\r\nazBias = %d", (int)axBias, (int)ayBias, (int)azBias);
	}
	



#define GYRO_NUM_CALIB_SAMPLES			50
	
int MAX21100_CalibrateGyro(void) {
	int16_t gx,gy,gz;
	int32_t gxAccum, gyAccum, gzAccum;
	int foundBadData;
	int numTries = 1;
	do {
		delayMs(500);
		foundBadData = 0;
		gxAccum = gyAccum = gzAccum = 0;
		for (int inx = 0; inx < GYRO_NUM_CALIB_SAMPLES; inx++){
         int gyrodr = 0;
		   do {
            uint8_t status = MAX21100_Read8(MAX21100_SYSTEM_STATUS);
		      if ((status & STATUS_GYRO_DR) && !(status & STATUS_GYRO_ERR)) gyrodr = 1;
            if (!gyrodr) taskYIELD();
            } while (!gyrodr);

			MAX21100_GetVector(MAX21100_GYRO_X_H,&gx, &gy, &gz);
			ESP_LOGI(TAG, "[%d] %d %d %d", inx, gx, gy, gz);
			// if a larger than expected gyro bias is measured, assume the unit was disturbed and try again
         // after a short delay, upto 10 times
			if ((ABS(gx) > GYRO_MAX_EXPECTED_OFFSET_1000DPS) || 
            (ABS(gy) > GYRO_MAX_EXPECTED_OFFSET_1000DPS) || 
            (ABS(gz) > GYRO_MAX_EXPECTED_OFFSET_1000DPS)) {
				foundBadData = 1;
				ESP_LOGE(TAG, "gyro calibration error, [%d] bias > %d",inx, GYRO_MAX_EXPECTED_OFFSET_1000DPS);
				break;
				}  
			gxAccum  += (int32_t) gx;
			gyAccum  += (int32_t) gy;
			gzAccum  += (int32_t) gz;
			}
		} while (foundBadData && (++numTries < 10));

	// update gyro biases only if calibration succeeded, else use the last saved 
   // values from flash memory. Valid scenario for gyro calibration failing is 
   // when you turn on the unit while flying. So not a big deal.
    if (!foundBadData) {		
		gxBias =  (int16_t)( gxAccum / GYRO_NUM_CALIB_SAMPLES);
		gyBias =  (int16_t)( gyAccum / GYRO_NUM_CALIB_SAMPLES);
		gzBias =  (int16_t)( gzAccum / GYRO_NUM_CALIB_SAMPLES);		
		}
	ESP_LOGI(TAG,"Num Tries = %d",numTries);
	ESP_LOGI(TAG,"gxBias = %d",gxBias);
	ESP_LOGI(TAG,"gyBias = %d",gyBias);
	ESP_LOGI(TAG,"gzBias = %d",gzBias);
	return (foundBadData ? 0 : 1);
	}



#define MAG_NUM_CALIB_SAMPLES			2000
	
int MAX21100_CalibrateMag(void) {
	int16_t mx,my,mz;
	int16_t mxMin, myMin,mzMin,mxMax, myMax, mzMax;
	mxMin = myMin = mzMin = 9999;
   mxMax = myMax = mzMax = -9999;

	for (int inx = 0; inx < MAG_NUM_CALIB_SAMPLES; inx++){
      MAX21100_Read8(MAX21100_SYSTEM_STATUS);
		MAX21100_GetVector(MAX21100_MAG_X_H, &mx, &my, &mz);
	   ESP_LOGI(TAG, "[%d] %d %d %d", inx, mx, my, mz);
		if (mx > mxMax) mxMax = mx;
      if (mx < mxMin) mxMin = mx;
		if (my > myMax) myMax = my;
      if (my < myMin) myMin = my;
		if (mz > mzMax) mzMax = mz;
      if (mz < mzMin) mzMin = mz;
      delayMs(20);
		}
	ESP_LOGI(TAG,"mxMin = %d, mxMax = %d",mxMin, mxMax);
	ESP_LOGI(TAG,"myMin = %d, myMax = %d",myMin, myMax);
	ESP_LOGI(TAG,"mzMin = %d, mzMax = %d",mzMin, mzMax);

   mxBias = (mxMin+mxMax)/2;
   myBias = (myMin+myMax)/2;
   mzBias = (mzMin+mzMax)/2;
	mxSensitivity = (mxMax - mxMin)/2;
	mySensitivity = (myMax - myMin)/2;
	mzSensitivity = (mzMax - mzMin)/2;
	
	mxScale_ = 1000.0f/(float)mxSensitivity;
	myScale_ = 1000.0f/(float)mySensitivity;
	mzScale_ = 1000.0f/(float)mzSensitivity;
	ESP_LOGI(TAG,"mxBias = %d, myBias = %d, mzBias = %d",mxBias, myBias, mzBias);
	ESP_LOGI(TAG,"mxSens  = %d, mySens = %d, mzSens = %d",mxSensitivity, mySensitivity, mzSensitivity);

   return 1;
	}

#if 0
void MAX21100_GetRotation(int16_t* gx, int16_t* gy, int16_t* gz) {
   uint8_t buf[6];
   MAX21100_ReadBuf(MAX21100_GYRO_X_H, AUTO_INCR, 6, buf);
	*gx = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	*gy = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	*gz = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	}


void MAX21100_GetAcceleration(int16_t* ax, int16_t* ay, int16_t* az) {
   uint8_t buf[6];
   MAX21100_ReadBuf(MAX21100_ACC_X_H, AUTO_INCR, 6, buf);
	*ax = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	*ay = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	*az = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	}

void MAX21100_GetMag(int16_t* mx, int16_t* my, int16_t* mz) {
   uint8_t buf[6];
   MAX21100_ReadBuf(MAX21100_MAG_X_H, AUTO_INCR, 6, buf);
	*mx = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	*my = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	*mz = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	}
#endif

void MAX21100_GetVector(uint8_t startAddr, int16_t* px, int16_t* py, int16_t* pz) {
   uint8_t buf[6];
   MAX21100_ReadBuf(startAddr, AUTO_INCR, 6, buf);
	*px = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	*py = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	*pz = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	}

// top 2 bits of addr are used to specify R/W operation, and auto-increment address	
void MAX21100_Write8(uint8_t addr, uint8_t val) {
	addr &= 0x3F; // 00: write, increment addr
	spiSimpleTransaction(_spi);
	IMUCS_0();
	spiTransferByteNL(_spi, addr);
	spiTransferByteNL(_spi, val);
	IMUCS_1();
	spiEndTransaction(_spi);
	}
	

uint8_t MAX21100_Read8(uint8_t addr) {
	addr &= 0x3F;
	addr |= 0x80; // 10: read, increment addr
	spiSimpleTransaction(_spi);
	IMUCS_0();
	spiTransferByteNL(_spi, addr);
	uint8_t b = spiTransferByteNL(_spi, 0);
	IMUCS_1();
	spiEndTransaction(_spi);
	return b;
	}


// when reading from FIFO specify NO_AUTO_INCR, else AUTO_INCR
void MAX21100_ReadBuf(uint8_t addr, uint8_t autoIncrement, int numBytes, uint8_t* pBuf){
	addr &= 0x3F;
	addr |= (0x80 | autoIncrement);
	spiSimpleTransaction(_spi);
	IMUCS_0();
	spiTransferByteNL(_spi, addr);
   spiTransferBytesNL(_spi, NULL, pBuf, numBytes);
   // for (int inx = 0; inx < numBytes; inx++) { 
   //     pBuf[inx] = spiTransferByteNL(_spi, 0);
   //    }
	IMUCS_1();
	spiEndTransaction(_spi);	
	}
	


