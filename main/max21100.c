#include "common.h"
#include "config.h"
#include "spihw.h"
#include "cct.h"
#include "max21100.h"
#include <math.h>

int16_t gxBias;
int16_t gyBias;
int16_t gzBias;
float 	gScale_;

int16_t axBias;
int16_t ayBias;
int16_t azBias;
float   aScale_;


#ifdef USE_MAG
int16_t mxBias;
int16_t myBias;
int16_t mzBias;
int16_t mxSensitivity;
int16_t mySensitivity;
int16_t mzSensitivity;

float  mxScale_;
float  myScale_;
float  mzScale_;

#endif

#define TAG "max21100"

void MAX21100_InitCalibrationParams(void) {
   // valid for +/-4G FS
	axBias = -179;
	ayBias = 11;
	azBias = 1707;	
	aScale_ = 1.0f/MAX21100_4G_SENSITIVITY; // accel in milli-gs
    
   // valid for FS=1000dps, BW = 100Hz
   gxBias = 7; 
   gyBias = 0; 
   gzBias = -29; 
	
   gScale_  = 1.0f/MAX21100_1000DPS_SENSITIVITY;   // rotation rate in deg/second
	
#ifdef USE_MAG
	// HMC5883L accessed via MAX21100 master I2C interface
   // valid for FS = 4Gauss

	// mx [-37, 920]
	// my [-611, 323]
	// mz [-381, 448]
	 
   mxBias = 441;
   myBias = -144;
   mzBias = 33;
	mxSensitivity = 957;
	mySensitivity = 934;
	mzSensitivity = 829;
	
	mxScale_ = 1000.0f/(float)mxSensitivity;
	myScale_ = 1000.0f/(float)mySensitivity;
	mzScale_ = 1000.0f/(float)mzSensitivity;
#endif
	}

#ifdef USE_MAG

void MAX21100_ConfigureGyroAccelMag(void) {
   delayMs(50);

	// BANK 0
    MAX21100_Write8(MAX21100_BANK_SELECT, 0);
    MAX21100_Write8(MAX21100_POWER_CFG, 0x7F); //  operating mode : accel+gyro low noise, all gyro axes enabled
    MAX21100_Write8(MAX21100_I2C_CFG, 0x69); // standard master i2c, spi 4-pin slave interface, big-endian, disable slave I2C interface
    MAX21100_Write8(MAX21100_IO_CFG, 0xAB); // pulldown aux+int1+int2, disconnect internal master and slave i2c pullups
    MAX21100_Write8(MAX21100_I2C_PAD, 0x02); // strong master drive current
    MAX21100_Write8(MAX21100_GYRO_CFG1, (GBW_150Hz << 2) | GFS_1000DPS);
    MAX21100_Write8(MAX21100_GYRO_CFG2, GODR_500Hz);
    MAX21100_Write8(MAX21100_PWR_ACC_CFG, (AFS_4G << 6) | 0x07); // accelerometer full scale and all accel axes enabled
    MAX21100_Write8(MAX21100_ACC_CFG1, (ABW_div3 << 4) | AODR_500Hz);
    MAX21100_Write8(MAX21100_ACC_CFG2, 0x00); // sample mag at accelerometer rate, disable accel HPF
	
    MAX21100_Write8(MAX21100_MAG_SLV_ADD, 0x1E); // set hmc5883L 7bit i2c slave address (8bit addr = 0x3C )
#if 0
    MAX21100_Write8(MAX21100_I2C_MST_ADD, 0x10); // hmc5883L select register : IDA
    MAX21100_Write8(MAX21100_DR_CFG, 0x35); // i2c one-shot byte read
    MAX21100_Write8(MAX21100_DR_CFG, 0x05); 
	cct_DelayUs(5000);
	ESP_LOGI(TAG, "IDA = %x", MAX21100_Read8(MAX21100_I2C_MST_RW));
    MAX21100_Write8(MAX21100_I2C_MST_ADD, 0x11); // hmc5883L select register : IDB
    MAX21100_Write8(MAX21100_DR_CFG, 0x35); // i2c one-shot byte read
    MAX21100_Write8(MAX21100_DR_CFG, 0x05); 
	cct_DelayUs(5000);
	ESP_LOGI(TAG, "IDB = %x", MAX21100_Read8(MAX21100_I2C_MST_RW));
    MAX21100_Write8(MAX21100_I2C_MST_ADD, 0x12); // hmc5883L select register : IDC
    MAX21100_Write8(MAX21100_DR_CFG, 0x35); // i2c one-shot byte read
    MAX21100_Write8(MAX21100_DR_CFG, 0x05); 
	cct_DelayUs(5000);
	ESP_LOGI(TAG, "IDC = %x", MAX21100_Read8(MAX21100_I2C_MST_RW));
#endif	

    MAX21100_Write8(MAX21100_I2C_MST_ADD, 0x00);  // select register : config_a
    MAX21100_Write8(MAX21100_I2C_MST_RW, 0x78);  // Set ODR at 75Hz
    MAX21100_Write8(MAX21100_DR_CFG, 0x15); // i2c one-shot byte write
    MAX21100_Write8(MAX21100_DR_CFG, 0x05);

    MAX21100_Write8(MAX21100_I2C_MST_ADD, 0x01); // select register : config_b
    MAX21100_Write8(MAX21100_I2C_MST_RW, 0x80); // set full scale to 4 gauss
    MAX21100_Write8(MAX21100_DR_CFG, 0x15); // i2c one-shot byte write
    MAX21100_Write8(MAX21100_DR_CFG, 0x05);

    MAX21100_Write8(MAX21100_I2C_MST_ADD, 0x02); // hmc5883L select register : mode 
    MAX21100_Write8(MAX21100_I2C_MST_RW, 0x00); // set normal working mode
    MAX21100_Write8(MAX21100_DR_CFG, 0x15); // i2c one-shot byte write
    MAX21100_Write8(MAX21100_DR_CFG, 0x05); 
 	 delayMs(20);

    MAX21100_Write8(MAX21100_MAG_SLV_REG, 0x03); // set hmc5883l sample data register address
    MAX21100_Write8(MAX21100_MAG_MAP_REG, 0x08 ); // XZY mapping, no axis inversions 
    MAX21100_Write8(MAX21100_MAG_SLV_CFG, 0x86); // Enable magnetometer, msb first, 400kHz i2c clock, read 6bytes of magnetometer data
	
    //MAX21100_Write8(MAX21100_DR_CFG, 0x09); // lower nybble = 1001 => dataready cleared when status reg is read, temperature enabled
    MAX21100_Write8(MAX21100_DR_CFG, 0x01); // lower nybble = 0001 => dataready cleared when all data is read, temperature enabled

	// BANK 1
    MAX21100_Write8(MAX21100_BANK_SELECT, 1);
    MAX21100_Write8(MAX21100_INT_SRC_SEL, 0x20); // gyro data ready generates drdy interrupt
    MAX21100_Write8(MAX21100_INT_MSK, 0x80); // drdy interrupt on INT1
    MAX21100_Write8(MAX21100_INT_TM0, 0x00); // INT1 unlatched
    MAX21100_Write8(MAX21100_INT_CFG2, 0x20); // INT1 enable
	ESP_LOGI(TAG, "interrupt mask = %02X", MAX21100_Read8(MAX21100_INT_MSK));
	ESP_LOGI(TAG, "interrupt src sel = %02X", MAX21100_Read8(MAX21100_INT_SRC_SEL));
	
	// BANK 2
    MAX21100_Write8(MAX21100_BANK_SELECT, 2); 
   // MAX21100_Write8(MAX21100_FUS_CFG0,0x07); // enable quaternion output @ 250Hz using g+a+m data
	
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
	
	
void MAX21100_GetMagData( float* pMagData) {
	uint8_t buf[6]; 
	int16_t raw[3];
	MAX21100_ReadBuf(MAX21100_MAG_X_H, AUTO_INCR, 6, buf);
	raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	raw[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	raw[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	pMagData[0] = (float)(raw[0] - mxBias) * mxScale_;
	pMagData[1] = (float)(raw[1] - myBias) * myScale_;
	pMagData[2] = (float)(raw[2] - mzBias) * mzScale_;
	}
	
#if 0
void MAX21100_GetQuaternions(float* pQuat) {
    uint8_t buf[8];
    MAX21100_Write8(MAX21100_BANK_SELECT, 2);
    MAX21100_ReadBuf(MAX21100_QUAT0_H, AUTO_INCR, 8, buf);
	int16_t quat[4];
    quat[0] = (int16_t)((((uint16_t)buf[0]) << 8) | (uint16_t)buf[1]);
    quat[1] = (int16_t)((((uint16_t)buf[2]) << 8) | (uint16_t)buf[3]);
    quat[2] = (int16_t)((((uint16_t)buf[4]) << 8) | (uint16_t)buf[5]);
    quat[3] = (int16_t)((((uint16_t)buf[6]) << 8) | (uint16_t)buf[7]);
    MAX21100_Write8(MAX21100_BANK_SELECT, 0);
	float invnorm = 1.0f/sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
	pQuat[0] = (float) quat[0] * invnorm;
	pQuat[1] = (float) quat[1] * invnorm;
	pQuat[2] = (float) quat[2] * invnorm;
	pQuat[3] = (float) quat[3] * invnorm;	
    }	
#endif	
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
      MAX21100_GetAcceleration(&ax, &ay, &az);
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

			MAX21100_GetRotation(&gx, &gy, &gz);
			//ESP_LOGI(TAG, "[%d] %d %d %d", inx, gx, gy, gz);
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

	// update gyro biases only if calibration succeeded, else use the last saved values from flash memory. Valid scenario for
	// gyro calibration failing is when you turn on the unit while flying. So not a big deal.
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
	


