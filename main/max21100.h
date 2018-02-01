#ifndef MAX21100_H_
#define MAX21100_H_

#define USE_MAG

#define MAX21100_START_UP_DELAY_MS   100

// gyroscope, digits/dps
#define MAX21100_2000DPS_SENSITIVITY 	15.0f
#define MAX21100_1000DPS_SENSITIVITY 	30.0f
#define MAX21100_500DPS_SENSITIVITY 	60.0f 
#define MAX21100_250DPS_SENSITIVITY    120.0f

// accelerometer, digits/mG
#define MAX21100_4G_SENSITIVITY 	7.5f 
#define MAX21100_2G_SENSITIVITY 	15.0f

// magnetometer, digits/mGauss (not used)
#define HMC5883L_19_SENSITIVITY		0.82f
#define HMC5883L_25_SENSITIVITY		0.66f 
#define HMC5883L_40_SENSITIVITY		0.44f 

// MAX21100 Common Bank
#define MAX21100_WHO_AM_I      0x20            
#define MAX21100_REVISION_ID   0x21                                                                          
#define MAX21100_BANK_SELECT   0x22      
#define MAX21100_SYSTEM_STATUS 0x23
#define MAX21100_GYRO_X_H      0x24
#define MAX21100_GYRO_X_L      0x25
#define MAX21100_GYRO_Y_H      0x26
#define MAX21100_GYRO_Y_L      0x27
#define MAX21100_GYRO_Z_H      0x28
#define MAX21100_GYRO_Z_L      0x29
#define MAX21100_ACC_X_H       0x2A
#define MAX21100_ACC_X_L       0x2B
#define MAX21100_ACC_Y_H       0x2C
#define MAX21100_ACC_Y_L       0x2D
#define MAX21100_ACC_Z_H       0x2E
#define MAX21100_ACC_Z_L       0x2F
#define MAX21100_MAG_X_H       0x30
#define MAX21100_MAG_X_L       0x31
#define MAX21100_MAG_Y_H       0x32
#define MAX21100_MAG_Y_L       0x33
#define MAX21100_MAG_Z_H       0x34
#define MAX21100_MAG_Z_L       0x35
#define MAX21100_TEMP_H        0x36
#define MAX21100_TEMP_L        0x37
#define MAX21100_FIFO_COUNT    0x3C
#define MAX21100_FIFO_STATUS   0x3D
#define MAX21100_FIFO_DATA     0x3E
#define MAX21100_RST_REG       0x3F


// MAX21100 Bank 0
#define MAX21100_POWER_CFG     0x00
#define MAX21100_GYRO_CFG1     0x01
#define MAX21100_GYRO_CFG2     0x02
#define MAX21100_GYRO_CFG3     0x03
#define MAX21100_PWR_ACC_CFG   0x04
#define MAX21100_ACC_CFG1      0x05
#define MAX21100_ACC_CFG2      0x06
#define MAX21100_MAG_SLV_CFG   0x07
#define MAX21100_MAG_SLV_ADD   0x08
#define MAX21100_MAG_SLV_REG   0x09
#define MAX21100_MAG_MAP_REG   0x0A
#define MAX21100_I2C_MST_ADD   0x0B
#define MAX21100_I2C_MST_RW    0x0C
#define MAX21100_MAG_CORR_X_H 0x0D
#define MAX21100_MAG_CORR_X_L 0x0E
#define MAX21100_MAG_CORR_Y_H 0x0F
#define MAX21100_MAG_CORR_Y_L 0x10
#define MAX21100_MAG_CORR_Z_H 0x11
#define MAX21100_MAG_CORR_Z_L 0x12
#define MAX21100_DR_CFG        0x13
#define MAX21100_IO_CFG        0x14
#define MAX21100_I2C_PAD       0x15
#define MAX21100_I2C_CFG       0x16
#define MAX21100_FIFO_TH       0x17
#define MAX21100_FIFO_CFG      0x18
#define MAX21100_DSYNC_CFG     0x1A
#define MAX21100_DSYNC_CNT     0x1B
#define MAX21100_ITF_OTP       0x1C

// MAX21100 Bank 1
#define MAX21100_INT_REF_X     0x00
#define MAX21100_INT_REF_Y     0x01
#define MAX21100_INT_REF_Z     0x02
#define MAX21100_INT_DEB_X     0x03
#define MAX21100_INT_DEB_Y     0x04
#define MAX21100_INT_DEB_Z     0x05
#define MAX21100_INT_MSK_X     0x06
#define MAX21100_INT_MSK_Y     0x07
#define MAX21100_INT_MSK_Z     0x08
#define MAX21100_INT_MSK_AQ    0x09
#define MAX21100_INT_CFG1      0x0A
#define MAX21100_INT_CFG2      0x0B
#define MAX21100_INT_TM0       0x0C
#define MAX21100_INT_STS_UL    0x0D
#define MAX21100_INT_STS       0x0E
#define MAX21100_INT_MSK       0x0F
#define MAX21100_INT_SRC_SEL   0x17
#define MAX21100_SERIAL_5      0x1A
#define MAX21100_SERIAL_4      0x1B
#define MAX21100_SERIAL_3      0x1C
#define MAX21100_SERIAL_2      0x1D
#define MAX21100_SERIAL_1      0x1E
#define MAX21100_SERIAL_0      0x1F

// MAX21100 Bank 2 

#define MAX21100_QUAT0_H         0x00
#define MAX21100_QUAT0_L         0x01
#define MAX21100_QUAT1_H         0x02
#define MAX21100_QUAT1_L         0x03
#define MAX21100_QUAT2_H         0x04
#define MAX21100_QUAT2_L         0x05
#define MAX21100_QUAT3_H         0x06
#define MAX21100_QUAT3_L         0x07
#define MAX21100_GYRO_CORR_X_H   0x13
#define MAX21100_GYRO_CORR_X_L   0x14
#define MAX21100_GYRO_CORR_Y_H   0x15
#define MAX21100_GYRO_CORR_Y_L   0x16
#define MAX21100_GYRO_CORR_Z_H   0x17
#define MAX21100_GYRO_CORR_Z_L   0x18
#define MAX21100_ACC_CORR_X 	0x19
#define MAX21100_ACC_CORR_Y 	0x1A
#define MAX21100_ACC_CORR_Z 	0x1B
#define MAX21100_FUS_CFG0        0x1C
#define MAX21100_FUS_CFG1        0x1D
#define MAX21100_ODR_TRIM     	0x1F

#define GYRO_MAX_EXPECTED_OFFSET_1000DPS  100

#define STATUS_QUAT_ERR		0x80
#define STATUS_QUAT_DR		0x40

#define STATUS_MAG_DR		0x20
#define STATUS_MAG_ERR		0x10

#define STATUS_ACC_ERR		0x08
#define STATUS_ACC_DR		0x04

#define STATUS_GYRO_ERR		0x02
#define STATUS_GYRO_DR		0x01

enum Ascale {
  AFS_16G = 0,
  AFS_8G,
  AFS_4G,
  AFS_2G
};

enum Aodr { 
  AODR_2kHz = 0,
  AODR_1kHz,
  AODR_500Hz,
  AODR_250Hz,
  AODR_125Hz,  
  AODR_62_5Hz,
  AODR_31_25Hz    
};

enum Abw { 
  ABW_div48 = 0,  
  ABW_div22,
  ABW_div9,
  ABW_div3        
};

enum Gscale { 
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS         
};

enum Godr { 
  GODR_8kHz = 0,
  GODR_4kHz,
  GODR_2kHz,
  GODR_1kHz,
  GODR_500Hz, 
  GODR_250Hz,
  GODR_125Hz,
  GODR_62_5Hz,
  GODR_31_25Hz,
  GODR_15_625Hz,
  GODR_7_8125Hz,   
  GODR_3_90625Hz

};

enum Gbw { 
  GBW_2Hz = 0,
  GBW_4Hz,
  GBW_6Hz,
  GBW_8Hz,
  GBW_10Hz,
  GBW_14Hz,
  GBW_22Hz,
  GBW_32Hz,
  GBW_50Hz,
  GBW_75Hz,
  GBW_100Hz,
  GBW_150Hz,
  GBW_200Hz,
  GBW_250Hz,
  GBW_300Hz,
  GBW_400Hz 
};

enum powerMode {  // power modes without using DSYNC enable
  powerDownmode = 0,
  gyrosleepmode,
  gyrospotmode,
  gyronormalmode,
  notused0,
  notused1,
  notused2,
  notused3,
  accelspotmode, 
  notused4,
  notused5,
  notused6,
  accelnormalmode,
  accelnormalgyrosleepmode,
  accelnormalgyrospotmode,
  accelnormalgyronormalmode  // 0x0F
};

#define    	AUTO_INCR  		0x00
#define 	NO_AUTO_INCR	0x40

extern int16_t gxBias;
extern int16_t gyBias;
extern int16_t gzBias;

extern int16_t axBias;
extern int16_t ayBias;
extern int16_t azBias;

extern int16_t mxBias;
extern int16_t myBias;
extern int16_t mzBias;
extern int16_t  mxSensitivity;
extern int16_t  mySensitivity;
extern int16_t  mzSensitivity;

void MAX21100_Write8(uint8_t addr, uint8_t val);
uint8_t MAX21100_Read8(uint8_t addr);
void MAX21100_ReadBuf(uint8_t addr, uint8_t autoIncrement, int numBytes, uint8_t* pBuf);
void MAX21100_InitCalibrationParams(void);
int MAX21100_CalibrateGyro(void);
void MAX21100_CalibrateAccel(void);
int MAX21100_CalibrateMag(void);

void MAX21100_ConfigureGyroAccel(void);
void MAX21100_GetGyroAccelData(float* pGyroData, float* pAccelData);
//void MAX21100_GetAcceleration(int16_t* ax, int16_t* ay, int16_t* az);
//void MAX21100_GetRotation(int16_t* gx, int16_t* gy, int16_t* gz);
//void MAX21100_GetMag(int16_t* mx, int16_t* my, int16_t* mz);
void MAX21100_GetVector(uint8_t startAddr, int16_t* px, int16_t* py, int16_t* pz);


#ifdef USE_MAG
void MAX21100_ConfigureGyroAccelMag(void);
void MAX21100_GetGyroAccelMagData(float* pGyroData, float* pAccelData, float* pMagData);
void MAX21100_GetGyroAccelMagDataRaw(int16_t* pGyroData, int16_t* pAccelData, int16_t* pMagData);
#endif


#endif
