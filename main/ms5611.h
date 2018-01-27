#ifndef MS5611_H_
#define MS5611_H_


//#define MS5611_TEST

// 10mS enough, but we're using vTaskDelay with a tick period of 10mS
// so 2 ticks are required to ensure at least one tick delay !
#define MS5611_SAMPLE_PERIOD_MS         20

#define MS5611_READ_TEMPERATURE 		11
#define MS5611_READ_PRESSURE			22

#define MS5611_CMD_RESET      	0x1E
#define MS5611_CMD_CONVERT_D1 	0x40
#define MS5611_CMD_CONVERT_D2 	0x50
#define MS5611_CMD_ADC_READ   	0x00
#define MS5611_CMD_ADC_4096 	0x08

void 	MS5611_TriggerPressureSample(void);
void 	MS5611_TriggerTemperatureSample(void);
uint32_t  	MS5611_ReadSample(void);
void 	MS5611_AveragedSample(int nSamples);	
void  	MS5611_CalculateTemperatureCx10(void);
float  	MS5611_CalculatePressurePa(void);
void 	MS5611_CalculateSensorNoisePa(void);
int 	MS5611_Configure(void);
int 	MS5611_SampleStateMachine(void);
void 	MS5611_InitializeSampleStateMachine(void);
float  	MS5611_Pa2Cm(float pa);
void 	MS5611_Test(int nSamples);	
uint8_t MS5611_CRC4(uint8_t prom[] );
int 	MS5611_ReadPROM(void);
void 	MS5611_GetCalibrationParameters(void);
void 	MS5611_Reset(void);

extern	float zCmAvg;
extern	float zCmSample;
extern	float paSample;
extern	int  celsiusSample;
extern	int zGood;

#endif // MS5611_H_
