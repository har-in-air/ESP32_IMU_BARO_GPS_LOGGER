#include "common.h"
#include "config.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/esp32-hal-gpio.h"

#include "spihw.h"
#include "spiflash.h"
#include "gps.h"
#include "ms5611.h"
#include "max21100.h"
#include "cct.h"
#include "imu.h"
#include "lcd1202.h"
#include "font_6x8.h"
#include "myadc.h"
#include "flashlog.h"
#include "mwifi.h"

#define TAG  "main"

uint16_t Btn0State;
uint16_t Btn15State;
volatile int Btn0Pressed = 0;
volatile int Btn15Pressed = 0;

int IsServer = 0;

volatile int ledState = 0;
float yaw, pitch, roll;


static intr_handle_t s_timer_handle;

volatile SemaphoreHandle_t imuSemaphore;
	
void pinConfig() {	
	pinMode(pinBTN0, INPUT); // already pulled up by 10K resistor
	pinMode(pinBTN15, INPUT_PULLUP); // use internal pullup
	
	pinMode(pinFlashCS, OUTPUT);
	pinMode(pinImuCS, OUTPUT);
	pinMode(pinBaroCS, OUTPUT);
	FLASHCS_1();
	BAROCS_1();
	IMUCS_1();
	
	pinMode(pinLCDCS, OUTPUT);
	LCDCS_HI();
	pinMode(pinLCDRST, OUTPUT);
	LCDRST_HI();

	pinMode(pinLED, OUTPUT);
	LED_OFF();
	
	pinMode(pinDRDYINT, INPUT);
	}


static void timer_isr(void* arg){
   TIMERG0.int_clr_timers.t0 = 1;
   TIMERG0.hw_timer[0].config.alarm_en = 1;
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   xSemaphoreGiveFromISR(imuSemaphore, &xHigherPriorityTaskWoken);
  	if( xHigherPriorityTaskWoken == pdTRUE) {
     	portYIELD_FROM_ISR(); // this wakes up imubaro_task immediately
	   }
   }


void init_timer(void){	
   timer_config_t config = {
      .alarm_en = true,
      .counter_en = false,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = true,
      .divider = 80   // timer runs off 80mhz bus clock, so 1 us per tick 
      };
    
   timer_init(TIMER_GROUP_0, TIMER_0, &config);
   timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
   timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1000); // 1000ticks == 1ms
   timer_enable_intr(TIMER_GROUP_0, TIMER_0);
   timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr, NULL, 0, &s_timer_handle);

	imuSemaphore = xSemaphoreCreateBinary();
	timer_start(TIMER_GROUP_0, TIMER_0);
	}

	
static void imubaro_task() {
	float gyrodps[3],accelmG[3],mag[3];
   float gxNEDdps, gyNEDdps, gzNEDdps, axNEDmG, ayNEDmG, azNEDmG, mxNED, myNED, mzNED;

	ESP_LOGI(TAG, "imubaro task started");
	SpiHW_SetFreq(SPI_CLOCK_LOW_FREQHZ);

	uint8_t maxID = MAX21100_Read8(MAX21100_WHO_AM_I);
   if (maxID != 0xB2) { 
	   ESP_LOGE(TAG, "MAX21100 device ID [expected B2] = %02X", maxID);
		lcd_printf(0,24, "Bad IMU ");
		while (1) {
			LED_ON();
			delayMs(100);
			LED_OFF();
			delayMs(100);
			}
      }

	MAX21100_InitCalibrationParams();
	MAX21100_ConfigureGyroAccelMag();
	LED_ON();
   int counter = 8;
   while (counter--) {
   	lcd_printf(0,24, "Gyro calib in %ds", counter+1);
      delayMs(1000);	
		}
	LED_OFF();

	MAX21100_CalibrateGyro();
   lcd_printf(0,24,"                ");  
  	lcd_printf(0,24, "x %d y %d z %d", gxBias, gyBias, gzBias);
   delayMs(3000);	
   lcd_printf(0,24,"                ");  
	//MAX21100_CalibrateAccel();

	if (!MS5611_Configure()) {
		ESP_LOGE(TAG, "Failure configuring MS5611");
		lcd_printf(0,24, "Bad Baro");
		while (1) {
			LED_ON();
			delayMs(100);
			LED_OFF();
			delayMs(100);
			}
		}	
   MS5611_AveragedSample(4);
	int altMeters = (int)(zCmAvg/100.0f);
	int tCelsius = (int)celsiusSample;
   ESP_LOGI(TAG,"Altitude %dm Temperature %dC", altMeters, tCelsius);
	MS5611_InitializeSampleStateMachine();
	delayMs(MS5611_SAMPLE_PERIOD_MS);
  	lcd_printf(0,40, "  BTN0 FOR LOG");

	SpiHW_SetFreq(SPI_CLOCK_MAX_FREQHZ);

#if 0
   uint32_t eus;
   cct_SetMarker();
   MAX21100_Read8(MAX21100_SYSTEM_STATUS);
	MAX21100_GetGyroAccelMagData(gyrodps, accelmG, mag);			
   eus = cct_ElapsedUs();
	ESP_LOGI(TAG, "max21100 read status + read data = %dus", eus); // ~110 us
#endif

	int drdyCounter = 0;
   int baroCounter = 0;	
	while (1) {		
		xSemaphoreTake(imuSemaphore, portMAX_DELAY);
      baroCounter++; // ticks every 1mS
		uint8_t status = MAX21100_Read8(MAX21100_SYSTEM_STATUS);
		uint8_t gyrodr = ((status & STATUS_GYRO_DR) && !(status & STATUS_GYRO_ERR)) ? 1 : 0;
		// imu output data rate is ~500Hz (~2mS interval). We check every 1mS to make sure we don't
		// miss a sample
		if (gyrodr) {
   		drdyCounter++; // ticks every 2mS
			MAX21100_GetGyroAccelMagData(gyrodps, accelmG, mag);			
         // xned = silkscreen y = forward = north
         // yned = silkscreen x = right = east
         // zned = down
			gxNEDdps = gyrodps[1];
			gyNEDdps = gyrodps[0];
			gzNEDdps = -gyrodps[2];
			
			axNEDmG = -accelmG[1];
			ayNEDmG = -accelmG[0];
			azNEDmG = accelmG[2];
			
			mxNED = mag[1];
			myNED = mag[0];
			mzNED = -mag[2];
#if 0
         float asqd = axNEDmG*axNEDmG + ayNEDmG*ayNEDmG + azNEDmG*azNEDmG;
			int bUseAccel = ((asqd > 490000.0f) && (asqd < 1690000.0f)) ? 1 : 0;	
         int bUseMag = 0;
			imu_MahonyAHRSupdate9DOF(bUseAccel, bUseMag,0.002f, DEG2RAD(gxNEDdps), DEG2RAD(gyNEDdps), DEG2RAD(gzNEDdps), axNEDmG, ayNEDmG, azNEDmG, mxNED, myNED, mzNED);
#endif
	
			if (FlashLogMutex) {
				if (xSemaphoreTake( FlashLogMutex, portMAX_DELAY )) {               
					FlashLogRecord.imu.gxNEDdps = gxNEDdps;
					FlashLogRecord.imu.gyNEDdps = gyNEDdps;
					FlashLogRecord.imu.gzNEDdps = gzNEDdps;
					FlashLogRecord.imu.axNEDmG = axNEDmG;
					FlashLogRecord.imu.ayNEDmG = ayNEDmG;
					FlashLogRecord.imu.azNEDmG = azNEDmG;
					FlashLogRecord.imu.mxNED = mxNED;
					FlashLogRecord.imu.myNED = myNED;
					FlashLogRecord.imu.mzNED = mzNED;
					if (IsLogging) {
						LED_ON();
						FlashLog_WriteRecord(&FlashLogRecord); // worst case 80byes : ~130uS intra-page, ~210uS across page boundary
                  memset(&FlashLogRecord, 0, sizeof(FLASHLOG_RECORD));
						LED_OFF();
						}
					xSemaphoreGive( FlashLogMutex );
					}			
				}

			if (drdyCounter >= 500) { // once every second
				drdyCounter = 0; 
#if 0
				imu_Quaternion2YawPitchRoll(q0, q1, q2, q3, &yaw, &pitch, &roll);
				ESP_LOGI(TAG,"ypr %4d %3d %3d, alt %d", (int)yaw, (int)pitch, (int)roll, (int)zCmSample);
				//lcd_printf(0,40,"ypr %4d %3d %3d", (int)yaw, (int)pitch, (int)roll);
				//ESP_LOGI(TAG,"Ax  %.1f  Ay  %.1f   Az  %.1f", accelmG[0], accelmG[1], accelmG[2]);
				//ESP_LOGI(TAG,"Gx  %.1f  Gy  %.1f   Gz  %.1f", gyrodps[0], gyrodps[1], gyrodps[2]);
				//ESP_LOGI(TAG,"Mx  %.1f  My  %.1f   Mz  %.1f",magraw[0], magraw[1], magraw[2]);
				//lcd_printf(0,0,"%4d %4d %4d", (int)accelmG[0], (int)accelmG[1], (int)accelmG[2]);
				//lcd_printf(0,16,"%4d %4d %4d", (int)gyrodps[0], (int)gyrodps[1], (int)gyrodps[2]);
				//lcd_printf(0,32,"%4d %4d %4d", (int)mxmin, (int)mymin, (int)mzmin);
				//lcd_printf(0,40,"%4d %4d %4d", (int)mxmax, (int)mymax, (int)mzmax);
				//lcd_printf(0,32,"%4d %4d %4d", (int)magraw[0], (int)magraw[1], (int)magraw[2]);
				//lcd_printf(0,0,"%4d %4d %4d", (int)axNED, (int)ayNED, (int)azNED);
				//lcd_printf(0,16,"%4d %4d %4d", (int)gxNED, (int)gyNED, (int)gzNED);
				//lcd_printf(0,32,"%4d %4d %4d", (int)mxNED, (int)myNED, (int)mzNED);
#endif
				}	
			} // if (gyrodr)
      if (baroCounter >= 10) {
         baroCounter = 0; // 10mS sample interval
		   // temperature + pressure => 20mS for a new altitude sample
		   int zMeasurementAvailable = MS5611_SampleStateMachine();
		   if (zMeasurementAvailable) {
			   if (FlashLogMutex) {
			   	if ( xSemaphoreTake( FlashLogMutex, portMAX_DELAY )) {
                  FlashLogRecord.hdr.baroFlags = 1;
					   FlashLogRecord.baro.heightMSLcm = zCmSample;
					   xSemaphoreGive( FlashLogMutex );
					   }
				   }
			   }  
		   } // baroCounter
      } //while 1
	} 
	
	
static void gps_task() {
	ESP_LOGI(TAG, "gps task started");
    const int uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
		};
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, pinGpsTXD, pinGpsRXD, pinGpsRTS, pinGpsCTS);
    uart_driver_install(uart_num, UART_RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    gps_Config();
    uint8_t* rcvBuffer = (uint8_t*) malloc(UART_RX_BUFFER_SIZE);
    while(1) {
       int len = uart_read_bytes(uart_num, rcvBuffer, UART_RX_BUFFER_SIZE, 20 / portTICK_RATE_MS);
		 if (len) {
		   for (int inx = 0; inx < len; inx++) { 
		      gps_StateMachine(rcvBuffer[inx]);
				}
		   }
	   }//while
   }

	
	
void app_main() {
	nvs_flash_init();
	pinConfig();
	ESP_LOGI(TAG, "IMU-BARO-GPS data logger compiled on %s at %s", __DATE__, __TIME__);

	delayMs(100);
	lcd_init();
   lcd_home();
   lcd_setFont(Font_6x8);

   uint32_t batVoltage = (uint32_t)(4.44f*adc_Sample()); // resistor ratio is 4.33,scaling error = 1.02
	ESP_LOGI(TAG, "Battery Voltage = %d.%03dmV", batVoltage/1000, batVoltage%1000);
	lcd_printf(0,0,"Bat %d.%03dV", batVoltage/1000,batVoltage%1000);

	SpiHW_Config(pinSCLK, pinMOSI, pinMISO, SPI_CLOCK_LOW_FREQHZ);

	if (!FlashLog_Init()) {
		ESP_LOGE(TAG, "Spi flash log error");
		lcd_printf(0,8,"Flash Error");		
		while (1) {
			LED_ON();
			delayMs(100);
			LED_OFF();
			delayMs(100);
			}
		}
	ESP_LOGI(TAG, "Flash Free Address %08d", FlashLogFreeAddress);
	lcd_printf(0,8,"Flash %08d",  FlashLogFreeAddress);

#if 0
	SpiHW_Config(pinSCLK, pinMOSI, pinMISO, SPI_CLOCK_MAX_FREQHZ);

   uint32_t eus;
   cct_SetMarker();
	FlashLog_WriteRecord(&FlashLogRecord); 
   eus = cct_ElapsedUs();
	ESP_LOGI(TAG, "Flashlog write record = %dus", eus);
   cct_SetMarker();
	FlashLog_WriteRecord(&FlashLogRecord); 
   eus = cct_ElapsedUs();
	ESP_LOGI(TAG, "Flashlog write record = %dus", eus);
   cct_SetMarker();
	FlashLog_WriteRecord(&FlashLogRecord); 
   eus = cct_ElapsedUs();
	ESP_LOGI(TAG, "Flashlog write record = %dus", eus);
   cct_SetMarker();
	FlashLog_WriteRecord(&FlashLogRecord); 
   eus = cct_ElapsedUs();
	ESP_LOGI(TAG, "Flashlog write record = %dus", eus);
#endif
	ESP_LOGI(TAG,"Press BTN15 within 4 seconds to erase flash");
	LED_ON();
   int bEraseFlag = 0;
   int counter = 400;
   while (counter--) {
   	lcd_printf(0,16, "BTN15 Erase %ds  ",(counter+50)/100);
	   if (!BTN15()) {
		   ESP_LOGI(TAG,"BTN15 PRESSED");
         bEraseFlag = 1;
         break;
         }
      delayMs(10);	
		}
	LED_OFF();
   if (bEraseFlag) {
		ESP_LOGI(TAG, "Erasing flash ...");
		lcd_printf(0,16, "Erasing...      ");
		FlashLog_Erase();
		ESP_LOGI(TAG, "Done");
		lcd_printf(0,8,"Flash %08d", FlashLogFreeAddress);		
		lcd_printf(0,16, "Flash Erased    ");
      }
	
   delayMs(2000);
		
	ESP_LOGI(TAG,"Press BTN0 within 4 seconds to start download server");
	LED_ON();
   IsServer = 0;
   counter = 400;
   while (counter--) {
   	lcd_printf(0,16, "BTN0 Server %ds  ",(counter+50)/100);
	   if (!BTN0()) {
		   ESP_LOGI(TAG,"BTN0 PRESSED");
		   IsServer = 1;
         break;
         }
      delayMs(10);
		}
	LED_OFF();

   if (IsServer) {
		tcpip_adapter_init();
		ESP_ERROR_CHECK(esp_event_loop_init(esp32_wifi_eventHandler, NULL));
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK(esp_wifi_init(&cfg));
		ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
      lcd_clear();
      lcd_printf(0,0,"Data Server Mode");
      lcd_printf(0,16,"AP : ESP32Logger");
      lcd_printf(0,32,"Open url");
      lcd_printf(0,40,"192.168.4.1/data");
		ESP_LOGI(TAG, "Wifi Data server mode selected");
      wifiInitAccessPoint();
   	delayMs(1000);
		xTaskCreatePinnedToCore(&http_server, "http_server", 4096, NULL, 5, NULL,1);		
      }
   else {	
		lcd_printf(0,16,"Data Logger Mode");
      ESP_LOGI(TAG, "Logger Mode started");
      delayMs(2000);
		init_timer();		
		xTaskCreatePinnedToCore(&imubaro_task, "imubarotask", 2048, NULL, 20, NULL, 0);
		xTaskCreatePinnedToCore(&gps_task, "gpstask", 2048, NULL, 20, NULL, 1);
		}
	
	int btnCounter = 0;
	while(1) {
      // debounce BTN0
		Btn0State = ((Btn0State<<1) | ((uint16_t)BTN0()) );
		if ((Btn0State | 0xFFF0) == 0xFFF8) {
			Btn0Pressed = 1;
			}    
		//Btn15State = ((Btn15State<<1) | ((uint16_t)BTN15()));
		//if ((Btn15State  | 0xFFF0)== 0xFFF8) {
		//	Btn15Pressed = 1;
		//	}
		if (Btn0Pressed) {
			Btn0Pressed = 0;
			ESP_LOGI(TAG,"BTN0 PRESSED");
			if (!IsLogging) {
            lcd_clear(); // when logging, make it clear that no data is being updated on screen
				int timer = 5;
				while (timer--) {
					lcd_printf(0,32, "LOG START %ds", timer+1);
					delayMs(1000);
					}
				}
			IsLogging = !IsLogging;
			lcd_printf(0,32, IsLogging ? "LOG STARTED " : "LOG STOPPED ");
			ESP_LOGI(TAG,"Logging %s", IsLogging ? "ON" : "OFF");
			}
		// display gps and baro altitude so you can check that gps has a good 3D fix	
		// before starting logging
		if ((!IsServer) && (!IsLogging)) {			
			btnCounter++;
			if (btnCounter >= 33) {
				btnCounter = 0;
				lcd_printf(0,16,"G %4dm  B %4dm", GpsHeightm, ((int)zCmSample)/100);
				}
			}
			
		delayMs(30);
		}		
	}
