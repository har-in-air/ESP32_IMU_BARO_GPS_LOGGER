#ifndef CONFIG_H_
#define CONFIG_H_

#include "sdkconfig.h"
#include "driver/gpio.h"

// Only the gps module tx pin is connected to uart1 rx pin
// The ublox gps module was configured offline with u-center v8.26
// to generate UBX binary protocol NAV_PVT packets @ 10Hz, 115200baudrate

#define pinGpsTXD  (-1)
#define pinGpsRXD  (22)
#define pinGpsRTS		(-1)
#define pinGpsCTS		(-1)

#define UART_RX_BUFFER_SIZE	512

// imu, baro, flash use VSPI IOMux compatible pins

#define pinSCLK      18
#define pinMOSI	   23
#define pinMISO	   19

#define pinImuCS	   17 
#define pinBaroCS	   16
#define pinFlashCS   5

#define IMUCS_1() 	{GPIO.out_w1ts = (1 << pinImuCS);}
#define IMUCS_0() 	{GPIO.out_w1tc = (1 << pinImuCS);}
#define BAROCS_1() 	{GPIO.out_w1ts = (1 << pinBaroCS);}
#define BAROCS_0() 	{GPIO.out_w1tc = (1 << pinBaroCS);}
#define FLASHCS_1()  {GPIO.out_w1ts = (1 << pinFlashCS);}
#define FLASHCS_0()  {GPIO.out_w1tc = (1 << pinFlashCS);}


// low speed for configuring the sensors, high speed for readout
#define SPI_CLOCK_LOW_FREQHZ 500000
#define SPI_CLOCK_MAX_FREQHZ 10000000

#define pinBTN0		0
#define pinBTN15	   15

#define BTN0()	      ((GPIO.in & (1 << pinBTN0)) ? 1 : 0)
#define BTN15()	   ((GPIO.in & (1 << pinBTN15)) ? 1 : 0)

// lcd uses HSPI IOMux compatible pins
#define pinLCDSCLK	14
#define pinLCDMOSI	13 
#define pinLCDCS	   26
#define pinLCDRST    25

#define LCDCS_HI() 	{GPIO.out_w1ts = (1 << pinLCDCS);}
#define LCDCS_LO() 	{GPIO.out_w1tc = (1 << pinLCDCS);}
#define LCDRST_HI() 	{GPIO.out_w1ts = (1 << pinLCDRST);}
#define LCDRST_LO() 	{GPIO.out_w1tc = (1 << pinLCDRST);}

#define LCDSPI_CLKFREQ_HZ 1000000

#define pinLED		   27

#define LED_ON() 		{GPIO.out_w1ts = (1 << pinLED);}
#define LED_OFF()		{GPIO.out_w1tc = (1 << pinLED);}

// ideally want to use the data ready interrupt from imu module,
// but it's not working, maybe soldering issue, can't see the signal
// on the pcb pin. so we're polling for the samples
#define pinDRDYINT	21

#define FLASH_W25Q128  // 128Mbit 104MHz
//#define FLASH_W25Q16   // 16Mbit 104MHz


#endif
