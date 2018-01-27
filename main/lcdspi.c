#include "common.h"
#include "config.h"
#include "driver/spi_master.h"
#include "lcdspi.h"


spi_device_handle_t spiLCD;


static void lcdspi_configBus(int pSCLK, int pMOSI) {
   esp_err_t ret;
   spi_bus_config_t buscfg={
       .miso_io_num = -1,
       .mosi_io_num = pMOSI,
       .sclk_io_num = pSCLK,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1
   	};
   ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
   assert(ret == ESP_OK);
   }


static void lcdspi_attachDevice(int clockFreqHz, int pCS, spi_device_handle_t* pDeviceHandle) {
   esp_err_t ret;
   spi_device_interface_config_t devcfg={
       .clock_speed_hz = clockFreqHz,
       .mode = SPIMODE0,                    
       .spics_io_num = pCS,               
       .queue_size = 1,
   	};
   ret = spi_bus_add_device(HSPI_HOST, &devcfg, pDeviceHandle);
   assert(ret == ESP_OK);
   }


void lcdspi_config() {
   lcdspi_configBus(pinLCDSCLK,pinLCDMOSI);
   lcdspi_attachDevice(LCDSPI_CLKFREQ_HZ, pinLCDCS, &spiLCD);
   }


void lcdspi_write9( uint16_t val) {
   esp_err_t ret;
   spi_transaction_t t;
   memset(&t, 0, sizeof(t));
   uint8_t b[2];
   b[0] = val>>8;
   b[1] = val&0xff;      
   t.length = 9;   // D/C + data, left justified in two byte array
   t.tx_buffer = b;              
   ret = spi_device_transmit(spiLCD, &t); 
   assert(ret == ESP_OK);            
  }




/*
void spi_write(spi_device_handle_t spi, const uint8_t b) {
   esp_err_t ret;
   spi_transaction_t t;
   memset(&t, 0, sizeof(t));       
   t.length = 8;                   
   t.tx_buffer = &b;               
   ret = spi_device_transmit(spi, &t);
   assert(ret == ESP_OK);            
  }

uint8_t spi_read(spi_device_handle_t spi) {
  esp_err_t ret;
  spi_transaction_t t;
  const uint8_t txb = 0;
  uint8_t rxb;
  memset(&t, 0, sizeof(t));      
  t.length = 8;                  
  t.tx_buffer = &txb;   //transmit dummy
  t.rx_buffer = &rxb;
  ret = spi_device_transmit(spi, &t);
  assert(ret == ESP_OK);
  return rxb;
 }

uint8_t spi_transfer(spi_device_handle_t spi, const uint8_t txb) {
   esp_err_t ret;
   spi_transaction_t t;
   uint8_t rxb;
   memset(&t, 0, sizeof(t));
   t.length = 8;            
   t.tx_buffer = &txb;    
   t.rx_buffer = &rxb;
   ret = spi_device_transmit(spi, &t);
   assert(ret == ESP_OK);           
   return rxb;
  }
*/

