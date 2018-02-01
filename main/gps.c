#include "common.h"
#include "config.h"
#include "driver/uart.h"
#include "gps.h"
#include "flashlog.h"

const uint8_t GpsUBXNAVPVTHdr[] = {0xB5,0x62,0x01,0x07, 0x5C, 0x00};

static int  NumValidHdrBytes;
static int  GpsState;
static int PktReceivedBytes;

static NAV_PVT NavPvt;
volatile int GpsHeightm;// for lcd display to compare with baro before starting logging
									   
static int UartNum;
static uint8_t UartRcvBuffer[UART_RX_BUFFER_SIZE];

									   
#define TAG "Gps"

static void ubx_Checksum(uint8_t* buffer, int numbytes, uint8_t* pcka, uint8_t* pckb) {
   uint8_t cka, ckb;
   cka = ckb = 0;
   for (int inx = 0; inx < numbytes; inx++) {
      cka += buffer[inx];
      ckb += cka;
      }
   *pcka = cka;
   *pckb = ckb;
   }

int gps_Config() {
   esp_err_t err;
   UartNum = GPS_UART_NUM;
   uart_config_t uart_config = {
     .baud_rate = 115200,
     .data_bits = UART_DATA_8_BITS,
     .parity = UART_PARITY_DISABLE,
     .stop_bits = UART_STOP_BITS_1,
     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
   };
   err = uart_param_config(UartNum, &uart_config);
   if (err != ERR_OK) {
      ESP_LOGE(TAG, "error configuring uart params");
      return -1;
      }
   err = uart_set_pin(UartNum, pinGpsTXD, pinGpsRXD, pinGpsRTS, pinGpsCTS);
   if (err != ERR_OK) {
      ESP_LOGE(TAG, "error configuring uart pins");
      return -2;
      }
   err = uart_driver_install(UartNum, UART_RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
   if (err != ERR_OK) {
      ESP_LOGE(TAG, "error installing uart driver");
      return -3;
      }

	NumValidHdrBytes = 0;
	PktReceivedBytes = 0;
	GpsState = GPS_STATE_IDLE;
   return 0;
	}


int gps_PacketChecksum(uint8_t* pBuf, int numBytes) {
	return 1;
	}


void gps_StateMachine()  {
   int len = uart_read_bytes(UartNum, UartRcvBuffer, UART_RX_BUFFER_SIZE, 20 / portTICK_RATE_MS);
   if (len) {
	   for (int inx = 0; inx < len; inx++) { 
         uint8_t rcvb = UartRcvBuffer[inx];
		   switch(GpsState) {
	         case GPS_STATE_IDLE :
	         default :
	   		if (rcvb == GpsUBXNAVPVTHdr[NumValidHdrBytes]) {
	    		   NumValidHdrBytes++;
	   			}
	   		else {
	  			   NumValidHdrBytes = 0; // reset 
	  			   }  
	   		if (NumValidHdrBytes == 6)	{ // header found
				   NumValidHdrBytes = 0;			
				   PktReceivedBytes = 0;
			      GpsState = GPS_STATE_PKTRCV;
					}
	        break;
	        			
			
	        case GPS_STATE_PKTRCV :
	        NavPvt.buffer[PktReceivedBytes++] = rcvb;
			   if (PktReceivedBytes == NAV_PVT_PKT_NUM_BYTES) {
				if (!gps_PacketChecksum(NavPvt.buffer, NAV_PVT_PKT_NUM_BYTES )) {
	            PktReceivedBytes = 0;
	            GpsState = GPS_STATE_IDLE;
					ESP_LOGE(TAG,"packet checksum error");
	            }
				else {
               GpsHeightm = NavPvt.pkt.heightMSLmm/1000; // for lcd display
               gps_UpdateLogRecord();
	            PktReceivedBytes = 0;
	           	GpsState = GPS_STATE_IDLE;
					}
				}            
	        break;					   
	        }
         }
      }
   } 


void gps_UpdateLogRecord() {
	if (FlashLogMutex) {
		if (xSemaphoreTake( FlashLogMutex, portMAX_DELAY )) {
			FlashLogRecord.hdr.gpsFlags = NavPvt.pkt.fixType;
			FlashLogRecord.gps.timeOfWeekmS = NavPvt.pkt.timeOfWeekmS;
			FlashLogRecord.gps.heightMSLmm = NavPvt.pkt.heightMSLmm;
			FlashLogRecord.gps.vertAccuracymm = NavPvt.pkt.vertAccuracymm;
			FlashLogRecord.gps.velNorthmmps = NavPvt.pkt.velNorthmmps;
			FlashLogRecord.gps.velEastmmps = NavPvt.pkt.velEastmmps;
			FlashLogRecord.gps.velDownmmps = NavPvt.pkt.velDownmmps;
			FlashLogRecord.gps.velAccuracymmps = NavPvt.pkt.speedAccuracymmps;
			FlashLogRecord.gps.lonDeg7 = NavPvt.pkt.lonDeg7;
			FlashLogRecord.gps.latDeg7 = NavPvt.pkt.latDeg7;	
			xSemaphoreGive( FlashLogMutex );
			}
		}
	}
