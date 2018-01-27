#include "common.h"
#include "config.h"
#include "gps.h"
#include "flashlog.h"

const uint8_t GpsUBXNAVPVTHdr[] = {0xB5,0x62,0x01,0x07, 0x5C, 0x00};

int  NumValidHdrBytes;
int  GpsState;
int  GpsPktRcvIndex;
GPS_PKT  GpsPkt[GPS_MAX_PKTS];


int GpsHeightm;
									   
#define TAG "Gps"

void gps_Config() {
	NumValidHdrBytes = 0;
	GpsPktRcvIndex = 0;
	
	GpsState = GPS_STATE_IDLE;
	for (int cnt = 0; cnt < GPS_MAX_PKTS; cnt++){
		GpsPkt[cnt].status = GPS_PKT_FREE;
		}
	}


int gps_PacketChecksum(uint8_t* pBuf, int numBytes) {
	return 1;
	}


void gps_StateMachine(uint8_t byte)  {
		switch(GpsState) {
	        case GPS_STATE_IDLE :
	        default :
	   		if (byte == GpsUBXNAVPVTHdr[NumValidHdrBytes]) {
	    		NumValidHdrBytes++;
	   			}
	   		else {
	  			NumValidHdrBytes = 0; // reset 
	  			}  
	   		if (NumValidHdrBytes == 6)	{ // header found
				NumValidHdrBytes = 0;			
	  			int cnt = 0; 
				// look for a free packet index
				while ((GpsPkt[GpsPktRcvIndex].status != GPS_PKT_FREE) && (cnt < GPS_MAX_PKTS)){
					cnt++;
					GpsPktRcvIndex++;
					if (GpsPktRcvIndex >= GPS_MAX_PKTS) GpsPktRcvIndex = 0;
					}
				if (cnt < GPS_MAX_PKTS)	{
					GpsPkt[GpsPktRcvIndex].status = GPS_PKT_RCV;
					GpsPkt[GpsPktRcvIndex].receivedBytes = 0;
					GpsState = GPS_STATE_PKTRCV;
					}
				else {
					ESP_LOGE(TAG, "packet overflow");
					}
	            }
	        break;
	        			
			
	        case GPS_STATE_PKTRCV :
	        GpsPkt[GpsPktRcvIndex].navpvt.buf[GpsPkt[GpsPktRcvIndex].receivedBytes++] = byte;
			if (GpsPkt[GpsPktRcvIndex].receivedBytes == NAV_PVT_PKT_NUM_BYTES) {
				if (!gps_PacketChecksum(GpsPkt[GpsPktRcvIndex].navpvt.buf, NAV_PVT_PKT_NUM_BYTES )) {
	            	GpsPkt[GpsPktRcvIndex].receivedBytes = 0;
	            	GpsPkt[GpsPktRcvIndex].status = GPS_PKT_FREE;
	            	GpsState = GPS_STATE_IDLE;
					ESP_LOGE(TAG,"packet checksum error");
	            	}
				else {
					   gps_UpdateLogRecord(GpsPktRcvIndex);
                  GpsHeightm = GpsPkt[GpsPktRcvIndex].navpvt.pkt.heightMSLmm/1000; // for lcd display
	            	GpsPkt[GpsPktRcvIndex].receivedBytes = 0;
	           		GpsPkt[GpsPktRcvIndex].status = GPS_PKT_FREE; 
	           		GpsState = GPS_STATE_IDLE;
					}
				}            
	        break;					   
	        }
    }


void gps_UpdateLogRecord(int pktIndex) {
	if (FlashLogMutex) {
		if (xSemaphoreTake( FlashLogMutex, portMAX_DELAY )) {
			FlashLogRecord.hdr.gpsFlags = GpsPkt[pktIndex].navpvt.pkt.fixType;
			FlashLogRecord.gps.timeOfWeekmS = GpsPkt[pktIndex].navpvt.pkt.timeOfWeekmS;
			FlashLogRecord.gps.heightMSLmm = GpsPkt[pktIndex].navpvt.pkt.heightMSLmm;
			FlashLogRecord.gps.vertAccuracymm = GpsPkt[pktIndex].navpvt.pkt.vertAccuracymm;
			FlashLogRecord.gps.velNorthmmps = GpsPkt[pktIndex].navpvt.pkt.velNorthmmps;
			FlashLogRecord.gps.velEastmmps = GpsPkt[pktIndex].navpvt.pkt.velEastmmps;
			FlashLogRecord.gps.velDownmmps = GpsPkt[pktIndex].navpvt.pkt.velDownmmps;
			FlashLogRecord.gps.velAccuracymmps = GpsPkt[pktIndex].navpvt.pkt.speedAccuracymmps;
			FlashLogRecord.gps.lonDeg7 = GpsPkt[pktIndex].navpvt.pkt.lonDeg7;
			FlashLogRecord.gps.latDeg7 = GpsPkt[pktIndex].navpvt.pkt.latDeg7;	
			xSemaphoreGive( FlashLogMutex );
			}
		}
	}
