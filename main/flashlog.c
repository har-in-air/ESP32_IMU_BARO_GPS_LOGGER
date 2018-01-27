#include "common.h"
#include "config.h"
#include "flashlog.h"

uint32_t          FlashLogFreeAddress  = 0;
FLASHLOG_RECORD   FlashLogRecord;
SemaphoreHandle_t FlashLogMutex = NULL;
int               IsLogging = 0;

#define TAG "SpiFlashLog"

void FlashLog_EraseChip(void) {
	SpiFlash_GlobalUnprotect();
   uint32_t lastSectorAddress = FLASH_SIZE_BYTES & 0xFFFFF000; // sector size = 4096
	for (uint32_t sectorAddress = 0; sectorAddress <= lastSectorAddress; sectorAddress += 4096) {
		SpiFlash_SectorErase(sectorAddress);
		delayMs(50);
		}
	ESP_LOGI(TAG,"Chip erased");
   FlashLogFreeAddress = 0;
   }


void FlashLog_Erase(void) {
	SpiFlash_GlobalUnprotect();
   uint32_t lastSectorAddress = FlashLogFreeAddress & 0xFFFFF000; // sector size = 4096
	for (uint32_t sectorAddress = 0; sectorAddress <= lastSectorAddress; sectorAddress += 4096) {
		SpiFlash_SectorErase(sectorAddress);
		delayMs(50);
		}
	ESP_LOGI(TAG,"Log erased");
   FlashLogFreeAddress = 0;
   }

	
int FlashLog_Init(void ) {
	uint16_t flashID = SpiFlash_ReadID();
	//if (flashID != 0xEF14) {
	//	ESP_LOGE(TAG, "Winbond W25Q16BV ID [expected EF14] = %04X", flashID);
	//	return 0;
	//	}
	if (flashID != 0xEF17) {
		ESP_LOGE(TAG, "Winbond W25Q128FVSG ID [expected EF17] = %04X", flashID);
		return 0;
		}
	IsLogging = 0;
	SpiFlash_GlobalUnprotect();
	FlashLogMutex = xSemaphoreCreateMutex();
	if (FlashLogMutex == NULL) {
		ESP_LOGE(TAG, "Error creating FlashLogMutex");
		return 0;
		}
   int numRecords;
	FlashLog_GetFreeAddress(&numRecords);
	ESP_LOGI(TAG,"Flash Free Address = %08d\r\nNumFlashRecords = %d", FlashLogFreeAddress, numRecords);
	return 1;
	}


// if there is only an IMU record, size of record = HDR + IMU
// if there is a BARO record, size of record = HDR + IMU + BARO
// if there is a GPS record, size of record = HDR + IMU + BARO + GPS even if BARO record is invalid

void FlashLog_GetFreeAddress(int* pNumRecords) {
   uint32_t addr = 0;
   uint32_t maxAddr = FLASHLOG_MAX_ADDR - sizeof(FLASHLOG_RECORD); 
   *pNumRecords = 0;
   while (1) {
      if (addr >= maxAddr) break; // flash is full
      LOG_HDR hdr;
      hdr.magic = 0;
      SpiFlash_ReadBuffer(addr, (uint8_t*)&hdr, sizeof(LOG_HDR));
      if (hdr.magic == 0xFFFF) break; // found free address
      addr += (sizeof(LOG_HDR) + sizeof(IMU_RECORD));
      if (hdr.baroFlags || hdr.gpsFlags) addr += sizeof(BARO_RECORD);
      if (hdr.gpsFlags) addr += sizeof(GPS_RECORD);
      (*pNumRecords)++;
	   if (((*pNumRecords) % 100) == 0) delayMs(10); // yield for task watchdog
      }
   FlashLogFreeAddress = addr;
   }


// ensure this function is called within a mutex take/give as there are multiple tasks
// updating the log record
void FlashLog_WriteRecord(FLASHLOG_RECORD* pRecord) {
	if (FlashLogFreeAddress > (FLASHLOG_MAX_ADDR - sizeof(FLASHLOG_RECORD))) return;
   pRecord->hdr.magic = 0xA55A;
   int numBytes = sizeof(LOG_HDR) + sizeof(IMU_RECORD);
   if (pRecord->hdr.baroFlags || pRecord->hdr.gpsFlags) numBytes += sizeof(BARO_RECORD);
   if (pRecord->hdr.gpsFlags) numBytes += sizeof(GPS_RECORD);

	SpiFlash_WriteBuffer(FlashLogFreeAddress, (uint8_t*)pRecord, numBytes);
	FlashLogFreeAddress += numBytes;
	}



void FlashLog_ReadRecord(uint32_t addr, FLASHLOG_RECORD* pRec) {
   LOG_HDR hdr;
   SpiFlash_ReadBuffer(addr, (uint8_t*)&hdr, sizeof(LOG_HDR));
   if (hdr.magic != 0xA55A) {
      ESP_LOGE(TAG, "error flash log header does not have magic id");
      return;
      }
   int numBytes = sizeof(LOG_HDR) + sizeof(IMU_RECORD);
   if (hdr.baroFlags || hdr.gpsFlags) numBytes += sizeof(BARO_RECORD);
   if (hdr.gpsFlags) numBytes += sizeof(GPS_RECORD);
	SpiFlash_ReadBuffer(addr, (uint8_t*)pRec, numBytes);
	}

