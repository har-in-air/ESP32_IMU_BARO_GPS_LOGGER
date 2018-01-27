#include "common.h"
#include "config.h"
#include "spihw.h"
#include "spiflash.h"


#define SPIFLASH_BUSY() 	(SpiFlash_GetStatus() & 0x01)

inline void SpiFlash_WriteEnable(void) {
   spiSimpleTransaction(_spi);
   FLASHCS_0();
   spiTransferByteNL(_spi, 0x06);
   FLASHCS_1();
   spiEndTransaction(_spi);
   }


/// Send write-enable command prior to writing data
inline void SpiFlash_WriteDisable(void) {
   spiSimpleTransaction(_spi);
   FLASHCS_0();
   spiTransferByteNL(_spi, 0x04);
   FLASHCS_1();
   spiEndTransaction(_spi);
   }	

/// For allowing writes to the flash, the chip needs to be
/// globally unprotected (done during initialization on system power up)
void SpiFlash_GlobalUnprotect(void) {
   while (SPIFLASH_BUSY());
   SpiFlash_WriteEnable();
   spiSimpleTransaction(_spi);
   FLASHCS_0();
   spiTransferByteNL(_spi, 0x01); // Write Status register
   spiTransferByteNL(_spi, 0x00); // enable all blocks for write
   FLASHCS_1();
   spiEndTransaction(_spi);
   }


/// Erase a 4kbyte sector, 18mS
/// @param address within the block
void SpiFlash_SectorErase(uint32_t address){
   while (SPIFLASH_BUSY());
   SpiFlash_WriteEnable();
   spiSimpleTransaction(_spi);
   FLASHCS_0();
   spiTransferByteNL(_spi, 0x20); // 4Kbyte sector
   spiTransferByteNL(_spi, (uint8_t)((address>>16)&0xff));
   spiTransferByteNL(_spi, (uint8_t)((address>>8)&0xff));
   spiTransferByteNL(_spi, (uint8_t)(address&0xff));
   FLASHCS_1();
   spiEndTransaction(_spi);
   while (SPIFLASH_BUSY());
   }


/// Erase the entire chip, 35mS
void SpiFlash_ChipErase(void) {
   while (SPIFLASH_BUSY());
   SpiFlash_WriteEnable();
   spiSimpleTransaction(_spi);
   FLASHCS_0();
   spiTransferByteNL(_spi, 0xC7);
   FLASHCS_1();
   spiEndTransaction(_spi);
   while (SPIFLASH_BUSY());
   }


/// Check status of last command
uint8_t  SpiFlash_GetStatus(void) {
   uint8_t rcvByte;
   spiSimpleTransaction(_spi);
   FLASHCS_0();
   spiTransferByteNL(_spi, 0x05);
   rcvByte = spiTransferByteNL(_spi, 0);
   FLASHCS_1();
   spiEndTransaction(_spi);
   return rcvByte;
   }



/// Write a single byte to a 24bit address 7uS
/// Block must be already erased
void SpiFlash_WriteByte(uint32_t address, uint8_t wrByte) {
   while (SPIFLASH_BUSY());
   SpiFlash_WriteEnable();
   spiSimpleTransaction(_spi);
   FLASHCS_0();
   spiTransferByteNL(_spi, 0x02);
   spiTransferByteNL(_spi, (uint8_t)((address>>16)&0xff));
   spiTransferByteNL(_spi, (uint8_t)((address>>8)&0xff));
   spiTransferByteNL(_spi, (uint8_t)(address&0xff));
   spiTransferByteNL(_spi, wrByte);
   FLASHCS_1();
   spiEndTransaction(_spi);
   }

/// Read a single byte from 24-bit address, high-speed mode upto 104MHz
uint8_t SpiFlash_ReadByte(uint32_t address) {
   uint8_t rdByte;
   while (SPIFLASH_BUSY());
   spiSimpleTransaction(_spi);
   FLASHCS_0();
   spiTransferByteNL(_spi, 0x03);
   spiTransferByteNL(_spi, (uint8_t)((address>>16)&0xff));
   spiTransferByteNL(_spi, (uint8_t)((address>>8)&0xff));
   spiTransferByteNL(_spi, (uint8_t)(address&0xff));
   rdByte =  spiTransferByteNL(_spi, 0);
   FLASHCS_1();
   spiEndTransaction(_spi);
   return rdByte;
   }

	
void SpiFlash_WriteBuffer(uint32_t addr, uint8_t* pBuf, int nBytes) {
   int nPageFreeBytes,nP0Bytes, nP1Bytes;
   nPageFreeBytes = (int)(256 - (addr & 0x000000ff));
   if (nBytes <= nPageFreeBytes) {
		nP0Bytes = nBytes;
		nP1Bytes = 0;
		}
	else {
		nP0Bytes = nPageFreeBytes;
		nP1Bytes = nBytes - nP0Bytes;
		}

   while (SPIFLASH_BUSY());
   SpiFlash_WriteEnable();
   spiSimpleTransaction(_spi);
   FLASHCS_0();
   uint8_t cmd[] = {0x02, (uint8_t)((addr>>16)&0xff), (uint8_t)((addr>>8)&0xff),(uint8_t)(addr&0xff)};
   spiTransferBytesNL(_spi, cmd, NULL, 4);
   spiTransferBytesNL(_spi, pBuf, NULL, nP0Bytes);
   FLASHCS_1();
   spiEndTransaction(_spi);

	if (nP1Bytes) {
	   addr >>= 8;
		addr++;
		addr <<= 8;
	   while (SPIFLASH_BUSY());
		SpiFlash_WriteEnable();
		spiSimpleTransaction(_spi);
      FLASHCS_0();
      uint8_t cmd[] = {0x02, (uint8_t)((addr>>16)&0xff), (uint8_t)((addr>>8)&0xff),(uint8_t)(addr&0xff)};
      spiTransferBytesNL(_spi, cmd, NULL, 4);
      pBuf += nP0Bytes;
      spiTransferBytesNL(_spi, pBuf, NULL, nP1Bytes);
		FLASHCS_1();
		spiEndTransaction(_spi);
		}
	}


void SpiFlash_ReadBuffer(uint32_t addr, uint8_t* pBuf, int nBytes) {
   while (SPIFLASH_BUSY());
   spiSimpleTransaction(_spi);
	FLASHCS_0();
	spiTransferByteNL(_spi, 0x0B);
	spiTransferByteNL(_spi, (uint8_t)((addr>>16)&0xff));
	spiTransferByteNL(_spi, (uint8_t)((addr>>8)&0xff));
	spiTransferByteNL(_spi, (uint8_t)(addr&0xff));
	spiTransferByteNL(_spi, 0x00);

   spiTransferBytesNL(_spi, NULL, pBuf, nBytes);

	FLASHCS_1();
	spiEndTransaction(_spi);
	return;
	}	


// Winbond W25Q16BV ID = 0xEF14
// Winbond W25Q128FVSG ID = 0xEF17

uint16_t SpiFlash_ReadID(void) {
   uint8_t manufID, devID;
   while (SPIFLASH_BUSY());
   spiSimpleTransaction(_spi);
	FLASHCS_0();
	spiTransferByteNL(_spi, 0x90);
	spiTransferByteNL(_spi, 0);
	spiTransferByteNL(_spi, 0);
	spiTransferByteNL(_spi, 0);
   manufID = spiTransferByteNL(_spi, 0);
   devID = spiTransferByteNL(_spi, 0);
   FLASHCS_1();
	spiEndTransaction(_spi);
   return  (((uint16_t)manufID)<<8 | (uint16_t)devID);
   }

	
