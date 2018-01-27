#ifndef SPIFLASH_H_
#define SPIFLASH_H_

#include "config.h"

#if defined(FLASH_W25Q128)
   #define FLASH_SIZE_BYTES	16777216
#elif defined(FLASH_W25Q16)
   #define FLASH_SIZE_BYTES	2097152
#endif

#define FLASH_SECTOR_SIZE	4096


void 	SpiFlash_Send(uint8_t out);
uint8_t SpiFlash_Receive(void);
uint8_t SpiFlash_ReadByte(uint32_t address);
void 	SpiFlash_WriteByte(uint32_t address, uint8_t wrByte);
uint8_t SpiFlash_GetStatus(void);
void 	SpiFlash_ChipErase(void);
void 	SpiFlash_SectorErase(uint32_t address);
void 	SpiFlash_GlobalUnprotect(void);
void 	SpiFlash_WriteEnable(void);
void 	SpiFlash_WriteDisable(void);
uint16_t  SpiFlash_ReadID(void);
void 	SpiFlash_WriteBuffer(uint32_t address, uint8_t* pBuffer, int nBytes);
void 	SpiFlash_ReadBuffer(uint32_t addr, uint8_t* pBuf, int nBytes);

#endif
