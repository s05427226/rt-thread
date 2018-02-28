/**
  ******************************************************************************
  * @file    rtc.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011 
  * @brief   This file contains all the functions prototypes for the rtc.h
  *          file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

//#define SPI_FLASH_USE_DMA
#define SPI_FLASH_TEST

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "spi.h"
#include <rtthread.h>
	 
#define	SPI_FLASH_MASTER_ID							0							 

//-------------AT45DB161D infomation------------------
#define AT45DB161D_FLASH_ID    					0x1F260000
#define AT45DB_PAGE_SIZE        				512
#define AT45DB_BLOCK_SIZE								4096
#define AT45DB_MAX_PAGE									4096	//it's meainning AT45DB size is 4096*512 bytes
#define Dummy_Byte                			0xA5

//-------------AT45DB161D operation code---------------
#define ReadStatusRegister        			0xD7
#define Read_Data                 			0x52
#define Page_Program              			0x82	
#define BlockErase                			0x50
#define SectorErase               			0x7C
#define PageErase              					0x81
#define ReadManuID_DeviceID       			0x9F

#define RDY_Flag   											0x80  /* Ready/busy(1/0) status flag */


u32 SPI_FLASH_ReadID(void);
void SPI_FLASH_WaitForEnd(void);
void SPI_FLASH_Sector_En_P(void);
void SPI_FLASH_Sector_Dis_P(void);

#ifdef SPI_FLASH_USE_DMA
u8 SPI_DMA_PageWrite(u32 WriteAddr,void *buff,u32 size);
u8 SPI_DMA_PageRead(u32 ReadAddr,void *buff,u32 size);

#else

u8 SPI_FLASH_WriteSector(uint32_t WriteAddr, uint8_t* pBuffer, uint16_t NumByteToWrite);
u8 SPI_FLASH_ReadSector(u32 ReadAddr, u8* pBuffer, u16 NumByteToRead);
#endif

u8 APP_SPI_FLASH_Init(void);

void flash_hw_init(void);

//#undef SPI_FLASH_TEST
#ifdef SPI_FLASH_TEST
void SPI_FLASH_Test(void);
#else
#define SPI_FLASH_Test()	
#endif

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H*/


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

