#ifndef __SPI_H
#define __SPI_H
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include <rtthread.h>

static rt_sem_t spi_rx_sem = RT_NULL;
static rt_sem_t spi_tx_sem = RT_NULL;

static rt_sem_t spi_sem = RT_NULL;

#define MAX_SPI_NUM									2
#define SPI_BUFF_SIZE 							512

// SPI总线速度设置 
#define SPI_SPEED_2   0
#define SPI_SPEED_4   1
#define SPI_SPEED_8   2
#define SPI_SPEED_16  3
#define SPI_SPEED_256 4


//====================spi 1 configs begin============================
/**************************************************
	 
#define SPI1_CLK_PORT_PIN					 	GPIOB3
#define SPI1_MISO_PORT_PIN					GPIOB4
#define SPI1_MOSI_PORT_PIN					GPIOB5
#define SPI1_FALSH_CS_PORT_PIN			GPIOG15
#define SPI1_SD_CS_PORT_PIN					GPIOD12//GPIOB6
**************************************************/

#define SPI1_GPIO_CLK							 	(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOG)
#define SPI1_DMA_CLK                RCC_AHB1Periph_DMA2
#define SPI1_SPI_CLK                RCC_APB2Periph_SPI1
	 
#define SPI1_GPIO_PORT							GPIOB
#define SPI1_CLK_PORT							 	GPIOB

#define SPI1_MISO_PIN								GPIO_Pin_4
#define SPI1_MOSI_PIN								GPIO_Pin_5
#define SPI1_CLK_PIN								GPIO_Pin_3

#define SPI1_MISO_PIN_SOURCE				GPIO_PinSource4
#define SPI1_MOSI_PIN_SOURCE				GPIO_PinSource5
#define SPI1_CLK_PIN_SOURCE					GPIO_PinSource3

#define SPI_FLASH_GPIO_AF									GPIO_AF_SPI1

#define SPI1_DMA                    	DMA2
#define SPI1_DMA_CHANNEL            	DMA_Channel_3
#define SPI1_DMA_STREAM_RX          	DMA2_Stream2 
#define SPI1_DMA_STREAM_TX          	DMA2_Stream3

// sd card cs config
#define SD_CS_GPIO_PORT								GPIOD//GPIOB
#define SD_CS_GPIO_PIN								GPIO_Pin_12//GPIO_Pin_6
#define SD_CS_GPIO_PIN_SOURCE					GPIO_PinSource12//GPIO_PinSource6

//at45db cs	config
#define SPI1_FLASH_CS_PORT						GPIOG
#define SPI1_FLASH_CS_PIN							GPIO_Pin_15
#define SPI1_FLASH_CS_PIN_SOURCE			GPIO_PinSource15

#define SD_CS_HIGH()									do {\
																				GPIO_SetBits(SD_CS_GPIO_PORT, SD_CS_GPIO_PIN);\
																			}while(0)	
#define SD_CS_LOW()										do{\
																				SPI_I2S_ReceiveData(SPI1);\
																				GPIO_SetBits(SPI1_FLASH_CS_PORT, SPI1_FLASH_CS_PIN);\
																				GPIO_ResetBits(SD_CS_GPIO_PORT, SD_CS_GPIO_PIN);\
																			}while(0)

#define SPI1_FLASH_CS_HIGH()					do {\
																				GPIO_SetBits(SPI1_FLASH_CS_PORT, SPI1_FLASH_CS_PIN);\
																			}while(0)
#define SPI1_FLASH_CS_LOW()						do{\
																				SPI_I2S_ReceiveData(SPI1);\
																				GPIO_SetBits(SD_CS_GPIO_PORT, SD_CS_GPIO_PIN);\
																				GPIO_ResetBits(SPI1_FLASH_CS_PORT, SPI1_FLASH_CS_PIN);\
																			}while(0)
//====================spi 1 configs end============================

extern u8 SPI1_SendByte(int spix,u8 byte);
extern void My_SPI_Init(int spix);
extern void SPIx_SetSpeed(u8 SpeedSet);
																			
void SPI1_DMAConfig(int spix,void *tx,u32 tx_size,void *rx,u32 rx_size);
																			
extern rt_err_t wait_for_spi_tx_comp(void);
extern rt_err_t wait_for_spi_rx_comp(void);
extern rt_err_t wait_for_spi_idle(void);
extern rt_err_t set_spi_idle(void);
#endif
