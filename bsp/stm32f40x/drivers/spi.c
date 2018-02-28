#include "spi.h"

#define SPI_RW_TIMEOUT					2000
#define SPI_TIMEOUT_RETRY_CNT		1000

static u8 Tx_Buff[SPI_BUFF_SIZE] = {0};
static u8 Rx_Buff[SPI_BUFF_SIZE] = {0};
static u8 spix_init_flag[MAX_SPI_NUM] = {0,0};

void SPIx_SetSpeed(u8 SpeedSet)
{
	SPI_InitTypeDef   SPIx_InitStructure;
	SPI_I2S_DeInit(SPI1);
		
	RCC_APB2PeriphClockCmd(SPI1_SPI_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(SPI1_DMA_CLK, ENABLE);
	
	SPIx_InitStructure.SPI_Direction =  SPI_Direction_2Lines_FullDuplex;
	SPIx_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPIx_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPIx_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPIx_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPIx_InitStructure.SPI_NSS = SPI_NSS_Soft;
	
	switch(SpeedSet)
	{
		case SPI_SPEED_2://二分频
			SPIx_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
			break;
		case SPI_SPEED_4://四分频
			SPIx_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
			break;
		case SPI_SPEED_8://八分频
			SPIx_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
			break;
		case SPI_SPEED_16://十六分频
			SPIx_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
			break;
		case SPI_SPEED_256://256分频
			SPIx_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
			break;
	}
	
	SPIx_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPIx_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPIx_InitStructure);
		
	SPI_Cmd(SPI1, ENABLE);	  
} 

void SPI1_NVIC_Config(int spix)
{
	if(spix == 0) {
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

		NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		DMA_Cmd(SPI1_DMA_STREAM_RX, ENABLE);
		DMA_Cmd(SPI1_DMA_STREAM_TX, ENABLE);

		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
	}
}
void SPI1_DMAConfig(int spix,void *tx,u32 tx_size,void *rx,u32 rx_size)
{
	DMA_InitTypeDef DMA_InitStructure;
	if(spix == 0) {
		DMA_DeInit(SPI1_DMA_STREAM_RX);
		DMA_InitStructure.DMA_Channel  = SPI1_DMA_CHANNEL;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(rx==0?Rx_Buff:rx);
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = rx_size > SPI_BUFF_SIZE?SPI_BUFF_SIZE:rx_size;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(SPI1_DMA_STREAM_RX, &DMA_InitStructure);

		DMA_DeInit(SPI1_DMA_STREAM_TX);
		DMA_InitStructure.DMA_Channel  = SPI1_DMA_CHANNEL;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(tx==0?Tx_Buff:tx);
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = tx_size > SPI_BUFF_SIZE?SPI_BUFF_SIZE:tx_size;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
		DMA_Init(SPI1_DMA_STREAM_TX, &DMA_InitStructure);

		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
	}
}

void SPI1_GPIOConfig(int spix)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	if(spix == 0) {
		RCC_AHB1PeriphClockCmd(SPI1_GPIO_CLK,ENABLE);
		
		/* Deselect the sd card: Chip Select high */
		GPIO_InitStructure.GPIO_Pin = SD_CS_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStructure);
		GPIO_SetBits(SD_CS_GPIO_PORT, SD_CS_GPIO_PIN);
		
		/* Deselect the FLASH: Chip Select high */
		GPIO_InitStructure.GPIO_Pin = SPI1_FLASH_CS_PIN;
		GPIO_Init(SPI1_FLASH_CS_PORT, &GPIO_InitStructure);
		GPIO_SetBits(SPI1_FLASH_CS_PORT, SPI1_FLASH_CS_PIN);
		
		//CLK
		GPIO_InitStructure.GPIO_Pin = SPI1_CLK_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(SPI1_CLK_PORT, &GPIO_InitStructure);
		
		//MOSI
		GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN;
		GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStructure);
		
		//MISO
		GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStructure);

		GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_MOSI_PIN_SOURCE, SPI_FLASH_GPIO_AF);
		GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_MISO_PIN_SOURCE, SPI_FLASH_GPIO_AF);
		GPIO_PinAFConfig(SPI1_CLK_PORT, SPI1_CLK_PIN_SOURCE, SPI_FLASH_GPIO_AF);
	}
}

void SPI1_SPIConfig(int spix)
{
	SPI_InitTypeDef   SPIx_InitStructure;
	if(spix == 0) {
		SPI_I2S_DeInit(SPI1);
		
		RCC_APB2PeriphClockCmd(SPI1_SPI_CLK, ENABLE);
		RCC_AHB1PeriphClockCmd(SPI1_DMA_CLK, ENABLE);
		
		SPIx_InitStructure.SPI_Direction =  SPI_Direction_2Lines_FullDuplex;
		SPIx_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPIx_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPIx_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPIx_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
		SPIx_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPIx_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; /* 72MHz / 8 = 14MHz */
		SPIx_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPIx_InitStructure.SPI_CRCPolynomial = 7;
		SPI_Init(SPI1, &SPIx_InitStructure);
		
		SPI_Cmd(SPI1, ENABLE);
	}
}

u8 SPI1_SendByte(int spix,u8 byte)
{
	int32_t timeout = SPI_TIMEOUT_RETRY_CNT;
  /* Loop while DR register in not emplty */
	if(spix == 0) {
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET && --timeout >0);
		if( timeout <= 0 ){ return 0;}
		
		/* Send byte through the SPI3 peripheral */
		SPI_I2S_SendData(SPI1, byte);
		timeout = SPI_TIMEOUT_RETRY_CNT;
		/* Wait to receive a byte */
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET && --timeout >0);
		if( timeout <= 0 ){ return 0;}
		
		/* Return the byte read from the SPI bus */
		return SPI_I2S_ReceiveData(SPI1);
	}
	return 0;
}
	
void DMA2_Stream2_IRQHandler(void)
{ 
	if(DMA_GetITStatus(SPI1_DMA_STREAM_RX,DMA_IT_TCIF2)==SET)
	{
		DMA_ClearITPendingBit(SPI1_DMA_STREAM_RX,DMA_IT_TCIF2 | DMA_IT_HTIF2 | DMA_IT_TEIF2);
		rt_sem_release(spi_rx_sem);
	}
}

void DMA2_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(SPI1_DMA_STREAM_TX,DMA_IT_TCIF3)==SET)
	{
		DMA_ClearITPendingBit(SPI1_DMA_STREAM_TX,DMA_IT_TCIF3 | DMA_IT_HTIF3 | DMA_IT_TEIF3);
		rt_sem_release(spi_tx_sem); 
	}
}

rt_err_t wait_for_spi_tx_comp()
{
	return rt_sem_take( spi_tx_sem, SPI_RW_TIMEOUT );
}


rt_err_t wait_for_spi_rx_comp()
{
	return rt_sem_take( spi_rx_sem, SPI_RW_TIMEOUT );
}

rt_err_t wait_for_spi_idle()
{
	return rt_sem_take( spi_sem, RT_WAITING_FOREVER );
}

rt_err_t set_spi_idle()
{
	return rt_sem_release( spi_sem);
}

void My_SPI_Init(int spix)
{
	if(spix_init_flag[spix] == 1)
		return;
	spix_init_flag[spix] = 1;
	
	spi_rx_sem = rt_sem_create("spirx", 0, RT_IPC_FLAG_FIFO);
	spi_tx_sem = rt_sem_create("spitx", 0, RT_IPC_FLAG_FIFO);
	spi_sem = rt_sem_create("spisem", 1, RT_IPC_FLAG_FIFO);
	
	SPI1_GPIOConfig(spix);
	SPI1_NVIC_Config(spix);
	SPI1_SPIConfig(spix);
}
