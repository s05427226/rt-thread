#include <board.h>
#include <rtthread.h>

#include "stm32f4xx.h"

#define ADDR_SRAM_EXT				((rt_uint32_t)0x60000000)
#define SRAM_WRITE(Address, Data)  (*(rt_uint16_t *)(Address) = (Data))

/*******************************************************************************
* Function Name  : FSMC_SRAM_WriteBuffer
* Description    : Writes a Half-word buffer to the FSMC SRAM memory.
* Input          : - pBuffer : pointer to buffer.
*                  - WriteAddr : SRAM memory internal address from which the data
*                    will be written.
*                  - NumHalfwordToWrite : number of half-words to write.
*                   
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_SRAM_WriteBuffer(rt_uint16_t* pBuffer, rt_uint32_t WriteAddr, rt_uint32_t NumHalfwordToWrite)
{
  for(; NumHalfwordToWrite != 0; NumHalfwordToWrite--) /* while there is data to write */
  {
    /* Transfer data to the memory */
    *(rt_uint16_t *) (ADDR_SRAM_EXT + WriteAddr) = *pBuffer++;
   
    /* Increment the address*/ 
    WriteAddr += 2;
  }  
}

/*******************************************************************************
* Function Name  : FSMC_SRAM_ReadBuffer
* Description    : Reads a block of data from the FSMC SRAM memory.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the SRAM memory.
*                  - ReadAddr : SRAM memory internal address to read from.
*                  - NumHalfwordToRead : number of half-words to read.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_SRAM_ReadBuffer(rt_uint16_t* pBuffer, rt_uint32_t ReadAddr, rt_uint32_t NumHalfwordToRead)
{
  for(; NumHalfwordToRead != 0; NumHalfwordToRead--) /* while there is data to read */
  {
    /* Read a half-word from the memory */
    *pBuffer++ = *(rt_uint16_t*) (ADDR_SRAM_EXT + ReadAddr);

    /* Increment the address*/ 
    ReadAddr += 2;
  } 
}

/***************************************************************
* Function Name  : FSMC_SRAM_WriteHalfWord
* Description    : Writes a half-word to the SRAM memory.
* Input          : - WriteAddr : SRAM memory internal address to write to.
*                  - Data : Data to write.
* Output         : None
* Return         :
*******************************************************************************/
void FSMC_SRAM_WriteHalfWord(rt_uint32_t WriteAddr, rt_uint16_t Data)
{
  //NOR_WRITE(ADDR_SHIFT(0x05555), 0x00AA);
 // NOR_WRITE(ADDR_SHIFT(0x02AAA), 0x0055);
 // NOR_WRITE(ADDR_SHIFT(0x05555), 0x00A0);
  SRAM_WRITE((ADDR_SRAM_EXT + WriteAddr), Data);

  //return (FSMC_NOR_GetStatus(Program_Timeout));
}

/******************************************************************************
* Function Name  : FSMC_SRAM_ReadHalfWord
* Description    : Reads a half-word from the SRAM memory.
* Input          : - ReadAddr : NOR memory internal address to read from.
* Output         : None
* Return         : Half-word read from the SRAM memory
*******************************************************************************/
rt_uint16_t FSMC_SRAM_ReadHalfWord(rt_uint32_t ReadAddr)
{
  //NOR_WRITE(ADDR_SHIFT(0x005555), 0x00AA);
  //NOR_WRITE(ADDR_SHIFT(0x002AAA), 0x0055); 
  //NOR_WRITE((Bank1_NOR2_ADDR + ReadAddr), 0x00F0 );

  return (*(rt_uint16_t *)((ADDR_SRAM_EXT + ReadAddr)));
}


static void RCC_Config(void)
{
	//RCC enable gpio
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE 
												| RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG ,ENABLE);
	//FSMC enable gpio
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);
}

static void GPIO_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitS;
	
	//----- DB0 -----  FSMC_D0	---------- PD14
	//----- DB1 -----  FSMC_D1	---------- PD15
	//----- DB2 -----  FSMC_D2	---------- PD0
	//----- DB3 -----  FSMC_D3	---------- PD1
	//----- DB4 -----  FSMC_D4	---------- PE7
	//----- DB5 -----  FSMC_D5	---------- PE8
	//----- DB6 -----  FSMC_D6	---------- PE9
	//----- DB7 -----  FSMC_D7	---------- PE10
	//----- DB8 -----  FSMC_D8	---------- PE11
	//----- DB9 -----  FSMC_D9	---------- PE12
	//----- DB10 ----  FSMC_D10	---------- PE13
	//----- DB11 ----  FSMC_D11	---------- PE14
	//----- DB12 ----  FSMC_D12	---------- PE15
	//----- DB13 ----  FSMC_D13	---------- PD8
	//----- DB14 ----  FSMC_D14	---------- PD9
	//----- DB15 ----  FSMC_D15	---------- PD10
	
	//gpio init
	GPIO_InitS.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 
												| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8
												| GPIO_Pin_9 | GPIO_Pin_10
												| GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitS.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitS.GPIO_OType = GPIO_OType_PP;
	GPIO_InitS.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitS.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD,&GPIO_InitS);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15,GPIO_AF_FSMC);
	
	//gpio init
	GPIO_InitS.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 
												| GPIO_Pin_9 | GPIO_Pin_10 
												| GPIO_Pin_11 | GPIO_Pin_12
												| GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOE,&GPIO_InitS);
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15,GPIO_AF_FSMC);
	
	//----- A0 -----  FSMC_A0	---------- PF0
	//----- A1 -----  FSMC_A1	---------- PF1
	//----- A2 -----  FSMC_A2	---------- PF2
	//----- A3 -----  FSMC_A3	---------- PF3
	//----- A4 -----  FSMC_A4	---------- PF4
	//----- A5 -----  FSMC_A5	---------- PF5
	//----- A6 -----  FSMC_A6	---------- PF12
	//----- A7 -----  FSMC_A7	---------- PF13
	//----- A8 -----  FSMC_A8	---------- PF14
	//----- A9 -----  FSMC_A9	---------- PF15
	//----- A10 ----  FSMC_A10---------- PG0
	//----- A11 ----  FSMC_A11---------- PG1
	//----- A12 ----  FSMC_A12---------- PG2
	//----- A13 ----  FSMC_A13---------- PG3
	//----- A14 ----  FSMC_A14---------- PG4
	//----- A15 ----  FSMC_A15---------- PG5
	//----- A16 ----  FSMC_A16---------- PD11
	//----- A17 ----  FSMC_A17---------- PD12
	//----- A18 ----  FSMC_A18---------- PD13
	
	//gpio init
	GPIO_InitS.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 
												| GPIO_Pin_2 | GPIO_Pin_3 
												| GPIO_Pin_4 | GPIO_Pin_5
												| GPIO_Pin_12 | GPIO_Pin_13 
												| GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOF,&GPIO_InitS);
	
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource2,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource3,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource4,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource5,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource12,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource13,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource14,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource15,GPIO_AF_FSMC);
	
	//gpio init
	GPIO_InitS.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 
												| GPIO_Pin_2 | GPIO_Pin_3 
												| GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOG,&GPIO_InitS);
	
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource0,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource1,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource2,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource3,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource4,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5,GPIO_AF_FSMC);
	
		//gpio init
	GPIO_InitS.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12
												| GPIO_Pin_13 ;
	GPIO_Init(GPIOD,&GPIO_InitS);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13,GPIO_AF_FSMC);
	
	//----- NOE ----  FSMC_NOE ---------- PD4
	//----- NWE ----  FSMC_NWE ---------- PD5
	//----- CE  ----  FSMC_NCS1---------- PD7
		//gpio init
	GPIO_InitS.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5
												| GPIO_Pin_7 ;
	GPIO_Init(GPIOD,&GPIO_InitS);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7,GPIO_AF_FSMC);
	
	//----- UB  ----  FSMC_NBL1---------- PE1
	//----- LB  ----  FSMC_NBL0---------- PE0
	//gpio init
	GPIO_InitS.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOE,&GPIO_InitS);
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1,GPIO_AF_FSMC);
}

static void FSMC_Config(void)
{
	FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitS;
	FSMC_NORSRAMTimingInitTypeDef read_write_timing;
	FSMC_NORSRAMTimingInitTypeDef	write_timing;
	
	read_write_timing.FSMC_AddressSetupTime = 0;
	read_write_timing.FSMC_AddressHoldTime = 0;
	read_write_timing.FSMC_DataSetupTime = 16;
	read_write_timing.FSMC_BusTurnAroundDuration = 0;
	read_write_timing.FSMC_CLKDivision = 0x00;
	read_write_timing.FSMC_DataLatency = 0x00;
	read_write_timing.FSMC_AccessMode = FSMC_AccessMode_A;
	
	write_timing.FSMC_AddressSetupTime = 0;
	write_timing.FSMC_AddressHoldTime = 0;
	write_timing.FSMC_DataSetupTime = 16;
	write_timing.FSMC_BusTurnAroundDuration = 0;
	write_timing.FSMC_CLKDivision = 0;
	write_timing.FSMC_DataLatency = 0;
	write_timing.FSMC_AccessMode = FSMC_AccessMode_A;

	FSMC_NORSRAMInitS.FSMC_Bank = FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitS.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitS.FSMC_MemoryType = FSMC_MemoryType_SRAM;
	FSMC_NORSRAMInitS.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitS.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitS.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitS.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitS.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitS.FSMC_WaitSignalActive =  FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitS.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitS.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitS.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitS.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitS.FSMC_ReadWriteTimingStruct = &read_write_timing;
	FSMC_NORSRAMInitS.FSMC_WriteTimingStruct = &write_timing;
	
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitS);
	
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1,ENABLE);
}

int rt_hw_ex_ram_init(void)
{
	int temp = 0;
	rt_uint16_t write[] = {0x11,0x22,0x33,0x44};
	rt_uint16_t read[4] = {0,};
	
	RCC_Config();
	GPIO_Config();
	FSMC_Config();
	
	FSMC_SRAM_WriteBuffer(write,0,4);
	FSMC_SRAM_ReadBuffer(read,0,4);
	
	temp = FSMC_SRAM_ReadHalfWord(0);
	temp = temp;
	
	return 0;
}
INIT_BOARD_EXPORT(rt_hw_lcd_init);
