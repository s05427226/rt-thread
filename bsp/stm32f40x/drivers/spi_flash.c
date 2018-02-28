#include "spi_flash.h"

static u32 g_flash_id = 0;
/**
  * @brief  Polls the status of the RDY/busy flag in the
  *   FLASH's status  register  and  loop  until opertaion
  *   has completed.
  * @param  None
  * @retval : None
  */
void SPI_FLASH_WaitForEnd()
{
  uint8_t FLASH_Status = 0;

  /* Select the FLASH: Chip Select low */
  SPI1_FLASH_CS_LOW();

  /* Send "Read Status Register" instruction */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,ReadStatusRegister);

  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);

  }
  while ((FLASH_Status & RDY_Flag) == RESET); /* Busy in progress */

  /* Deselect the FLASH: Chip Select high */
  SPI1_FLASH_CS_HIGH();
}


u32 SPI_FLASH_ReadID(void)
{
	u32 Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
  SPI1_FLASH_CS_LOW();
  SPI1_SendByte(SPI_FLASH_MASTER_ID,ReadManuID_DeviceID);
  Temp0 = SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
  Temp1 = SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
  Temp2 = SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
  SPI1_FLASH_CS_HIGH();      
  Temp = (Temp0 << 24) | (Temp1 << 16) | (Temp2 << 8);
  return Temp;
}

void SPI_FLASH_Sector_En_P()
{
	SPI1_FLASH_CS_LOW();
  SPI1_SendByte(SPI_FLASH_MASTER_ID,0x3D);
	SPI1_SendByte(SPI_FLASH_MASTER_ID,0x2A);
	SPI1_SendByte(SPI_FLASH_MASTER_ID,0x7F);
	SPI1_SendByte(SPI_FLASH_MASTER_ID,0xA9);
  SPI1_FLASH_CS_HIGH();
}

void SPI_FLASH_Sector_Dis_P()
{
	SPI1_FLASH_CS_LOW();
  SPI1_SendByte(SPI_FLASH_MASTER_ID,0x3D);
	SPI1_SendByte(SPI_FLASH_MASTER_ID,0x2A);
	SPI1_SendByte(SPI_FLASH_MASTER_ID,0x7F);
	SPI1_SendByte(SPI_FLASH_MASTER_ID,0x9A);
  SPI1_FLASH_CS_HIGH();
}

/**
  * @brief  Erases the specified FLASH sector.
  * @param SectorAddr: address of the sector to erase.
  * @retval : None
  */
void SPI_FLASH_SectorErase(uint32_t SectorAddr)
{
  /* Sector Erase */
  /* Select the FLASH: Chip Select low */
	SPI_FLASH_Sector_En_P();
	
  SPI1_FLASH_CS_LOW();
  /* Send Sector Erase instruction */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,SectorErase);
  /* Send SectorAddr high nibble address byte */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,(SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,(SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,SectorAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  SPI1_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForEnd();
	
	SPI_FLASH_Sector_Dis_P();
}

/**
  * @brief  Erases the specified FLASH page.
  * @param SectorAddr: address of the sector to erase.
  * @retval : None
  */
void SPI_FLASH_PageErase(uint32_t PageAddr)
{
  /* Sector Erase */
	SPI_FLASH_Sector_En_P();
	
  /* Select the FLASH: Chip Select low */
  SPI1_FLASH_CS_LOW();
  /* Send Sector Erase instruction */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,PageErase);
  /* Send SectorAddr high nibble address byte */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,(PageAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,(PageAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,PageAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  SPI1_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForEnd();
	
	SPI_FLASH_Sector_Dis_P();
}

#ifndef SPI_FLASH_USE_DMA
/*******************************************************************************
* Function Name  : SPI_FLASH_ReadSector
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : 0-success; 1-falied
*******************************************************************************/
u8 SPI_FLASH_ReadSector(u32 ReadAddr, u8* pBuffer, u16 NumByteToRead)
{
	u16  uiPageAddress;
	u8 bytes[2];

  uiPageAddress  = (u16)ReadAddr & 0x0FFF;
	uiPageAddress *= 4;

	bytes[0] = (u8)(uiPageAddress>>8);					// msb
	bytes[1] = (u8)(uiPageAddress & 0xFF);          	// lsb
				
  /* Select the FLASH: Chip Select low */
  SPI1_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,Read_Data);

  /* Send ReadAddr high nibble address byte to read from */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,bytes[0]);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,bytes[1]);
  /* Send ReadAddr low nibble address byte to read from */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,0x00);

	/* Send a dummy byte to generate the clock needed by the FLASH */
	SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
	/* Send a dummy byte to generate the clock needed by the FLASH */
	SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
	/* Send a dummy byte to generate the clock needed by the FLASH */
	SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
	/* Send a dummy byte to generate the clock needed by the FLASH */
	SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
	
  while (NumByteToRead--) /* while there is data to be read */
  {
    /* Read a byte from the FLASH */
    *pBuffer = SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI1_FLASH_CS_HIGH();
	
	return 0;
}

/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE
  *         cycle(Page WRITE sequence). The number of byte can't exceed
  *         the FLASH page size.
  * @param pBuffer : pointer to the buffer  containing the data to be
  *                  written to the FLASH.
  * @param WriteAddr : FLASH's internal address to write to.
  * @param NumByteToWrite : number of bytes to write to the FLASH,
  *                       must be equal or less than "AT45DB_PAGE_SIZE" value.
  * @retval : 0-success; 1-falied
  */
u8 SPI_FLASH_WriteSector(uint32_t WriteAddr, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
	u16  uiPageAddress;
	u8 bytes[2];
	uiPageAddress  = (u16)WriteAddr & 0x0FFF;
	uiPageAddress *= 4;
	
	bytes[0] = (u8)(uiPageAddress>>8);					// msb
	bytes[1] = (u8)(uiPageAddress & 0xFF);				// lsb
	
	SPI_FLASH_Sector_En_P();
	
  /* Select the FLASH: Chip Select low */
  SPI1_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,Page_Program);
  /* Send WriteAddr high nibble address byte to write to */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,bytes[0]);
  /* Send WriteAddr medium nibble address byte to write to */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,bytes[1]);
  /* Send WriteAddr low nibble address byte to write to */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,0x00);

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /* Send the current byte */
    SPI1_SendByte(SPI_FLASH_MASTER_ID,*pBuffer);
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI1_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForEnd();
	
	SPI_FLASH_Sector_Dis_P();
	
	return 0;
}


#else
//ret value:1 failed,0 success

u8 SPI_DMA_PageWrite(u32 WriteAddr,void *buff,u32 size)
{
	u16  uiPageAddress;
	u8 bytes[2];
	u8 retry = 0;
	u16 retval = 0;
	
	uiPageAddress  = (u16)WriteAddr & 0x0FFF;
	uiPageAddress <<= 2;
	
	bytes[0] = (u8)(uiPageAddress>>8);					// msb
	bytes[1] = (u8)(uiPageAddress & 0xFF);				// lsb
  /* Enable the write access to the FLASH */

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		if(retry++<3)
			rt_thread_delay(1);
		else{
			retval = 1;
			rt_kprintf("SPI_DMA_PageWrite 1\n");
			goto out;
		}	
	}
	
	SPI_FLASH_Sector_En_P();
	
	SPI1_DMAConfig(SPI_FLASH_MASTER_ID,buff,size,(void *)0,0);
	DMA_ITConfig(SPI1_DMA_STREAM_RX, DMA_IT_TC, ENABLE);
	DMA_ITConfig(SPI1_DMA_STREAM_TX, DMA_IT_TC, ENABLE);
	DMA_SetCurrDataCounter(SPI1_DMA_STREAM_RX,size > SPI_BUFF_SIZE?SPI_BUFF_SIZE:size);
	DMA_SetCurrDataCounter(SPI1_DMA_STREAM_TX,size > SPI_BUFF_SIZE?SPI_BUFF_SIZE:size);
	
  SPI1_FLASH_CS_LOW();
	
  /* Send "Write to Memory " instruction */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,Page_Program);
  /* Send WriteAddr high nibble address byte to write to */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,bytes[0]);
  /* Send WriteAddr medium nibble address byte to write to */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,bytes[1]);
  /* Send WriteAddr low nibble address byte to write to */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,0x00);

	DMA_Cmd(SPI1_DMA_STREAM_TX, ENABLE);
	DMA_Cmd(SPI1_DMA_STREAM_RX, ENABLE);

	if( wait_for_spi_rx_comp() != RT_EOK  ) {
		retval = 1;
		rt_kprintf("SPI_DMA_PageWrite 2\n");
	}
	
	DMA_Cmd(SPI1_DMA_STREAM_TX, DISABLE);
	DMA_Cmd(SPI1_DMA_STREAM_TX, DISABLE);
	SPI1_FLASH_CS_HIGH();
	SPI_FLASH_WaitForEnd();
	SPI_FLASH_Sector_Dis_P();
	
	if(retval == 1)
		goto out;
	
	retval = DMA_GetCurrDataCounter(SPI1_DMA_STREAM_TX);
	
out:
	return !!retval;
}
//ret value:1 failed,0 success
u8 SPI_DMA_PageRead(u32 ReadAddr,void *buff,u32 size)
{
	u16  uiPageAddress;
	u8 bytes[2];
	u8 retry = 0;
	u16 retval = 0;

  uiPageAddress  = (u16)ReadAddr & 0x0FFF;
	uiPageAddress <<= 2;

	bytes[0] = (u8)(uiPageAddress>>8);					// msb
	bytes[1] = (u8)(uiPageAddress & 0xFF);          	// lsb
	
	while (DMA_GetCmdStatus(SPI1_DMA_STREAM_RX) != DISABLE)
	{
		if(retry++<3)
			rt_thread_delay(1);
		else{
			retval = 1;
			rt_kprintf("SPI_DMA_PageRead 1\n");
			goto out;
		}
	}
	
	retry = 0;
  while (DMA_GetCmdStatus(SPI1_DMA_STREAM_TX) != DISABLE)
	{
		if(retry++<3)
			rt_thread_delay(1);
		else{
			retval = 1;
			rt_kprintf("SPI_DMA_PageRead 2\n");
			goto out;
		}
	}		
	
  /* Select the FLASH: Chip Select low */
	SPI1_DMAConfig(SPI_FLASH_MASTER_ID,(void *)0,0,buff,size);
	DMA_ITConfig(SPI1_DMA_STREAM_RX, DMA_IT_TC, ENABLE);
	DMA_ITConfig(SPI1_DMA_STREAM_TX, DMA_IT_TC, ENABLE);
	DMA_SetCurrDataCounter(SPI1_DMA_STREAM_RX,size > SPI_BUFF_SIZE?SPI_BUFF_SIZE:size);
	DMA_SetCurrDataCounter(SPI1_DMA_STREAM_TX,size > SPI_BUFF_SIZE?SPI_BUFF_SIZE:size);
	
  SPI1_FLASH_CS_LOW();
	
  SPI1_SendByte(SPI_FLASH_MASTER_ID,Read_Data);

	/* Send ReadAddr high nibble address byte to read from */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,bytes[0]);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,bytes[1]);
  /* Send ReadAddr low nibble address byte to read from */
  SPI1_SendByte(SPI_FLASH_MASTER_ID,0x00);

	/* Send a dummy byte to generate the clock needed by the FLASH */
	SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
	/* Send a dummy byte to generate the clock needed by the FLASH */
	SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
	/* Send a dummy byte to generate the clock needed by the FLASH */
	SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);
	/* Send a dummy byte to generate the clock needed by the FLASH */
	SPI1_SendByte(SPI_FLASH_MASTER_ID,Dummy_Byte);

	DMA_Cmd(SPI1_DMA_STREAM_TX, ENABLE);
	DMA_Cmd(SPI1_DMA_STREAM_RX, ENABLE);

	if( wait_for_spi_rx_comp() != RT_EOK ) {
		retval = 1;
		rt_kprintf("SPI_DMA_PageRead 3\n");
	}
	DMA_Cmd(SPI1_DMA_STREAM_TX, DISABLE);
	DMA_Cmd(SPI1_DMA_STREAM_RX, DISABLE);
	SPI1_FLASH_CS_HIGH();

	if(retval == 1)
		goto out;
	
	retval = DMA_GetCurrDataCounter(SPI1_DMA_STREAM_RX);
	
out:
	return !!retval;
}

#endif

rt_size_t flash_read(rt_device_t dev, rt_off_t offset, void * buf, rt_size_t size)
{
	u8 ret = 0;
	rt_size_t res = size;
	rt_off_t off = 0;
	char *buff = (char *)buf;
	
	wait_for_spi_idle();

#ifdef SPI_FLASH_USE_DMA
	
	while(res>0) {
		ret += (rt_size_t)SPI_DMA_PageRead((u32)(offset+off), (u8 *)buff,AT45DB_PAGE_SIZE);
		off++;
		res -= 1;
		buff += 512;
	}
	
#else
	
	while(res>0) {
		ret += (rt_size_t)SPI_FLASH_ReadSector((u32)(offset+off), (u8 *)buff,AT45DB_PAGE_SIZE);
		off++;
		res -= 1;
		buff += 512;
	}
	
#endif
	set_spi_idle();
	
	if(!ret)
		return size;
	else {
		rt_kprintf("flash_read read return 0\n");
		return 0;
	}
}

rt_size_t flash_write(rt_device_t dev, rt_off_t offset,const void * buf, rt_size_t size)
{
		u8 ret = 0;
	rt_size_t res = size;
	rt_off_t off = 0;
	char *buff = (char *)buf;

	wait_for_spi_idle();


#ifdef SPI_FLASH_USE_DMA

	while(res>0) {
		ret += (rt_size_t)SPI_DMA_PageWrite((u32)(offset+off), (u8 *)buff,AT45DB_PAGE_SIZE);
		off++;
		res -= 1;
		buff += 512;
	}

#else
	
	while(res>0) {
		ret += (rt_size_t)SPI_FLASH_WriteSector((u32)(offset+off), (u8 *)buff,AT45DB_PAGE_SIZE);
		off++;
		res -= 1;
		buff += 512;
	}

#endif
	set_spi_idle();
	
	if(!ret)
		return size;
	else {
		rt_kprintf("flash_read write return 0\n");
		return 0;
	}
}
static rt_err_t flash_init(rt_device_t dev)
{
	My_SPI_Init(0);
	
	g_flash_id = SPI_FLASH_ReadID();
	if(g_flash_id != AT45DB161D_FLASH_ID)
		rt_kprintf("FLASH_ReadID error  flash_id:%d!\n",g_flash_id);

	return RT_EOK;
}
static rt_err_t flash_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}
static rt_err_t flash_close(rt_device_t dev)
{

	return RT_EOK;
}

static rt_err_t flash_control(rt_device_t dev, int cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;
				
				if(g_flash_id == 0x1F260000)
				{
					geometry->bytes_per_sector = AT45DB_PAGE_SIZE;
					geometry->block_size = AT45DB_BLOCK_SIZE;
					geometry->sector_count = AT45DB_MAX_PAGE;
				}
    }

	return RT_EOK;
}

static struct rt_device	spi_flash_device;

void flash_hw_init()
{
	spi_flash_device.type    = RT_Device_Class_Block;
	spi_flash_device.init    = flash_init;
	spi_flash_device.open    = flash_open;
	spi_flash_device.close   = flash_close;
	spi_flash_device.read 	 = flash_read;
	spi_flash_device.write   = flash_write;
	spi_flash_device.control = flash_control;
	/* no private */
	spi_flash_device.user_data = RT_NULL;

	rt_device_register(&spi_flash_device, "flash0",
										 RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

}

#ifdef SPI_FLASH_TEST

u32 g_spi_flash_id=0;
int flash_w(int x)
{
	rt_device_t device = RT_NULL;
	char buf[AT45DB_PAGE_SIZE];
	int i;

	flash_hw_init();
	
	for(i =0; i< AT45DB_PAGE_SIZE; i++ )
	{
		buf[i] = i;
	}

	// step 1:find device
	device = rt_device_find("flash0");
	if( device == RT_NULL)
	{
			rt_kprintf("device %s: not found!\r\n");
			return RT_ERROR;
	}
	device->open(device,RT_DEVICE_FLAG_RDWR);
	device->write(device,x, buf,1);
	device->close(device);
	rt_kprintf("Finsh test\n");
	return 0;
}

int flash_r(int x)
{
	rt_device_t device = RT_NULL;
	char read[AT45DB_PAGE_SIZE] = {0,};
	int i;

	// step 1:find device
	device = rt_device_find("flash0");
	if( device == RT_NULL)
	{
			rt_kprintf("device %s: not found!\r\n");
			return RT_ERROR;
	}
	device->open(device,RT_DEVICE_FLAG_RDWR);
	device->read(device,x, read,1);
	
	for(i=0;i<AT45DB_PAGE_SIZE;i++) {
		rt_kprintf("%02x ",read[i]);
	}
	rt_kprintf("\n");
	
	device->close(device);
	rt_kprintf("Finsh test\n");
	return 0;
}

int flash_a()
{
	rt_device_t device = RT_NULL;
	char buf[AT45DB_PAGE_SIZE];
	char read[AT45DB_PAGE_SIZE];
	int i, j;
	rt_tick_t tick_start,tick_end;
	
	flash_hw_init();
	
	for(i =0; i< AT45DB_PAGE_SIZE; i++ )
	{
		buf[i] = i;
		read[i] = 0;
	}

	// step 1:find device
	device = rt_device_find("flash0");
	if( device == RT_NULL)
	{
			rt_kprintf("device %s: not found!\r\n");
			return RT_ERROR;
	}
	device->open(device,RT_DEVICE_FLAG_RDWR);
	
	tick_start = rt_tick_get();
	rt_kprintf("testing begin at %d tick\n",tick_start);
	for(j=0;j<AT45DB_MAX_PAGE;j++) 
	{
		device->write(device,j, buf,1);
		device->read(device,j, read,1);
		for(i =0; i< 512; i++ )
		{
			if( buf[i] != read[i] ) {
				rt_kprintf("error at %d:%d: %d!=%d\n", j,i, buf[i], read[i]);
				goto out;
			}
		}
	}

out:
	tick_end = rt_tick_get();
	rt_kprintf("testing end at %d tick\n",tick_end);
	rt_kprintf("testing used %d tick\n",tick_end - tick_start);
	device->close(device);
	rt_kprintf("Finsh test\n");
	return 0;
}


int flash_p(int x)
{
	SPI_FLASH_PageErase(x);
	rt_kprintf("Finsh test\n");
	return 0;
}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(flash_w, test flash system);
FINSH_FUNCTION_EXPORT(flash_r, test flash system);
FINSH_FUNCTION_EXPORT(flash_a, test flash system);
FINSH_FUNCTION_EXPORT(flash_p, test flash system);
#endif

#endif
