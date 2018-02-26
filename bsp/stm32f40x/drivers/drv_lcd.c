#include <board.h>
#include <rtthread.h>

#include "stm32f4xx.h"
#include "drv_lcd.h"

_lcd_dev lcd_dev;


void rt_hw_us_delay(rt_uint32_t us)
{
	rt_uint32_t delta;
	rt_uint32_t current_delay;
	us = us * (SysTick->LOAD/(1000000/RT_TICK_PER_SECOND)); 
	delta = SysTick->VAL;
	do
	{
		if ( delta > SysTick->VAL )
			current_delay = delta - SysTick->VAL;
		else
			current_delay = SysTick->LOAD + delta - SysTick->VAL;
	}
	while( current_delay < us );
}

void delay_us(rt_uint16_t us)
{
	rt_hw_us_delay(us);
}

void delay_ms(rt_uint16_t ms)
{
	rt_hw_us_delay(ms*1000);
}

void lcd_write_reg(rt_uint16_t regval)
{
	regval=regval;
	LCD->LCD_REG = regval;
}

void lcd_write_data(rt_uint16_t data)
{
	data=data;
	LCD->LCD_RAM = data;
}

rt_uint16_t lcd_read_ram()
{
	vu16 ram;	
	ram=LCD->LCD_RAM;	
	return ram;	 
}

void lcd_write_reg_with_value(rt_uint16_t reg,rt_uint16_t reg_val)
{
	LCD->LCD_REG = reg;
	LCD->LCD_RAM = reg_val;
}

void lcd_write_ram_prepare(void)
{
	LCD->LCD_REG = lcd_dev.wramcmd;
}

rt_uint16_t lcd_bgr2rgb(rt_uint16_t value)
{
	rt_uint16_t  red, green, blue;

	blue = (value >> 0) & 0x1f;
	green = (value >> 5) & 0x3f;
	red = (value >> 11) & 0x1f;

	return (blue << 11) + (green << 5) + (red << 0);
}

static void lcd_set_cursor(rt_uint16_t Xpos, rt_uint16_t Ypos)
{
	lcd_write_reg(lcd_dev.set_x_cmd);
	lcd_write_data(Xpos >> 8); 
	lcd_write_data(Xpos & 0XFF);

	lcd_write_reg(lcd_dev.set_y_cmd);
	lcd_write_data(Ypos >> 8); 
	lcd_write_data(Ypos & 0XFF);
}

static void lcd_set_scan_direction(rt_uint8_t dir)
{
	rt_uint16_t regval = 0;
	rt_uint16_t dirreg = 0;
	rt_uint16_t temp;

	switch (dir)
	{
	case L2R_U2D://????,????
		regval |= (0 << 7) | (0 << 6) | (0 << 5);
		break;
	case L2R_D2U://????,????
		regval |= (1 << 7) | (0 << 6) | (0 << 5);
		break;
	case R2L_U2D://????,????
		regval |= (0 << 7) | (1 << 6) | (0 << 5);
		break;
	case R2L_D2U://????,????
		regval |= (1 << 7) | (1 << 6) | (0 << 5);
		break;
	case U2D_L2R://????,????
		regval |= (0 << 7) | (0 << 6) | (1 << 5);
		break;
	case U2D_R2L://????,????
		regval |= (0 << 7) | (1 << 6) | (1 << 5);
		break;
	case D2U_L2R://????,????
		regval |= (1 << 7) | (0 << 6) | (1 << 5);
		break;
	case D2U_R2L://????,????
		regval |= (1 << 7) | (1 << 6) | (1 << 5);
		break;
	}

	dirreg = 0X36;
	lcd_write_reg_with_value(dirreg, regval);

	if (regval & 0X20)
	{
		if (lcd_dev.width < lcd_dev.height)//??X,Y
		{
			temp = lcd_dev.width;
			lcd_dev.width = lcd_dev.height;
			lcd_dev.height = temp;
		}
	}
	else
	{
		if (lcd_dev.width > lcd_dev.height)//??X,Y
		{
			temp = lcd_dev.width;
			lcd_dev.width = lcd_dev.height;
			lcd_dev.height = temp;
		}
	}
	
	lcd_write_reg(lcd_dev.set_x_cmd);
	lcd_write_data(0);
	lcd_write_data(0);
	lcd_write_data((lcd_dev.width - 1) >> 8);
	lcd_write_data((lcd_dev.width - 1) & 0XFF);

	lcd_write_reg(lcd_dev.set_y_cmd);
	lcd_write_data(0);
	lcd_write_data(0);
	lcd_write_data((lcd_dev.height - 1) >> 8);
	lcd_write_data((lcd_dev.height - 1) & 0XFF);
}


void lcd_set_backlight(rt_uint8_t pwm)
{
	lcd_write_reg(0xBE);
	lcd_write_data(0x05);
	lcd_write_data(pwm*2.55);
	lcd_write_data(0x01);
	lcd_write_data(0xFF);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
}

void lcd_set_display_direction(rt_uint8_t dir)
{
	lcd_dev.dir = dir;
	if (dir == 0)
	{
		lcd_dev.width = 240;
		lcd_dev.height = 320;
	}
	else
	{
		lcd_dev.width = 320;
		lcd_dev.height = 240;
	}

	lcd_dev.wramcmd = 0X2C;
	lcd_dev.set_x_cmd = 0X2A;
	lcd_dev.set_y_cmd = 0X2B;

//	lcd_set_scan_direction(DFT_SCAN_DIR);
}

static void lcd_clear(rt_uint16_t color)
{
	unsigned int index=0;
	lcd_set_cursor(0,0);
	lcd_write_ram_prepare();                      /* Prepare to write GRAM */
	for (index=0; index<(lcd_dev.height*lcd_dev.width); index++)
	{
			lcd_write_data(color);
	}
}


void _lcd_low_level_init(void)
{
	 GPIO_InitTypeDef  GPIO_InitS;
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 
	FSMC_NORSRAMTimingInitTypeDef  writeTiming;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG, ENABLE);//使能PD,PE,PF,PG时钟  
  RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);//使能FSMC时钟  
	
 
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
	
	//gpio init
	GPIO_InitS.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_9;
	GPIO_Init(GPIOG,&GPIO_InitS);
	
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5,GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9,GPIO_AF_FSMC);
	
	readWriteTiming.FSMC_AddressSetupTime = 0XF;	 //地址建立时间（ADDSET）为16个HCLK 1/168M=6ns*16=96ns	
  readWriteTiming.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（ADDHLD）模式A未用到	
  readWriteTiming.FSMC_DataSetupTime = 60;			//数据保存时间为60个HCLK	=6*60=360ns
  readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
  readWriteTiming.FSMC_CLKDivision = 0x00;
  readWriteTiming.FSMC_DataLatency = 0x00;
  readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 
    
	writeTiming.FSMC_AddressSetupTime =9;	      //地址建立时间（ADDSET）为9个HCLK =54ns 
  writeTiming.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（A		
  writeTiming.FSMC_DataSetupTime = 8;		 //数据保存时间为6ns*9个HCLK=54ns
  writeTiming.FSMC_BusTurnAroundDuration = 0x00;
  writeTiming.FSMC_CLKDivision = 0x00;
  writeTiming.FSMC_DataLatency = 0x00;
  writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 

 
  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2;//  这里我们使用NE4 ，也就对应BTCR[6],[7]。
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; // 不复用数据地址
  FSMC_NORSRAMInitStructure.FSMC_MemoryType =FSMC_MemoryType_SRAM;// FSMC_MemoryType_SRAM;  //SRAM   
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//存储器数据宽度为16bit   
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;// FSMC_BurstAccessMode_Disable; 
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;   
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;	//  存储器写使能
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; // 读写使用不同的时序
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming; //读写时序
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &writeTiming;  //写时序

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //初始化FSMC配置

  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);  // 使能BANK1 
	
	delay_ms(50);
	
	lcd_write_reg(0XD3);
	delay_ms(50);
	lcd_dev.id = lcd_read_ram();
	lcd_dev.id = lcd_read_ram();
	lcd_dev.id = lcd_read_ram();
	lcd_dev.id <<= 8;
	lcd_dev.id |= lcd_read_ram();
	
	lcd_write_reg(0xC8);       //Set EXTC
	lcd_write_data(0xFF);
	lcd_write_data(0x93);
	lcd_write_data(0x42);
	
	lcd_write_reg(0xb6);  
	lcd_write_data(0x0a);
	lcd_write_data(0xE0);//SS GS
 
	lcd_write_reg(0x36); //Memory Access Control
	lcd_write_data(0x08); //MY,MX,MV,ML,BGR,MH
	
	lcd_write_reg(0x3A); //Pixel Format Set
	lcd_write_data(0x55); //DPI [2:0],DBI [2:0]
	
	lcd_write_reg(0xC0); //Power Control 1
	lcd_write_data(0x13); //VRH[5:0]
	lcd_write_data(0x0E); //VC[3:0]
 
	lcd_write_reg(0xC1);  //Power Control 2
	lcd_write_data(0x02); //SAP[2:0],BT[3:0]
 
	lcd_write_reg(0xC5); //VCOM
	lcd_write_data(0XCA); //C8++
	
	lcd_write_reg(0xB1);      
	lcd_write_data(0x00);     
	lcd_write_data(0x1B);
	
	lcd_write_reg(0xB4);      
	lcd_write_data(0x02);
	
	//*****************GAMMA*****************  
	lcd_write_reg(0xE0);
	lcd_write_data(0x00);//P01-VP63   
	lcd_write_data(0x01);//P02-VP62   
	lcd_write_data(0x04);//P03-VP61   
	lcd_write_data(0x00);//P04-VP59   
	lcd_write_data(0x11);//P05-VP57   
	lcd_write_data(0x08);//P06-VP50   
	lcd_write_data(0x35);//P07-VP43   
	lcd_write_data(0x79);//P08-VP27,36
	lcd_write_data(0x45);//P09-VP20   
	lcd_write_data(0x07);//P10-VP13   
	lcd_write_data(0x0D);//P11-VP6    
	lcd_write_data(0x09);//P12-VP4    
	lcd_write_data(0x16);//P13-VP2    
	lcd_write_data(0x17);//P14-VP1    
	lcd_write_data(0x0F);//P15-VP0    
	
	lcd_write_reg(0xE1);
	lcd_write_data(0x00);//P01
	lcd_write_data(0x28);//P02
	lcd_write_data(0x29);//P03
	lcd_write_data(0x02);//P04
	lcd_write_data(0x0F);//P05
	lcd_write_data(0x06);//P06
	lcd_write_data(0x3F);//P07
	lcd_write_data(0x25);//P08
	lcd_write_data(0x55);//P09
	lcd_write_data(0x06);//P10
	lcd_write_data(0x15);//P11
	lcd_write_data(0x0F);//P12
	lcd_write_data(0x38);//P13
	lcd_write_data(0x38);//P14
	lcd_write_data(0x0F);//P15
	//********Window(??/??)****************
	lcd_write_reg(0x2A); //320
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x01);
	lcd_write_data(0x3F);
	
	lcd_write_reg(0x2B); //240
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0x00);
	lcd_write_data(0xEF);

	//****************************** 
	lcd_write_reg(0x11);//Exit Sleep
	delay_ms(120);
	lcd_write_reg(0x29);//Display On
	lcd_write_reg(0x2c); 
	
	lcd_set_display_direction(1);

	lcd_clear(Blue);
}

static rt_err_t lcd_init(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}

static rt_err_t lcd_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t lcd_control(rt_device_t dev, int cmd, void *args)
{
	switch (cmd)
	{
	case RTGRAPHIC_CTRL_GET_INFO:
	{
		struct rt_device_graphic_info *info;

		info = (struct rt_device_graphic_info*) args;
		RT_ASSERT(info != RT_NULL);

		info->bits_per_pixel = 16;
		info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
		info->framebuffer = RT_NULL;
		info->width = 320;
		info->height = 240;
	}
	break;

	case RTGRAPHIC_CTRL_RECT_UPDATE:
		/* nothong to be done */
		break;

	default:
		break;
	}

	return RT_EOK;
}

static void _lcd_set_pixel(const char* pixel, int x, int y)
{
	lcd_set_cursor(x, y);
	lcd_write_ram_prepare();
	LCD->LCD_RAM = *(uint16_t *)pixel;
}

static void _lcd_get_pixel(char* pixel, int x, int y)
{
	rt_uint16_t red = 0;
	rt_uint16_t green = 0;
	rt_uint16_t blue = 0;

	if (x >= lcd_dev.width || y >= lcd_dev.height)
	{
		*(rt_uint16_t*)pixel = 0;
		return;
	}

	lcd_set_cursor(x, y);

	lcd_write_reg(0X2E);
	lcd_read_ram();
	red = lcd_read_ram();
	delay_us(2);

	blue = lcd_read_ram();
	green = red & 0XFF;

	*(rt_uint16_t*)pixel = (((red >> 11) << 11) | ((green >> 10) << 5) | (blue >> 11));
}

static void _lcd_draw_hline(const char* pixel, int x1, int x2, int y)
{
	lcd_set_cursor(x1, y);
	lcd_write_ram_prepare();

	for (; x1 < x2; x1++)
	{
		LCD->LCD_RAM = *(uint16_t *)pixel;
	}
}

static void _lcd_draw_vline(const char* pixel, int x, int y1, int y2)
{
	for (; y1 < y2; y1++)
	{
		_lcd_set_pixel(pixel, x, y1);  //write red data
	}
}

static void _lcd_blit_line(const char* pixels, int x, int y, rt_size_t size)
{
	rt_uint16_t *ptr = (rt_uint16_t*)pixels;

	lcd_set_cursor(x, y);
	lcd_write_ram_prepare();

	while (size--)
	{
		LCD->LCD_RAM = *ptr++;
	}
}

static struct rt_device lcd_device;

static struct rt_device_graphic_ops lcd_ops =
{
	_lcd_set_pixel,
	_lcd_get_pixel,
	_lcd_draw_hline,
	_lcd_draw_vline,
	_lcd_blit_line
};
	
int rt_hw_lcd_init(void)
{
	_lcd_low_level_init();

	/* register lcd device */
	lcd_device.type = RT_Device_Class_Graphic;
	lcd_device.init = lcd_init;
	lcd_device.open = lcd_open;
	lcd_device.close = lcd_close;
	lcd_device.control = lcd_control;
	lcd_device.read = RT_NULL;
	lcd_device.write = RT_NULL;

	lcd_device.user_data = &lcd_ops;

	/* register graphic device driver */
	rt_device_register(&lcd_device, "lcd",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

	return 0;
}
INIT_BOARD_EXPORT(rt_hw_lcd_init);
