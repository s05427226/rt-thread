#ifndef __H_DRV_LCD
#define __H_DRV_LCD

#include <rtthread.h>

typedef struct
{
	rt_uint16_t LCD_REG;
	rt_uint16_t LCD_RAM;
}LCD_TypeDef;

#define LCD_BASE	((rt_uint32_t) (0x64000000 | 0x0000FFFE))
#define LCD 			((LCD_TypeDef*)LCD_BASE)

typedef struct
{
	rt_uint16_t width;
	rt_uint16_t	height;
	
	rt_uint16_t id;
	
	rt_uint8_t dir;
	
	rt_uint16_t wramcmd;
	rt_uint16_t set_x_cmd;
	rt_uint16_t set_y_cmd;
	
}_lcd_dev;

/* LCD color */
#define White            0xFFFF
#define Black            0x0000
#define Grey             0xF7DE
#define Blue             0x001F
#define Blue2            0x051F
#define Red              0xF800
#define Magenta          0xF81F
#define Green            0x07E0
#define Cyan             0x7FFF
#define Yellow           0xFFE0

int rt_hw_lcd_init(void);

extern _lcd_dev lcd_dev;

//??????
#define L2R_U2D  0 		//从左到右，从上到下
#define L2R_D2U  1 		//????,????
#define R2L_U2D  2 		//????,????
#define R2L_D2U  3 		//????,????
#define U2D_L2R  4 		//????,????
#define U2D_R2L  5 		//????,????
#define D2U_L2R  6 		//????,????
#define D2U_R2L  7		//????,????	 
#define DFT_SCAN_DIR  L2R_U2D  //???????
#endif
