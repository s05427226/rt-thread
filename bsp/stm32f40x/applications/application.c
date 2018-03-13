/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2014-04-27     Bernard      make code cleanup. 
 */

#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32f4xx_eth.h"
#endif

#ifdef RT_USING_GDB
#include <gdb_stub.h>
#endif

#ifdef RT_USING_GUIENGINE
#include "drv_lcd.h"
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif

#ifdef RT_USING_DFS
/* dfs init */
//#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>
#endif

#ifdef RT_USING_USB_DEVICE
extern void rt_hw_usbd_init(void);
extern rt_err_t rt_usb_device_init(void);
extern void rt_hw_ramdisk_init(void);
#endif

#ifdef RT_USING_GUIENGINE
//#define TEST_CALIBRATION
#include "touch_setup.h"
void load_touch_cfg()
{
	calibration_set_restore(calibration_restore);//initialize the pointer to load user data
	calibration_set_after(calibration_store); //initialize the poiter to save user data
	calibration_init();

}

#endif

void rt_init_thread_entry(void* parameter)
{
    /* GDB STUB */
#ifdef RT_USING_GDB
    gdb_set_device("uart6");
    gdb_start();
#endif

	/* Filesystem Initialization */
#ifdef RT_USING_DFS
	{
		extern void flash_hw_init(void);
		flash_hw_init();
		/* init the device filesystem */
		dfs_init();

#ifdef RT_USING_DFS_ELMFAT
		/* init the elm chan FatFs filesystam*/
		elm_init();
	#ifdef TEST_CALIBRATION
		dfs_mkfs("elm", "flash0");
	#endif
		if (dfs_mount("flash0", "/", "elm", 0, 0) == 0)
		{
			rt_kprintf("flash0 mount to / success!\n");
		}
		else
		{
			rt_kprintf("flash0 mount to / failed!\n");
			dfs_mkfs("elm", "flash0");
			dfs_mount("flash0", "/", "elm", 0, 0);
		}

#endif
	}
#endif

    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif
	
		{
			extern int rt_hw_ex_ram_init(void);
			rt_hw_ex_ram_init();
		}
		
#ifdef RT_USING_GUIENGINE
		{
			rt_device_t device;
			
			rt_hw_lcd_init();

			device = rt_device_find("lcd");
			/* re-set graphic device */
			rtgui_graphic_set_device(device);
			
			rtgui_system_server_init();
		}

		{
			extern void rtgui_touch_hw_init(void);
			rtgui_touch_hw_init();

			load_touch_cfg();
		}
#endif
		
#ifdef RT_USING_USB_DEVICE
    /* init disk first */
    rt_hw_ramdisk_init();
    /* usb device controller driver initilize */
    rt_hw_usbd_init();
#endif
}

int rt_application_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}

/*@}*/
