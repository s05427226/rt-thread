#include <rtthread.h>  
#include <dfs_posix.h>  
#include <rtgui/calibration.h>  
#include "stdlib.h"//for atoi  
#include <touch_setup.h>  
	
#define cali_file    "/cali.ini"  
	
static const char* kn_touch_min_x = "touch_min_x";  
static const char* kn_touch_max_x = "touch_max_x";  
static const char* kn_touch_min_y = "touch_min_y";  
static const char* kn_touch_max_y = "touch_max_y";  

void show_main_window(struct calibration_data *data)
{
	extern void application_init(void);
	application_init();
}

static rt_uint32_t read_line(int fd, char* line, rt_uint32_t line_size)  
{  
		char *pos, *next;  
		rt_uint32_t length;  
	
		length = read(fd, line, line_size);  
		if (length > 0)  
		{  
				pos = strstr(line, "\r\n");  
				if (pos == RT_NULL)  
				{  
						pos = strstr(line, "\n");  
						next = pos ++;  
				}  
				else next = pos + 2;  
	
				if (pos != RT_NULL)  
				{  
						*pos = '\0';  
	
						/* move back */  
						lseek(fd, -(length - (next - line)), SEEK_CUR);  
	
						length = pos - line;  
				}  
				else length = 0;  
		}  
	
		return length;  
}  
  
static void cali_load_default(void)  
{  
	/* 
	struct calibration_data setup; 
 
	rt_kprintf("cali_load_default!\r\n"); 
		//OTM4001 default calibration data 
	setup.min_x = 0x3aa8; 
	setup.max_x = 0x792f; 
	setup.min_y = 0xd7f1; 
	setup.max_y = 0x573b; 
 
	cali_save(&setup); 
	*/  
}  
  
rt_err_t cali_load(struct calibration_data* setup)  
{  
	int fd, length;  
	char line[64];  
  
	rt_kprintf("Load calibration data\n");  
  
	fd = open(cali_file, O_RDONLY, 0);  
	if (fd >= 0)  
	{  
		length = read_line(fd, line, sizeof(line));  
		if (strcmp(line, "[config]") == 0)  
		{  
			char* begin;  
  
			// touch_min_x  
			length = read_line(fd, line, sizeof(line));  
			if (length == 0)  
			{  
				close(fd);  
				cali_load_default();  
				return RT_EOK;  
			}  
			if (strncmp(line, kn_touch_min_x, sizeof(kn_touch_min_x) - 1) == 0)  
			{  
				begin = strchr(line, '=');  
				begin++;  
				setup->min_x = atoi(begin);  
			}  
  
			// touch_max_x  
			length = read_line(fd, line, sizeof(line));  
			if (length == 0)  
			{  
				close(fd);  
				cali_load_default();  
				return RT_EOK;  
			}  
			if (strncmp(line, kn_touch_max_x, sizeof(kn_touch_max_x) - 1) == 0)  
			{  
				begin = strchr(line, '=');  
				begin++;  
				setup->max_x = atoi(begin);  
			}  
  
			// touch_min_y  
			length = read_line(fd, line, sizeof(line));  
			if (length == 0)  
			{  
				close(fd);  
				cali_load_default();  
				return RT_EOK;  
			}  
			if (strncmp(line, kn_touch_min_y, sizeof(kn_touch_min_y) - 1) == 0)  
			{  
				begin = strchr(line, '=');  
				begin++;  
				setup->min_y = atoi(begin);  
			}  
  
			// touch_max_y  
			length = read_line(fd, line, sizeof(line));  
			if (length == 0)  
			{  
				close(fd);  
				cali_load_default();  
				return RT_EOK;  
			}  
			if (strncmp(line, kn_touch_max_y, sizeof(kn_touch_max_y) - 1) == 0)  
			{  
				begin = strchr(line, '=');  
				begin++;  
				setup->max_y = atoi(begin);  
			}  
		}  
		else  
		{  
			close(fd);  
			cali_load_default();  
			return RT_EOK;  
		}  
	}  
	else  
	{  
		close(fd);  
		cali_load_default();  
		return -RT_ERROR;  
	}  
  
	close(fd); 
	show_main_window(RT_NULL);	
	return RT_EOK;  
}  
  
rt_err_t cali_save(struct calibration_data* cali)  
{  
	int fd, size;  
	char* p_str;  
	char* buf = rt_malloc(1024);  
  
	if (buf == RT_NULL)  
	{  
		rt_kprintf("no memory\r\n");  
		return RT_ENOMEM;  
	}  
  
	p_str = buf;  
  
	fd = open(cali_file, O_WRONLY | O_TRUNC, 0);  
	if (fd >= 0)  
	{  
		size = sprintf(p_str, "[config]\r\n"); // [config] sprintf(p_str,"")  
		p_str += size;  
  
		size = sprintf(p_str, "%s=%d\r\n", kn_touch_min_x, cali->min_x); //touch_min_x  
		p_str += size;  
  
		size = sprintf(p_str, "%s=%d\r\n", kn_touch_max_x, cali->max_x); //touch_max_x  
		p_str += size;  
  
		size = sprintf(p_str, "%s=%d\r\n", kn_touch_min_y, cali->min_y); //touch_min_y  
		p_str += size;  
  
		size = sprintf(p_str, "%s=%d\r\n", kn_touch_max_y, cali->max_y); //touch_max_y  
		p_str += size;  
	}  
  
	size = write(fd, buf, p_str - buf);  
	if (size == (p_str - buf))  
	{  
		rt_kprintf("file write succeed:\r\n");  
	}  
  
	close(fd);  
	rt_free(buf);  
  
	return RT_EOK;  
}  
  
/** This let the user space to restore the last calibration data. 
 * 
 * calibration_restore is a callback before calibration started. If it returns 
 * RT_TRUE, the calibration won't be started. In this condition, you must setup 
 * the calibration_data via something like: 
 * 
 *     device = rt_device_find("touch"); 
 *     if(device != RT_NULL) 
 *         rt_device_control(device, RT_TOUCH_CALIBRATION_DATA, &data); 
 * 
 * It it returns RT_FALSE, the normal calibration process will be started. If 
 * you don't have such feature, there is no need to call this function. The 
 * calibration will always be started by RTGUI. 
 */  
rt_bool_t calibration_restore(void)  
{  
	//rt_kprintf("cali setup entered\n");  
		struct calibration_data setup;   
		if(cali_load(&setup) == RT_EOK)   
		{   
			 struct calibration_data data;   
			 rt_device_t device;   
			 /* update the calibration data  */   
			 data.min_x = setup.min_x;   
			 data.max_x = setup.max_x;   
			 data.min_y = setup.min_y;   
			 data.max_y = setup.max_y;   
			 rt_kprintf("cali finished (%d, %d), (%d, %d)\n",  
			   data.min_x,  
			   data.max_x,  
			   data.min_y,  
			   data.max_y);  
			 device = rt_device_find("touch");   
			 if(device != RT_NULL)   
					 rt_device_control(device, RT_TOUCH_CALIBRATION_DATA, &data);   
			 return RT_TRUE;   
		}   
		/* if have no the calibration data,return false  */   
	return RT_FALSE;      
}  

/** This provide ways to save the calibration_data to user space. 
 * 
 * calibration_after is a callback after the calibration has finished.  User 
 * space could use this function to save the data to some where else. No need 
 * to call this if you don't have such function. 
 */  
void calibration_store(struct calibration_data *data)  
{  
		struct calibration_data store;   
		store.min_x = data->min_x;   
		store.max_x = data->max_x;   
		store.min_y = data->min_y;   
		store.max_y = data->max_y;   
		cali_save(&store);   
	rt_kprintf("cali finished (%d, %d), (%d, %d)\n",  
			   data->min_x,  
			   data->max_x,  
			   data->min_y,  
			   data->max_y);  
		show_main_window(data);
} 
