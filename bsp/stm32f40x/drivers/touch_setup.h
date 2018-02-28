#ifndef __TOUCH_SETUP_H_  
#define __TOUCH_SETUP_H_  
#include <rtthread.h>  
#include <rtgui/calibration.h>  
  
rt_err_t cali_load(struct calibration_data* setup);  
rt_err_t cali_save(struct calibration_data* setup);  
rt_bool_t calibration_restore(void);  
void calibration_store(struct calibration_data *data);  
//the pointer to load user data ,need to initialize  
void calibration_set_restore(rt_bool_t (*calibration_restore)(void));  
//the pointer to save user data ,need to initialize  
void calibration_set_after(void (*calibration_after)(struct calibration_data *data));  
  
#endif  
