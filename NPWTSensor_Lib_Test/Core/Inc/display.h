#ifndef DISPLAY_H_
#define DISPLAY_H_


#include "main.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>


#define FIRST_BLOCK 93
#define SECOND_BLOCK 99
#define THIRD_BLOCK 105
#define FOURTHD_BLOCK 111


#define CHARGING_LOGO_X  90
#define CHARGING_LOGO_Y  0


#define WORKING_AREA_X  5
#define WORKING_AREA_Y  24
#define WORKING_WITH_X  115
#define WORKING_WITH_Y  42


#define LOGO_X_PIXEL 33
#define LOGO_Y_PIXEL 31


typedef enum{
	
	charging = 0x00u,
	operating,
	
}charging_status_t;


typedef enum{
	
	display_sleep = 0x00u,
	display_wake,
	dispay_setting,
	display_working,
	
}display_mode_t;
	
	
	


extern I2C_HandleTypeDef hi2c2;

#define I2C_HANDLER hi2c2

HAL_StatusTypeDef display_init(void);

void set_bms_soc(uint8_t soc,charging_status_t mode);

void set_preasure(uint32_t data);

void display_clear(void); 

void display_mode(display_mode_t mode);















#endif


