/*
 * HX710B.h
 *
 *  Created on: Mar 4, 2023
 *      Author: shota
 */

#ifndef INC_HX710B_H_
#define INC_HX710B_H_

#include "main.h"
#include "dwt.h"


/* calibration value  val 2.98023e-7 */ 

#define Sensor_Delay(x) HAL_Delay(x)
/* Define HX710B Pressure Sensor Module Communication Pins */

/* input pin with pull up */
#define SENSOR_DOUT_PIN   Sensor_Out_Pin
#define SENSOR_DOUT_PORT  Sensor_Out_GPIO_Port

/* output  */
#define SENSOR_CLK_PIN   Sensor_Clock_Pin
#define SENSOR_CLK_PORT  Sensor_Clock_GPIO_Port

#define Micro_Delay(x) DWT_Delay(x)


#define DOUT_READ 			 HAL_GPIO_ReadPin(SENSOR_DOUT_PORT, SENSOR_DOUT_PIN)
#define CLK_HIGH 				 HAL_GPIO_WritePin(SENSOR_CLK_PORT, SENSOR_CLK_PIN, GPIO_PIN_SET)
#define CLK_LOW  				 HAL_GPIO_WritePin(SENSOR_CLK_PORT, SENSOR_CLK_PIN, GPIO_PIN_RESET)
#define ISREAD_DEVICE    while(DOUT_READ);

#define ADC_RESOLUTION 0.0000015
// 25.16 max 
// 0 13.18


typedef enum{
HX710B_DIFF1 = 25,
HX710B_TEMP  = 26,
HX710B_DIFF2 = 27,
}HX710B_mode_t;


typedef enum{
	
	HX710B_sleep_state	 = 0x01U,
	HX710B_working_state = 0x00U,
	
	
}HX710B_state_t;




typedef enum {
	
	HX710B_Working_Range  = 0x00U, 
	HX710B_Below_To_Range = 0x01U,
	HX710B_Above_To_Range = 0x02U,
	
}HX710B_current_range_t;


typedef struct{
	
	HX710B_mode_t mode;
	HX710B_state_t state;
	HX710B_current_range_t current_working_range;
	
}HX710B_config_t;



void  HX710B_Sensor_Module_Init(HX710B_config_t *config);
float HX710B_Read_Sensor(HX710B_config_t *config);
void  HX710B_Enter_Sleep(HX710B_config_t *config);
void  HX710B_Exit_Sleep(HX710B_config_t *config);
void  HX710B_Set_Mode(HX710B_config_t *config);








#endif /* INC_HX710B_H_ */
