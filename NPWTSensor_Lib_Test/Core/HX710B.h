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


#define Sensor_Delay(x) HAL_Delay(x)
/* Define HX710B Pressure Sensor Module Communication Pins */

/* input pin with pull up */
#define DOUT_PIN  GPIO_PIN_9
#define DOUT_PORT GPIOB

/* output  */
#define CLK_Pin   GPIO_PIN_8
#define CLK_PORT  GPIOB


typedef enum{
HX710B_DIFF1 = 25,
HX710B_TEMP  = 26,
HX710B_DIFF2 = 27,
}HX710B_mode_t;



#define DOUT_READ HAL_GPIO_ReadPin(DOUT_PORT, DOUT_PIN)
#define CLK_HIGH  HAL_GPIO_WritePin(CLK_PORT, CLK_Pin, GPIO_PIN_SET)
#define CLK_LOW   HAL_GPIO_WritePin(CLK_PORT, CLK_Pin, GPIO_PIN_RESET)



void Sensor_Module_Init(HX710B_mode_t mode);
void isReady(void);
void Read_Sensor(uint32_t *data);






#endif /* INC_HX710B_H_ */
