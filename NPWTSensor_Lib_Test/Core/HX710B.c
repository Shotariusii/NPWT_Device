/*
 * HX710B.c
 *
 *  Created on: Mar 4, 2023
 *      Author: shota
 */


#include "HX710B.h"

void Sensor_Module_Init(HX710B_mode_t mode){
	DWT_Init();
	Sensor_Delay(1);

	for(int i=0; i<mode; i++){
		CLK_HIGH;
		DWT_Delay(1);
		CLK_LOW;
		DWT_Delay(1);
	}

}



void isReady(void)
{
	while(DOUT_READ) ;
	Sensor_Delay(1);
}





void Read_Sensor(uint32_t *data){

	CLK_LOW;
	isReady();

	uint32_t raw = 0;
	for(int i=0; i<27; i++){
		CLK_HIGH;
		DWT_Delay(1);
		CLK_LOW;
		DWT_Delay(1);
		if(i < 24){
			raw = raw << 1;
			if(DOUT_READ) raw = raw | 0x01;;
		}
	}
	*data = raw ^ 0x800000;

	//CLK_HIGH;


}
