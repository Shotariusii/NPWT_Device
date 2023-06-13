/*
 * HX710B.c
 *
 *  Created on: Mar 4, 2023
 *      Author: shota
 */


#include "HX710B.h"

void HX710B_Sensor_Module_Init(HX710B_config_t *config){
	DWT_Init();
	Sensor_Delay(1);
	HX710B_Set_Mode(config);
}


void  HX710B_Set_Mode(HX710B_config_t *config){
	
		for(int i=0; i<config->mode; i++){
		CLK_HIGH;
		Micro_Delay(1);
		CLK_LOW;
		Micro_Delay(1);
	}
	
}

 uint32_t temp_data;


float HX710B_Read_Sensor(HX710B_config_t *config){
	
	HX710B_Exit_Sleep(config);
	ISREAD_DEVICE;
	volatile uint32_t raw = 0;
	float Preausre_Data=0;
	for(int i=0; i<config->mode; i++){
		CLK_HIGH;
		Micro_Delay(1);
		CLK_LOW;
		Micro_Delay(1);
		if(i < 24){
			raw = raw << 1;
			if(DOUT_READ) raw = raw | 0x01;
		}
	}
	
	temp_data=raw;
	raw=raw^0x800000;
	Preausre_Data= raw*ADC_RESOLUTION *128;                              //((raw*ADC_RESOLUTION*1000) + 13.18)/0.2516;
	

	/*
	if(raw  & 0x800000){
	raw=temp_data | 0xff000000;  
	raw=(~raw)+1;	
	Preausre_Data =     raw*ADC_RESOLUTION ; //101325.0 - ((float)raw * ADC_RESOLUTION);	
	}else{
	  raw=raw | 0x00000000;
		Preausre_Data =  raw*ADC_RESOLUTION  ;    //101325.0 + ((float)raw * ADC_RESOLUTION);
	}*/
	
	HX710B_Enter_Sleep(config);
	return Preausre_Data;
	
}




void  HX710B_Enter_Sleep(HX710B_config_t *config){
	
	config->state=HX710B_sleep_state;
	CLK_HIGH;
	Micro_Delay(500);
	
}



void  HX710B_Exit_Sleep(HX710B_config_t *config){
	
	config->state=HX710B_working_state;
	CLK_LOW;
	Micro_Delay(500);
	HX710B_Set_Mode(config);
	
}
