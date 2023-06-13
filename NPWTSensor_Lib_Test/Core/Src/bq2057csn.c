
#include "bq2057csn.h"





bq2057csn_condition_t bq2057_GetCondition(bq2057csn_config_t *ptr){
	
	GPIO_PinState temp_pin_state;
	
	temp_pin_state=HAL_GPIO_ReadPin(ptr->stat_pin_port,ptr->stat_pin);
	HAL_GPIO_WritePin(ptr->pullup_pin_port,ptr->pullup_pin,GPIO_PIN_SET);
	bq2057_delay(1);
	
	if(HAL_GPIO_ReadPin(ptr->stat_pin_port,ptr->stat_pin) == GPIO_PIN_SET) {
		if(temp_pin_state == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(ptr->pullup_pin_port,ptr->pullup_pin,GPIO_PIN_RESET);
			return sleepmode_or_fault;
		}
	}
	HAL_GPIO_WritePin(ptr->pullup_pin_port,ptr->pullup_pin,GPIO_PIN_RESET);
	
	return  (bq2057csn_condition_t)temp_pin_state;
	

}
