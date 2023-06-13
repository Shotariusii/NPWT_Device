#ifndef  BQ2057CSN_H_
#define  BQ2057CSN_H_


// Charging Current 1470 // charging end current 680 ma

#include "main.h"


#define bq2057_delay(x) osDelay(x)

typedef enum{
	
	battery_charging_complete = 0x00U,
	battery_charging,
	sleepmode_or_fault,
	
}bq2057csn_condition_t;



typedef struct{
	
		GPIO_TypeDef* pullup_pin_port;
	  GPIO_TypeDef* stat_pin_port;
		uint16_t      pullup_pin; 
	  uint16_t      stat_pin;
	 	
}bq2057csn_config_t;



bq2057csn_condition_t bq2057_GetCondition(bq2057csn_config_t *ptr);





#endif /* BQ2057CSN_H_ */
