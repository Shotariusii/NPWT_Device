#ifndef  BATTERYMANAGMENT_H_
#define  BATTERYMANAGMENT_H_


/* 

the battery soc level is controlled with Voltage, so The realtives voltage and battery 
Soc is not proportional thas why the SoC level could be wrong based on voltage. 

*/ 

#include "main.h"
#include "bq2057csn.h"
#include "ina226.h"



#define  LI_ON_CELL_MAX_VOLTAGE   4200U // mv

#define  BATTERY_FACTORY_COLUMBS  2000.0f  // columbs mah
#define  CURRENT_SAMPLE_TIME 1.13f    // in sc
#define  BMS_LOW_VOLTAGE_TRESHOLD   3200U // mv
#define  BMS_CHARGING_CURRENT_HIGH_TRESHOLD 
#define  BMS_CHARGING_CURRENT_LOW_TRESHOLD  500 // ma



typedef enum{
	
	bms_charging_complete = 0x00u, 
	bms_charging,
	bms_charging_removed
	
}chargin_state_t;

 
typedef enum{
	
	bms_init_ok = 0x00U,
	bms_init_error
	

}bms_init_t;


typedef enum{
	
	bms_operation_mode = 0x00U,
	bms_sleep_mode,

}bms_operation_mode_t;


typedef enum{
	
	operational_voltage = 0x00U,
	below_operational_voltage,
	
}bms_voltage_t;
	



typedef struct {
	
	uint8_t current_soc;
	chargin_state_t charging_state;
	bms_voltage_t op_voltage;
	
	struct INA226_Configuration_Struct *ina_config;
	bq2057csn_config_t *bq2057csn_condition;
	
}battery_managment_t;


bms_init_t bat_managment_init(battery_managment_t *ptr);

void bat_managment_processloop(battery_managment_t *ptr);

void bms_set_operation_mode(battery_managment_t *ptr, bms_operation_mode_t mode);





































#endif 



