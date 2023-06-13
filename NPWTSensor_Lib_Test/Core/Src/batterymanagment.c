#include "baterrymanagment.h"


bms_init_t bat_managment_init(battery_managment_t *ptr){
	
	if(Ina226_Init(ptr->ina_config) == INA226_OK) return bms_init_ok;
	return bms_init_error;
}


	int temp_ina_current=0;
 uint32_t temp_voltage_value=0;

void bat_managment_processloop(battery_managment_t *ptr){
	
	
	INA226_data_availability_t ina226_update_status;
	chargin_state_t temp_state;
	
	
	temp_state=(chargin_state_t)bq2057_GetCondition(ptr->bq2057csn_condition);
	

	if(INA226_Check_data(ptr->ina_config) == ina226_data_updated){
		temp_ina_current=INA226_Get_Current(ptr->ina_config);	
		if((temp_ina_current > 0) && ( temp_ina_current < BMS_CHARGING_CURRENT_LOW_TRESHOLD)  && (temp_state == bms_charging_complete)) {
			ptr->charging_state = bms_charging_complete;
		}else if((temp_ina_current > 0) && (temp_state == bms_charging )) {
			ptr->charging_state = bms_charging;
		}else{
			ptr->charging_state = bms_charging_removed;
		}
	
		temp_voltage_value=INA226_Get_VBus_Voltage(ptr->ina_config);
		ptr->current_soc = (uint8_t)(temp_voltage_value  -  BMS_LOW_VOLTAGE_TRESHOLD )/10 ;
		
		if(temp_voltage_value < BMS_LOW_VOLTAGE_TRESHOLD ) {
			ptr->op_voltage=below_operational_voltage;
		}else{
			ptr->op_voltage=operational_voltage;
		}		
	}	
}






void bms_set_operation_mode(battery_managment_t *ptr, bms_operation_mode_t mode){
	
	if(mode == bms_sleep_mode ) {
		ptr->ina_config->Mode=Mode_Power_Down;
		INA226_Set_Mode(ptr->ina_config);
			
	}else{
		ptr->ina_config->Mode=Mode_Bus_And_Shunt_Cont;
		INA226_Set_Mode(ptr->ina_config);
		
	}
	
	
}







 