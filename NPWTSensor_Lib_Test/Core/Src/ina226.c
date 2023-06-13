#include "ina226.h"







INA226_StatusTypeDef Ina226_Software_Reset(struct INA226_Configuration_Struct *Struct_V){
	
		uint8_t Buffer[3] ={0};	
				
    Buffer[0] = ADDRESS_OF_CONFIGURATION_REGISTER;
		Buffer[1] = 0x80;
    Buffer[2] = 0x00;	
		
		return (INA226_StatusTypeDef)HAL_I2C_Master_Transmit(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1),Buffer,3,HAL_MAX_DELAY);
	  
}



INA226_StatusTypeDef Ina226_Init(struct INA226_Configuration_Struct *Struct_V)
{
	uint8_t Buffer[3] ={0};														
  HAL_StatusTypeDef Temp_Status_Data=HAL_ERROR;
	
	Buffer[0] = ADDRESS_OF_CONFIGURATION_REGISTER;
	Buffer[1] = (Struct_V->Avarage_Num <<1 ) | (Struct_V->Vbus_Conv_Time >> 2) ;
	Buffer[2] = (Struct_V->Vbus_Conv_Time << 6) | (Struct_V->Vshunt_Conv_Time << 3) | Struct_V->Mode;
	
	Temp_Status_Data=HAL_I2C_IsDeviceReady(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1),HAL_DEVICEREADY_TRIAL_NUMBER,HAL_I2C_TIMOUT);
	if(Temp_Status_Data != HAL_OK ){
		return (INA226_StatusTypeDef)Temp_Status_Data;
	}

	Temp_Status_Data=HAL_I2C_Master_Transmit(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1),Buffer,3,HAL_I2C_TIMOUT);// set Configuration setting
	if(Temp_Status_Data != HAL_OK ){
		return (INA226_StatusTypeDef)Temp_Status_Data;
	}
	
	float Calibration_Value=(0.00512F / (Struct_V->Current_LSB * Struct_V->Shunt_Resistor_Val));


	Buffer[0]=ADDRESS_OF_CALIBRATIORN_REGISTER ;
	Buffer[1]=(uint8_t)(Calibration_Value/256) ;
	Buffer[2]=(uint8_t)Calibration_Value ;
	 
	Temp_Status_Data=HAL_I2C_Master_Transmit(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1),Buffer,3,HAL_I2C_TIMOUT); // set Calibration Value
  
  if(	Temp_Status_Data != HAL_OK) {
		return (INA226_StatusTypeDef)Temp_Status_Data;
	}
	
	return INA226_OK;
}




uint32_t INA226_Get_VBus_Voltage(struct INA226_Configuration_Struct *Struct_V){

	uint8_t Buffer[2]={0};
	uint32_t Read_Voltage_Value=0;
	
	
	Buffer[0]=ADDRESS_OF_BUS_VOLTAGE_REGISTER;
  HAL_I2C_Master_Transmit(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1),Buffer,1,HAL_I2C_TIMOUT);   // pointing bus voltage reg

  HAL_I2C_Master_Receive(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1) | 0x01,Buffer,2,HAL_I2C_TIMOUT);  // reading bus voltage
	

	Read_Voltage_Value = (int)(((Buffer[0] << 8) | Buffer[1])*LSB_FOR_BUS*1000);
	
	return Read_Voltage_Value;
	
	
}






int INA226_Get_Current(struct INA226_Configuration_Struct *Struct_V){
	
	
	uint8_t Buffer[2]={0};
	uint16_t Temp_Data=0;
	int Read_Current_Value=0;
	
	Buffer[0] = ADDRESS_OF_CURRENT_REGISTER;
	HAL_I2C_Master_Transmit(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1),Buffer,1,HAL_I2C_TIMOUT);// pointing Current_Reg voltage reg
	HAL_I2C_Master_Receive(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1) | 0x01,Buffer,2,HAL_I2C_TIMOUT);
 
	
	if(Buffer[0] & (1<<7))   // check sign bit
	{
		Temp_Data = ~(((Buffer[0] << 8) | Buffer[1]) -1 ) ;// reverse twos complement;	
		Read_Current_Value = (int)(0-(1000*(Temp_Data * Struct_V->Current_LSB)));		
	} 
	else 
	{
	  Read_Current_Value = (int)((((Buffer[0] << 8) | Buffer[1])*Struct_V->Current_LSB)*1000);
	}
	
	return Read_Current_Value;
	
}




INA226_data_availability_t INA226_Check_data(struct INA226_Configuration_Struct *Struct_V){
	
		uint8_t Buffer[2]={0};
	
	
	Buffer[0]=ADDRESS_OF_MASK_ENABLE_REGISTER;
  HAL_I2C_Master_Transmit(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1),Buffer,1,HAL_I2C_TIMOUT);   

  HAL_I2C_Master_Receive(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1) | 0x01,Buffer,2,HAL_I2C_TIMOUT);  
	
	
  if(Buffer[1] & 8 ) return ina226_data_updated;
		
	return ina226_data_is_processing;
	
	
	
}


void INA226_Set_Mode(struct INA226_Configuration_Struct *Struct_V){
	uint8_t Buffer[3] ={0};

	
	if(Struct_V->Mode == Mode_Power_Down ) {
	Buffer[0] = ADDRESS_OF_CONFIGURATION_REGISTER;
	Buffer[1] = (Struct_V->Avarage_Num <<1 ) | (Struct_V->Vbus_Conv_Time >> 2) ;
	Buffer[2] = (Struct_V->Vbus_Conv_Time << 6) | (Struct_V->Vshunt_Conv_Time << 3) | Struct_V->Mode;			
		
	HAL_I2C_Master_Transmit(&(Struct_V->I2C_Channel),(Struct_V->Slave_Addrs << 1),Buffer,3,HAL_I2C_TIMOUT);// set Configuration setting 
		
	}else{
		Ina226_Init(Struct_V);
	}
	
	
}








