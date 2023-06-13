

#ifndef INA226_H_
#define INA226_H_



#include "main.h"

/*
Current_Lsb = Maximum_expected_Current / 2^15
*/


#define HAL_I2C_TIMOUT               1000 
#define HAL_DEVICEREADY_TRIAL_NUMBER 50



#define LSB_FOR_BUS   0.00125 // The LSB for the Bus Voltage Register (02h) is a fixed 1.25 mV/bit

// Ina226 Register address 

#define ADDRESS_OF_CONFIGURATION_REGISTER		0x00
#define ADDRESS_OF_SHUNT_VOLTAGE_REGISTER   0x01
#define ADDRESS_OF_BUS_VOLTAGE_REGISTER 	  0x02
#define ADDRESS_OF_POWER_REGISTER        	  0x03
#define ADDRESS_OF_CURRENT_REGISTER      	  0x04
#define ADDRESS_OF_CALIBRATIORN_REGISTER    0x05
#define ADDRESS_OF_MASK_ENABLE_REGISTER     0x06
#define ADDRESS_OF_ALERT_REGISTER           0x07
#define ADDRESS_OF_MANUFACTURERID_REGISTER  0xFE
#define ADDRESS_OF_DIE_ID_REGISTER          0xFF

/*                                  */

typedef enum {
 Mode_Bus_And_Shunt_Cont=0x07,
 Mode_Shunt_Cont=0x05,
 Mode_Bus_Cont=0x06,
 Mode_Power_Down=0x00
}Operation_Mode;

typedef enum {		
 Conv_Shunt_140us,       
 Conv_Shunt_204us,      
 Conv_Shunt_332us,       
 Conv_Shunt_588us,      
 Conv_Shunt_1_1ms,       
 Conv_Shunt_2_116ms,     
 Conv_Shunt_4_156ms,     
 Conv_Shunt_8_244ms     
}Vshunt_Conversion_Time;

typedef enum {	
 Conv_Vbus_140us,       
 Conv_Vbus_204us,       
 Conv_Vbus_332us,       
 Conv_Vbus_588us,       
 Conv_Vbus_1_1ms,       
 Conv_Vbus_2_116ms,     
 Conv_Vbus_4_156ms,     
 Conv_Vbus_8_244ms    
}Vbus_Conversion_Time;

typedef enum{
 Set_Avg_1,                     
 Set_Avg_4,               
 Set_Avg_16,            
 Set_Avg_64,            
 Set_Avg_128,             
 Set_Avg_256,           
 Set_Avg_512,             
 Set_Avg_1024          
}Samples_Avarage_Number;


typedef enum
{
  INA226_OK,      
  INA226_ERROR,    
  INA226_BUSY,     
  INA226_TIMEOUT 
} INA226_StatusTypeDef;


typedef enum{
	
	ina226_data_updated = 0x01U,
	ina226_data_is_processing

	
}INA226_data_availability_t;



struct INA226_Configuration_Struct{
	
	I2C_HandleTypeDef       I2C_Channel;
	uint8_t                 Slave_Addrs;
	float                   Current_LSB;
	float                   Shunt_Resistor_Val;

	Operation_Mode          Mode;
	Vshunt_Conversion_Time  Vshunt_Conv_Time;
	Vbus_Conversion_Time    Vbus_Conv_Time;
	Samples_Avarage_Number  Avarage_Num;
	
};


/**
 * @brief the function configures ina226 and checks if Communication is succsesfuly 
   @param INA226_Configuration_Struct type pointer, from where it takes the corresponding config data and slave address
   @retval   INA226_StatusTypeDef 
 */


INA226_StatusTypeDef Ina226_Init(struct INA226_Configuration_Struct *Struct_V); // Sets INA226 Configuration Registers and Calibration Value


/**
 * @brief the function resets ina226 to default configuration
   @param INA226_Configuration_Struct type pointer, from where it takes the corresponding addres 
   @retval   INA226_StatusTypeDef 
 */

INA226_StatusTypeDef Ina226_Software_Reset(struct INA226_Configuration_Struct *Struct_V); //System Reset, Set Configuration setting to default

/**
 * @brief the function measures voltage and returns it in "ma" unit
   @param INA226_Configuration_Struct type pointer, from where it takes the corresponding addres 
   @retval   uint32_t 
 */

uint32_t INA226_Get_VBus_Voltage(struct INA226_Configuration_Struct *Struct_V); 

/**
 * @brief the function measures current and returns it in "ma" unit
   @param INA226_Configuration_Struct type pointer, from where it takes the corresponding addres 
   @retval   signed int
 */
int INA226_Get_Current(struct INA226_Configuration_Struct *Struct_V);  // Get Shunt current 




/**
 * @brief the function checks if ina226 has updated data in register
   @param INA226_Configuration_Struct type pointer, from where it takes the corresponding addres 
   @retval   INA226_data_availability_t type status
 */
 
INA226_data_availability_t INA226_Check_data(struct INA226_Configuration_Struct *Struct_V);




/**
 * @brief the function sets ina226 operation modes
   @param INA226_Configuration_Struct type pointer, from where it takes the corresponding addres 
   @retval  void
 */
 
void INA226_Set_Mode(struct INA226_Configuration_Struct *Struct_V);


#endif /* INA226_H_ */


