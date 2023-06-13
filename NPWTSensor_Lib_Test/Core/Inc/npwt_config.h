#ifndef NPWT_CONFIG_H_
#define NPWT_CONFIG_H_



#include "main.h" 




#define INITIAL_MMHG_SET_VALUE    120

/*
 @brief masks for internal communication between tasks to disp task, 
*/


#define MASK_FROM_SYSTEM_TASK        0x80000000
#define MASK_FROM_BMS_TASK    			 0x40000000   
#define MASK_FROM_BUTTON_TASK   		 0x20000000

#define MASK_FOR_SET_FLAG            0x10000000  
#define MASK_FOR_UNSET_FLAG          0xEFFFFFFF

#define MASK_FOR_SYSTEM_ON           MASK_FOR_SET_FLAG
#define MASK_FOR_SYSTEM_OFF          MASK_FOR_UNSET_FLAG
#define MASK_FOR_CHARGING_ON         MASK_FOR_SET_FLAG
#define MASK_FOR_CHARGING_OFF        MASK_FOR_UNSET_FLAG


#define MASK_FOR_SYSTEM_ERROR        0x08000000

#define MASK_FOR_CLEARING_COMUNICATION_FLAGS  0x00FFFFFF


#define SYSTEM_PREASURE_MAX_VALUE    1000
#define SYSTEM_PREASURE_MIN_VALUE    10



#define MODE_BUTTON_PRESSED_FLAG  0x00000001
#define NPWT_SYSTEM_ON            0x01U 
#define NPWT_SYSTEM_OFF           0x00U 








#endif 




