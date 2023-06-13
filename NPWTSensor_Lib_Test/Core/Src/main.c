/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bq2057csn.h"
#include "ina226.h"
#include "baterrymanagment.h"
#include "display.h"
#include "npwt_config.h"
#include "HX710B.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

/* Definitions for mmhgHandler */
osThreadId_t mmhgHandlerHandle;
const osThreadAttr_t mmhgHandler_attributes = {
  .name = "mmhgHandler",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for ButtonHandler */
osThreadId_t ButtonHandlerHandle;
const osThreadAttr_t ButtonHandler_attributes = {
  .name = "ButtonHandler",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BMSHandler */
osThreadId_t BMSHandlerHandle;
const osThreadAttr_t BMSHandler_attributes = {
  .name = "BMSHandler",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayQueue */
osMessageQueueId_t DisplayQueueHandle;
const osMessageQueueAttr_t DisplayQueue_attributes = {
  .name = "DisplayQueue"
};
/* Definitions for mmhgQueue */
osMessageQueueId_t mmhgQueueHandle;
const osMessageQueueAttr_t mmhgQueue_attributes = {
  .name = "mmhgQueue"
};
/* Definitions for I2CSemaphore */
osSemaphoreId_t I2CSemaphoreHandle;
const osSemaphoreAttr_t I2CSemaphore_attributes = {
  .name = "I2CSemaphore"
};
/* Definitions for Mode_Button_Event */
osEventFlagsId_t Mode_Button_EventHandle;
const osEventFlagsAttr_t Mode_Button_Event_attributes = {
  .name = "Mode_Button_Event"
};
/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
void mmhgHandlerTask(void *argument);
void DisplayTask(void *argument);
void ButtonsHandlerTask(void *argument);
void BMSHandlerTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

battery_managment_t battery_managment ;
HX710B_config_t hx710b_config;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableWakeUpPin(Set_Mode_Pin); 
	
	battery_managment.ina_config = (struct INA226_Configuration_Struct*)malloc(sizeof(*battery_managment.ina_config));
  battery_managment.bq2057csn_condition =(bq2057csn_config_t*) malloc(sizeof(*battery_managment.bq2057csn_condition));
	
	battery_managment.bq2057csn_condition->pullup_pin_port=STAT_PULL_UP_GPIO_Port;
	battery_managment.bq2057csn_condition->pullup_pin=STAT_PULL_UP_Pin;
	battery_managment.bq2057csn_condition->stat_pin_port=STAT_GPIO_Port;
	battery_managment.bq2057csn_condition->stat_pin=STAT_Pin;
	
	battery_managment.ina_config->I2C_Channel=hi2c2;
	battery_managment.ina_config->Slave_Addrs=0x40;
	battery_managment.ina_config->Mode=Mode_Bus_And_Shunt_Cont;
	battery_managment.ina_config->Vbus_Conv_Time=Conv_Vbus_1_1ms;
	battery_managment.ina_config->Vshunt_Conv_Time=Conv_Shunt_1_1ms;
	battery_managment.ina_config->Avarage_Num=Set_Avg_512;
	battery_managment.ina_config->Shunt_Resistor_Val=0.01;
	battery_managment.ina_config->Current_LSB=0.000045;
	

	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of I2CSemaphore */
  I2CSemaphoreHandle = osSemaphoreNew(1, 1, &I2CSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of DisplayQueue */
  DisplayQueueHandle = osMessageQueueNew (1, sizeof(uint32_t), &DisplayQueue_attributes);

  /* creation of mmhgQueue */
  mmhgQueueHandle = osMessageQueueNew (1, sizeof(uint32_t), &mmhgQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mmhgHandler */
  mmhgHandlerHandle = osThreadNew(mmhgHandlerTask, NULL, &mmhgHandler_attributes);

  /* creation of Display */
  DisplayHandle = osThreadNew(DisplayTask, NULL, &Display_attributes);

  /* creation of ButtonHandler */
  ButtonHandlerHandle = osThreadNew(ButtonsHandlerTask, NULL, &ButtonHandler_attributes);

  /* creation of BMSHandler */
  BMSHandlerHandle = osThreadNew(BMSHandlerTask, NULL, &BMSHandler_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Mode_Button_Event */
  Mode_Button_EventHandle = osEventFlagsNew(&Mode_Button_Event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*  read_data=HX710B_Read_Sensor(&HX710B_config);
		  HAL_Delay(1000);
	    */
		


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STAT_PULL_UP_GPIO_Port, STAT_PULL_UP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AIR_PUMP_Pin|Buzzer_Pin|Sensor_Clock_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Set_Mode_Pin */
  GPIO_InitStruct.Pin = Set_Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Set_Mode_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Count_UP_Pin Count_Down_Pin Sensor_Out_Pin */
  GPIO_InitStruct.Pin = Count_UP_Pin|Count_Down_Pin|Sensor_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STAT_PULL_UP_Pin */
  GPIO_InitStruct.Pin = STAT_PULL_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STAT_PULL_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STAT_Pin */
  GPIO_InitStruct.Pin = STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AIR_PUMP_Pin Buzzer_Pin Sensor_Clock_Pin */
  GPIO_InitStruct.Pin = AIR_PUMP_Pin|Buzzer_Pin|Sensor_Clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	
	osThreadFlagsSet(ButtonHandlerHandle,MODE_BUTTON_PRESSED_FLAG);
	///////////////////////////////////////////
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_mmhgHandlerTask */
/**
  * @brief  Function implementing the mmhgHandler thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mmhgHandlerTask */

float temp;
void mmhgHandlerTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	/*
	uint32_t temp_var = 200;
	HAL_ADCEx_Calibration_Start(&hadc1);
	uint8_t sumcount=0;
	uint32_t adc_sum;
	*/
  for(;;)
  {
		
		
	
  /* 
  	HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		sumcount++;
		adc_sum+=HAL_ADC_GetValue(&hadc1);
		if(sumcount>10) {
		adc_sum/=sumcount;
		temp = (3.3 *adc_sum) / 4096.0  ;
		adc_sum=0;
		sumcount=0;
		}
 
		*/
	
		
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DisplayTask */
/**
* @brief Function implementing the Display thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DisplayTask */
void DisplayTask(void *argument)
{
  /* USER CODE BEGIN DisplayTask */
  /* Infinite loop */
	display_init();
	osDelay(1000);
	uint32_t recievedata=0;
	
  for(;;)
  {
		
		/*
		osDelay(1000);
		set_preasure((uint32_t)(temp*1000));*/
		
		
		

   if(osMessageQueueGet (DisplayQueueHandle, (void*) &recievedata, NULL, osWaitForever) == osOK) {
		 osSemaphoreAcquire(I2CSemaphoreHandle,osWaitForever);
		 // Checking address of threads to identify who send it 
		 
		 if(recievedata & MASK_FROM_BUTTON_TASK ) {
			 
			 if(recievedata & MASK_FOR_SYSTEM_ON){
					set_preasure(recievedata & MASK_FOR_CLEARING_COMUNICATION_FLAGS);
					display_mode((display_mode_t)3);
			 }else{
					set_preasure(recievedata & MASK_FOR_CLEARING_COMUNICATION_FLAGS);
					display_mode((display_mode_t)2);
				 
			 }
			 
		 } else if ( recievedata & MASK_FROM_SYSTEM_TASK ) {
			 set_preasure(recievedata & MASK_FOR_CLEARING_COMUNICATION_FLAGS);
		 } else{ 
			 
			 if(recievedata & MASK_FOR_CHARGING_ON){
					set_bms_soc(0,(charging_status_t)0);
				}else{
					set_bms_soc((uint8_t)recievedata,(charging_status_t)1);
				}
	
		 }
		  osSemaphoreRelease(I2CSemaphoreHandle);
		 
		}
	
	 
		
	 /*
	 last measure i had 142 byte free space;
	 stackspace=osThreadGetStackSpace(DisplayHandle);
	 osDelay(1); */
		
  }
  /* USER CODE END DisplayTask */
}

/* USER CODE BEGIN Header_ButtonsHandlerTask */
/**
* @brief Function implementing the ButtonHandler thread.
* @param argument: Not used
* @retval None

*/


uint32_t stackspace;
/* USER CODE END Header_ButtonsHandlerTask */
void ButtonsHandlerTask(void *argument)
{
  /* USER CODE BEGIN ButtonsHandlerTask */
  /* Infinite loop */
	
	
	uint32_t Set_Value=INITIAL_MMHG_SET_VALUE ;
	uint32_t Prev_Set_Value=INITIAL_MMHG_SET_VALUE ;
	Set_Value= (Set_Value & MASK_FOR_SYSTEM_OFF) | MASK_FROM_BUTTON_TASK ;
	uint8_t npwt_device_state=NPWT_SYSTEM_OFF;
	GPIO_PinState Count_Up_Pin_State;
	GPIO_PinState Count_Down_Pin_State;
	uint8_t speed_up_value=1;
	uint8_t continue_pressed_counter=1;
	
	osMessageQueuePut(DisplayQueueHandle,&Set_Value,1,osWaitForever);
	
  for(;;)
  {

  osDelay(1000);
	
	if(npwt_device_state == NPWT_SYSTEM_ON) {
		osThreadFlagsWait(MODE_BUTTON_PRESSED_FLAG,osFlagsWaitAny,osWaitForever);
		npwt_device_state=NPWT_SYSTEM_OFF;
		Set_Value = (Set_Value & MASK_FOR_SYSTEM_OFF) | MASK_FROM_BUTTON_TASK ;
		osMessageQueuePut(DisplayQueueHandle,&Set_Value,1,osWaitForever);
	} else{
		
		if(osThreadFlagsGet()==MODE_BUTTON_PRESSED_FLAG) {
			osThreadFlagsClear(MODE_BUTTON_PRESSED_FLAG);
			npwt_device_state=NPWT_SYSTEM_ON;
			// here should sent the set value to system handler task

			Set_Value |= MASK_FOR_SYSTEM_ON | MASK_FROM_BUTTON_TASK ;
			osMessageQueuePut(DisplayQueueHandle,&Set_Value,1,osWaitForever);
		} else{
	
			Count_Down_Pin_State=HAL_GPIO_ReadPin(Count_Down_GPIO_Port,Count_Down_Pin);
			Count_Up_Pin_State=HAL_GPIO_ReadPin(Count_UP_GPIO_Port,Count_UP_Pin);
			
			
			
	    if(Count_Down_Pin_State == GPIO_PIN_RESET ) Set_Value -=speed_up_value;
			if(Count_Up_Pin_State == GPIO_PIN_RESET ) Set_Value +=speed_up_value;		
		  osDelay(20);
			
			if((Count_Down_Pin_State == GPIO_PIN_RESET ) || (Count_Up_Pin_State == GPIO_PIN_RESET )) {
				continue_pressed_counter++;
				if(continue_pressed_counter>5) speed_up_value = 5;
			} else{
				continue_pressed_counter=0;
				speed_up_value=1;
			} 
			
			
			if((Set_Value & MASK_FOR_CLEARING_COMUNICATION_FLAGS) < SYSTEM_PREASURE_MIN_VALUE )  Set_Value =SYSTEM_PREASURE_MIN_VALUE;
			if((Set_Value & MASK_FOR_CLEARING_COMUNICATION_FLAGS) > SYSTEM_PREASURE_MAX_VALUE )  Set_Value =SYSTEM_PREASURE_MAX_VALUE;
			
			Set_Value = (Set_Value & MASK_FOR_SYSTEM_OFF) | MASK_FROM_BUTTON_TASK ;
			
			
			if(Set_Value !=Prev_Set_Value ){
			Prev_Set_Value=Set_Value;
	  	osMessageQueuePut(DisplayQueueHandle,&Set_Value,1,osWaitForever);
			}
			}
	}
		
		
		stackspace=osThreadGetStackSpace(ButtonHandlerHandle);
	 osDelay(1); 
		
	
  }
  /* USER CODE END ButtonsHandlerTask */
}

/* USER CODE BEGIN Header_BMSHandlerTask */
/**
* @brief Function implementing the BMSHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMSHandlerTask */
void BMSHandlerTask(void *argument)
{
  /* USER CODE BEGIN BMSHandlerTask */
	
	bms_init_t bms_init_status; 
	chargin_state_t temp_charging_state = battery_managment.charging_state;
	uint32_t Send_value = 0;
	

	osSemaphoreAcquire(I2CSemaphoreHandle,osWaitForever);
	bms_init_status=bat_managment_init(&battery_managment);
	osSemaphoreRelease(I2CSemaphoreHandle);
		
  /* Infinite loop */
  for(;;)
  {	
		bms_set_operation_mode(&battery_managment,bms_operation_mode);
	  	display_mode(display_wake); */
		
		osDelay(1000); 
		
	  if(bms_init_status !=bms_init_error){
			
		if(temp_charging_state != battery_managment.charging_state) {
			temp_charging_state = battery_managment.charging_state;
			
			switch((int)battery_managment.charging_state) {
				
				case  bms_charging:
					Send_value |=  MASK_FROM_BMS_TASK  | MASK_FOR_CHARGING_ON; 
				break;
				
				case bms_charging_complete :
					Send_value |=  MASK_FROM_BMS_TASK  | MASK_FOR_CHARGING_ON; 
					
				break;
				
				case bms_charging_removed : 
					Send_value = (Send_value & MASK_FOR_CHARGING_OFF) | MASK_FROM_BMS_TASK; 
		
				break;
					
			}
			Send_value |= battery_managment.current_soc;
			osMessageQueuePut(DisplayQueueHandle,&Send_value,1,osWaitForever);
		}
		
		
		osSemaphoreAcquire(I2CSemaphoreHandle,osWaitForever);
		bat_managment_processloop(&battery_managment);
		osSemaphoreRelease(I2CSemaphoreHandle);
		
	//	osDelay(3000);
	//	bms_set_operation_mode(&battery_managment,bms_sleep_mode);
	//	display_mode(display_sleep);
		
		// turn off freertos scheduler with turning of systic interrupt
	//	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
 //   HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		
    if(battery_managment.op_voltage == below_operational_voltage ) {
			
			// goes system into slee mode.
	
 
		}			
		
	}
		
	
  }
  /* USER CODE END BMSHandlerTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
