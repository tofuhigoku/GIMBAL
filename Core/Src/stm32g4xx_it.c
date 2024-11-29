/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint64_t tim1_cnt_it;
extern uint8_t error_code;
extern uint8_t stop, error_mag, error_mag_cnt;

extern MT6701_sensor MT6701;
extern Motor_struct	BLDC;
extern SerialMessage Serial_mgs;
extern st_drv8311p_t drv8311p_reg_data;

extern FDCAN_TxHeaderTypeDef   TxHeader;
extern FDCAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               TxData[20];
extern uint8_t               RxData[20];

extern FDCAN_HandleTypeDef hfdcan1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;



extern uint16_t cmd;
extern uint8_t addr;
extern uint16_t w_data;
extern uint16_t r_data;
extern uint8_t flag;



extern uint16_t ADC1_buffer[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	if(!(HAL_GetTick()%500))
	{
//		DRV8311P_write_n_read();
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	}
//	if((HAL_GetTick()%100 == 0) && (htim1.Instance->DIER & 0x0001))
//	{
//		htim1.Instance->DIER &= ~0x0001;		// enable timer1 Interrupt
//		Vbus_Sampling(&BLDC.v_bus,&BLDC.Vbus_raw, BLDC.Vbus_Gain);
//		htim1.Instance->DIER |=0x0001;		// enable timer1 Interrupt
//	}
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	
	GPIOB->ODR |= (1 << 5);
	
	ADC1_buffer[1] = HAL_ADC_GetValue(&hadc1);
	ADC1_buffer[2] = HAL_ADC_GetValue(&hadc2);
	
	get_current(&BLDC, 0 , ADC1_buffer[1], ADC1_buffer[2]);
	
	error_code = MT6701_read_data(&MT6701);
	error_mag |= MT6701._4bit_status;
	if(MT6701._4bit_status>=0x8)
	{
		error_mag_cnt +=1;
	}
	position_sensor_sample(&MT6701,MT6701._dt);
	
	if(BLDC.op_mode == Motor_No_OP){
		get_DQ_current(&BLDC, MT6701.elec_angle);
		setPhaseVoltage(0, 0, 0, &BLDC);
		DRV8311P_update_Phase_Voltage(&drv8311p_address, &drv8311p_reg_data, BLDC.dtc_u, BLDC.dtc_v, BLDC.dtc_w, 0);
	}
	else if(BLDC.op_mode == Motor_PA_PoleAlign)
	{
		get_DQ_current(&BLDC, MT6701.elec_angle);
		
		Uart_Put_FloatNumber(&huart1, BLDC.i_d);
		
		setPhaseVoltage(0, BLDC.v_calib, _3PI_2*0, &BLDC);
		DRV8311P_update_Phase_Voltage(&drv8311p_address, &drv8311p_reg_data, BLDC.dtc_u, BLDC.dtc_v, BLDC.dtc_w, 0);
		
	}
	else if(BLDC.op_mode == Motor_SimpleTest)
	{
		get_DQ_current(&BLDC, MT6701.elec_angle);
		setPhaseVoltage(BLDC.v_calib,0, MT6701.elec_angle , &BLDC);
		DRV8311P_update_Phase_Voltage(&drv8311p_address, &drv8311p_reg_data, BLDC.dtc_u, BLDC.dtc_v, BLDC.dtc_w, 0);
	}
	else
	{
		commutate(&BLDC, &MT6701);
		DRV8311P_update_Phase_Voltage(&drv8311p_address, &drv8311p_reg_data, BLDC.dtc_u, BLDC.dtc_v, BLDC.dtc_w, 0);
	}
	
	GPIOB->ODR &= ~(1 << 5);
	
	
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

