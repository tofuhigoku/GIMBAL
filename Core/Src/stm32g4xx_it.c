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
//	if(flag==1)
//		{
//			flag = 0;
//			 DRV8311P_read_addr(addr, &r_data);
////			DRV8311P_read_multiple_addr(drv8311p_address.DEV_STS1, (uint16_t*) &drv8311p_reg_data, 21);
//			DRV8311P_read_All_reg(&drv8311p_address, &drv8311p_reg_data);

//			
//		}
//	else if(flag == 2)
//	{
//		flag = 0;
//		DRV8311P_write_addr(addr, &w_data);
//	}
//	else if (flag == 3)
//	{
//		flag =0;
//		DRV8311P_Init(&drv8311p_address, &drv8311p_reg_data);
//		
//	}
//	else if (flag == 4)
//	{
//		flag = 0; 
//		DRV8311P_PWM_GEN(&drv8311p_address, 1);
//	}
//	else if (flag == 5)
//	{
//		flag = 0; 
//		DRV8311P_PWM_GEN(&drv8311p_address, 0);
//	}
//	else if (flag == 6)
//	{
//		flag = 0;
//		DRV8311P_update_PWM(&drv8311p_address,  drv8311p_reg_data.PWMG_A_DUTY, drv8311p_reg_data.PWMG_B_DUTY, drv8311p_reg_data.PWMG_C_DUTY);
//	}
//	else if (flag == 7)
//	{
//		flag = 0;
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
//	}
//	else if (flag == 8)
//	{
//		flag = 0;
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
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
	ADC1_buffer[1] = HAL_ADC_GetValue(&hadc1);
	ADC1_buffer[2] = HAL_ADC_GetValue(&hadc2);
	GPIOB->ODR |= (1 << 5);
	
	HAL_ADC_Start(&hadc1);
//	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc1, 100);
	
	error_code = MT6701_read_data(&MT6701);
	error_mag |= MT6701._4bit_status;
	if(MT6701._4bit_status>=0x8)
	{
		error_mag_cnt +=1;
	}
	position_sensor_sample(&MT6701,0.0002 + 0.00005*0);
	
	if(stop==0){
		setPhaseVoltage(0, 0, 0, &BLDC);
		DRV8311P_update_Phase_Voltage(&drv8311p_address, &drv8311p_reg_data, BLDC.dtc_u, BLDC.dtc_v, BLDC.dtc_w, BLDC.phase_order);
		get_DQ_current(&BLDC, MT6701.elec_angle);
	}
	else if(stop ==0x1)
	{
		setPhaseVoltage(0, BLDC.v_calib, _3PI_2*0, &BLDC);
		DRV8311P_update_Phase_Voltage(&drv8311p_address, &drv8311p_reg_data, BLDC.dtc_u, BLDC.dtc_v, BLDC.dtc_w, BLDC.phase_order);
		get_DQ_current(&BLDC, MT6701.elec_angle);
//		UU_put_float(USART1, BLDC.i_d, 4);
//		UU_PutChar(USART1, '\n');
	}
	else if(stop == 0x2)
	{
		setPhaseVoltage(BLDC.v_calib, 0, MT6701.elec_angle, &BLDC);
		DRV8311P_update_Phase_Voltage(&drv8311p_address, &drv8311p_reg_data, BLDC.dtc_u, BLDC.dtc_v, BLDC.dtc_w, BLDC.phase_order);
		get_DQ_current(&BLDC, MT6701.elec_angle);
	}
	
	GPIOB->ODR &= ~(1 << 5);
	
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

