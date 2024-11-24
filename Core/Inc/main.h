/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MT6701_SSI.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math_ops.h"
//#include "foc.h"
//#include "lookup.h"
#include "math.h"
#include "OwOc.h"
#include "monitor.h"
#include "DRV8311P.h"

extern st_drv8311p_t drv8311p_address;

uint8_t BLDC_get_phase_order(st_drv8311p_t* p_drv8311p_address, st_drv8311p_t* p_drv8311p_reg_data, Motor_struct *MOTOR, MT6701_sensor * encoder, float calib_voltage);
void Uart_Put_UintNumber(UART_HandleTypeDef* huart, int32_t x, uint8_t EOL_flag);
void Uart_Put_FloatNumber(UART_HandleTypeDef* huart, float y);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */





/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
