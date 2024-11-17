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
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define	DEV_STS1_offset			0
	
#define	OT_STS_offset				0x4
#define SUP_STS_offset			0x5
#define DRV_STS_offset			0x6
#define SYS_STS_offset			0x7

#define PWM_SYNC_PRD_offset	0xC
#define FLT_MODE_offset			0x10
#define SYSF_CTRL_offset		0x12
#define DRVF_CTRL_offset		0x13
#define FLT_TCTRL_offset		0x16
#define FLT_CLR_offset			0x17
#define PWMG_PERIOD_offset	0x18
#define PWMG_A_DUTY_offset	0x19
#define PWMG_B_DUTY_offset	0x1A
#define PWMG_C_DUTY_offset	0X1B
#define PWM_STATE_offset		0X1C
#define PWMG_CTRL_offset		0x1D
#define PWM_CTRL1_offset		0x20
#define DRV_CTRL_offset			0x22
#define CSA_CTRL_offset			0x23
#define SYS_CTRL_offset			0x3F

typedef struct st_drv8311p
{
	uint16_t	DEV_STS1;
	
	uint16_t	OT_STS;
	uint16_t	SUP_STS;
	uint16_t	DRV_STS;
	uint16_t	SYS_STS;
	
	uint16_t	PWM_SYNC_PRD;
	uint16_t	FLT_MODE;
	uint16_t	SYSF_CTRL;
	uint16_t	DRVF_CTRL;
	uint16_t	FLT_TCTRL;
	uint16_t	FLT_CLR;
	uint16_t	PWMG_PERIOD;
	uint16_t	PWMG_A_DUTY;
	uint16_t	PWMG_B_DUTY;
	uint16_t	PWMG_C_DUTY;
	uint16_t	PWM_STATE;
	uint16_t	PWMG_CTRL;
	uint16_t	PWM_CTRL1;
	uint16_t	DRV_CTRL;
	uint16_t	CSA_CTRL;
	uint16_t	SYS_CTRL;
		
	
} st_drv8311p_t;




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
