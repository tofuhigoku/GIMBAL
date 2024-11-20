#ifndef __DRV8311P_H__
#define __DRV8311P_H__

#include "stm32g4xx_hal.h"

extern SPI_HandleTypeDef hspi2;

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
		
	uint16_t PWM_MAX_PRD_cnt;
} st_drv8311p_t;

void DRV8311P_read_addr(uint8_t addr, uint16_t *p_Data);
void DRV8311P_read_multiple_addr(uint8_t base_addr, uint16_t *p_Data, size_t num_of_reg);
void DRV8311P_read_All_reg(st_drv8311p_t* p_drv8311p_address, st_drv8311p_t* p_drv8311p_reg_data);

void DRV8311P_write_addr(uint8_t addr, uint16_t *p_Data);

void DRV8311P_Init(st_drv8311p_t* p_drv8311p_address, st_drv8311p_t* p_drv8311p_reg_data);

void DRV8311P_PWM_GEN(st_drv8311p_t* p_drv8311p_address, uint8_t ON_OFF);
void DRV8311P_update_PWM(st_drv8311p_t* p_drv8311p_address, uint16_t dtcA, uint16_t dtcB, uint16_t dtcC);
void DRV8311P_update_Phase_Voltage(st_drv8311p_t* p_drv8311p_address, st_drv8311p_t* p_drv8311p_reg_data, float Ua, float Ub,  float Uc, uint8_t swap_phase_oder );


#endif