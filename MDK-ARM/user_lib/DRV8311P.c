#include "DRV8311P.h"

st_drv8311p_t drv8311p_address = {
		DEV_STS1_offset,
		OT_STS_offset,				
		SUP_STS_offset,			
		DRV_STS_offset,		
		SYS_STS_offset,		

		PWM_SYNC_PRD_offset,	
		FLT_MODE_offset,
		SYSF_CTRL_offset,		
		DRVF_CTRL_offset,	
		FLT_TCTRL_offset,	
		FLT_CLR_offset,	
		PWMG_PERIOD_offset,	
		PWMG_A_DUTY_offset,
		PWMG_B_DUTY_offset,
		PWMG_C_DUTY_offset,
		PWM_STATE_offset,
		PWMG_CTRL_offset,	
		PWM_CTRL1_offset,	
		DRV_CTRL_offset,	
		CSA_CTRL_offset,		
		SYS_CTRL_offset,		
};

void DRV8311P_read_addr(uint8_t addr, uint16_t *p_Data)
{
	uint16_t cmd = 0x8000;
	cmd = cmd | ( addr << 3 );
	uint8_t write_buffer[4] = {cmd >> 8, cmd & 0x00ff, 0, 0};
	uint8_t read_buffer[4] = {0};
	

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0); 
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &write_buffer, (uint8_t*) &read_buffer, 4 , HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1); 
	*p_Data = (read_buffer[2] << 8 )| read_buffer[3];
}

void DRV8311P_read_multiple_addr(uint8_t base_addr, uint16_t *p_Data, size_t num_of_reg)
{
	uint16_t cmd = 0x8000;
	cmd = cmd | ( base_addr << 3 );
	uint8_t write_buffer[50] = {cmd >> 8, cmd & 0x00ff, 0, 0};
	uint8_t read_buffer[50] = {0};
	

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0); 
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &write_buffer, (uint8_t*) &read_buffer, num_of_reg*sizeof(uint16_t) , HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1); 
	
	for(size_t i = 0; i < num_of_reg; i++)
	{
		*(p_Data + i) = (read_buffer[2 + 2*i] << 8) | read_buffer[2 + 2*i + 1];
		
	}
//	memcpy(p_Data, &read_buffer[2], num_of_reg*sizeof(uint16_t));
//	for(size_t i = 0; i < num_of_reg; i++)
//	{
//		*(p_Data+i) = DRV8311P_read_addr(
//		
//	}
}

void DRV8311P_read_All_reg(st_drv8311p_t* p_drv8311p_address, st_drv8311p_t* p_drv8311p_reg_data)
{
	uint16_t* p_base_addr = (uint16_t*)&p_drv8311p_address->DEV_STS1;
	uint16_t* p_data = (uint16_t*) p_drv8311p_reg_data;
	for(size_t i = 0; i < sizeof(st_drv8311p_t)/sizeof(uint16_t); i++)
	{
		DRV8311P_read_addr(*(uint8_t*)(p_base_addr + i), (uint16_t*)(p_data + i));
		
	}
	p_drv8311p_reg_data->PWM_MAX_PRD_cnt = p_drv8311p_reg_data->PWM_SYNC_PRD/2;
}


void DRV8311P_write_addr(uint8_t addr, uint16_t *p_Data)
{
	uint16_t cmd = 0x0000;
	cmd = cmd | ( addr << 3 );
	uint8_t write_buffer[4] = {cmd >> 8, cmd & 0x00ff, *p_Data>>8, (*p_Data)&0xff};
	uint8_t read_buffer[4] = {0};
	

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0); 
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &write_buffer, (uint8_t*) &read_buffer, 4 , HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1); 
//	*p_Data = (read_buffer[2] << 8 )| read_buffer[3];
}

void DRV8311P_Init(st_drv8311p_t* p_drv8311p_address, st_drv8311p_t* p_drv8311p_reg_data)
{
	DRV8311P_read_All_reg(p_drv8311p_address, p_drv8311p_reg_data);

	uint16_t writedata = 0x31;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->DRVF_CTRL, &writedata);	// set OCP current level is 5A
	
	writedata = 0x03;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->FLT_TCTRL, &writedata);	// set SLOW_RETRY = 0.5s, FAST_RETRY = 5ms
	
//	writedata = 1000;
//	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWMG_PERIOD, &writedata);	// set PWMG_PERIOD = 1000
	
	writedata = 0;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWMG_A_DUTY, &writedata);	// set PWMG_A_DUTY = 0
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWMG_B_DUTY, &writedata);	// set PWMG_B_DUTY = 0
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWMG_C_DUTY, &writedata);	// set PWMG_C_DUTY = 0
	
	writedata = 0x05D;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWMG_CTRL, &writedata);	// 
	
	writedata = 0x007;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWM_CTRL1, &writedata);	// 
	
	writedata = 0x11;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->DRV_CTRL, &writedata);	// set TDEAD_CTRL = 200ns, SLEW_RATE = 75 V/µs
	
	writedata = 0x08;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->CSA_CTRL, &writedata);	// set CSA_EN = 1,  CSA_GAIN = 0.25V/A


	DRV8311P_read_All_reg(p_drv8311p_address, p_drv8311p_reg_data);
//	p_drv8311p_reg_data->PWM_MAX_PRD_cnt = p_drv8311p_reg_data->PWM_SYNC_PRD/2;
}
void DRV8311P_PWM_GEN(st_drv8311p_t* p_drv8311p_address, uint8_t ON_OFF)
{
	ON_OFF = (ON_OFF>0)?1:0;
	uint16_t buffer = 0x0;
	DRV8311P_read_addr((uint8_t) p_drv8311p_address->PWMG_CTRL, &buffer);
	buffer = (ON_OFF>0)?(buffer|(1 << 10)):(buffer & (~(1 << 10)));
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWMG_CTRL, &buffer);
}

void DRV8311P_update_PWM(st_drv8311p_t* p_drv8311p_address, uint16_t dtcA, uint16_t dtcB, uint16_t dtcC)
{

	uint16_t writedata = dtcA;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWMG_A_DUTY, &writedata);	// set PWMG_A_DUTY
	
	writedata = dtcB;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWMG_B_DUTY, &writedata);	// set PWMG_B_DUTY
	
	writedata = dtcC;
	DRV8311P_write_addr((uint8_t) p_drv8311p_address->PWMG_C_DUTY, &writedata);	// set PWMG_C_DUTY
}

void DRV8311P_update_Phase_Voltage(st_drv8311p_t* p_drv8311p_address, st_drv8311p_t* p_drv8311p_reg_data, float Ua, float Ub,  float Uc, uint8_t swap_phase_oder )
{

		DRV8311P_update_PWM(p_drv8311p_address, (p_drv8311p_reg_data->PWM_MAX_PRD_cnt*Ua),
																						(p_drv8311p_reg_data->PWM_MAX_PRD_cnt*Ub),
																						(p_drv8311p_reg_data->PWM_MAX_PRD_cnt*Uc));

}