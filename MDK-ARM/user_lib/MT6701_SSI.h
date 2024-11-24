/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MT6701_SSI_H
#define __MT6701_SSI_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx.h"
//#include "foc.h"

/* ---------------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* Define --------------------------------------------------------------------*/
#define MT6701_SSI	hspi1
#define MT6701_CS_LOW			{HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);}
#define MT6701_CS_HIGH		{HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);}

static uint8_t tableCRC6[64] = {
 0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
 0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
 0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
 0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
 0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
 0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02
};
#define N_POS_SAMPLES 20		// Number of position samples to store.  should put this somewhere else...
#define N_LUT 128						// lut size, currently constan. Perhaps to be made variable by user?
#define pole_pairs 7

extern float _calibrationLut[N_LUT];
typedef struct
{
  uint8_t rawdata[3];       
	uint16_t raw_14bit_angle;
	uint8_t _4bit_status, _6bit_CRC;
  float raw_angle,shaft_angle;  
	float last_shaft_angle;
  int32_t n_of_revolution;      
	float _angle_multiturn[N_POS_SAMPLES], _turns, _first_sample;
  float velocity, elec_velocity, rpm;     
	float last_velocity; 
	
	float Accel;
	uint8_t lost_magnet;
	
	
	float zero_electric_angle;
	float elec_angle;
// create pointer for lut memory
//	float _calibrationLut[N_LUT];
	int directionSensor;
	float _dt;
} MT6701_sensor;






	// Init inital angles




typedef struct{
	union{
		uint8_t spi_tx_buff[2];
		uint16_t spi_tx_word;
	};
	union{
		uint8_t spi_rx_buff[2];
		uint16_t spi_rx_word;
	};
	float angle_singleturn, old_angle, angle_multiturn[N_POS_SAMPLES], elec_angle, velocity, elec_velocity, ppairs, vel2;
	float output_angle_multiturn;
	int raw, count, old_count, turns;
	int count_buff[N_POS_SAMPLES];
	int m_zero, e_zero;
	int offset_lut[N_LUT];
	uint8_t first_sample;
} EncoderStruct;



//void ps_warmup(EncoderStruct * encoder, int n);
//void ps_sample(EncoderStruct * encoder, float dt);
//void ps_print(EncoderStruct * encoder, int dt_ms);
/* ---------------------------------------------------------------------------*/

/* Functions ------------------------------------------------------------------*/
uint8_t MT6701_read_data(MT6701_sensor * encoder);
float _normalizeAngle(float angle);
float electricalAngle(float MechanicalAngle, MT6701_sensor * encoder);
static uint8_t CRC6_43_18bit (uint32_t w_InputData);

void position_sensor_warmup(MT6701_sensor * encoder, int n);
void position_sensor_sample(MT6701_sensor * encoder, float dt);
float get_Calibrated_Angle(float __raw_angle);
/* ----------------------------------------------------------------------------*/




#ifdef __cplusplus
}
#endif

#endif   /*__MT6701_SSI_H*/