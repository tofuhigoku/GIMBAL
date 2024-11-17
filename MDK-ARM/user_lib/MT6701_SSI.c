#include "MT6701_SSI.h"
#include <stdio.h>
#include <string.h>
#include "math_ops.h"
//#include "lookup.h"
//#include "foc.h"

uint8_t MT6701_read_data(MT6701_sensor * encoder){
	MT6701_CS_LOW;
	HAL_SPI_Receive(&MT6701_SSI, (uint8_t *)encoder->rawdata, 3, 10);
	MT6701_CS_HIGH;
	encoder->raw_14bit_angle = ((encoder->rawdata[0]<<8)|(encoder->rawdata[1]))>>2;
	encoder->_4bit_status = ((encoder->rawdata[1]&0x3)<<2)|(encoder->rawdata[2]>>6);
	encoder->_6bit_CRC = encoder->rawdata[2]&0x3f;
	encoder->lost_magnet = (encoder->_4bit_status>>3)?1:0;
	
	if(!encoder->lost_magnet)
	{
		if(encoder->_6bit_CRC == CRC6_43_18bit((encoder->raw_14bit_angle<<4)|encoder->_4bit_status)){
				encoder->raw_angle = encoder->raw_14bit_angle/16384.0f*_2PI;
				return 1;
		}
		else		return 2;
	}
	else
	{
		return 0;
	}
}
static uint8_t CRC6_43_18bit (uint32_t w_InputData)
{
 uint8_t b_Index = 0;
 uint8_t b_CRC = 0;

 b_Index = (uint8_t )(((uint32_t)w_InputData >> 12u) & 0x0000003Fu);

 b_CRC = (uint8_t )(((uint32_t)w_InputData >> 6u) & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = (uint8_t )((uint32_t)w_InputData & 0x0000003Fu);
 b_Index = b_CRC ^ tableCRC6[b_Index];

 b_CRC = tableCRC6[b_Index];

 return b_CRC;
} 
float _normalizeAngle(float angle){
  float a = fmodf(angle, _2PI);
	
  return a >= 0 ? a : (a + _2PI);
}

float electricalAngle(float MechanicalAngle, MT6701_sensor * encoder){
  return  _normalizeAngle( (float)(pole_pairs) *( MechanicalAngle)  - encoder->zero_electric_angle );
}

void position_sensor_warmup(MT6701_sensor * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	for(int i = 0; i<n; i++){
			MT6701_read_data(encoder);
	}
}

float get_Calibrated_Angle(float __raw_angle){
    // raw encoder position e.g. 0-2PI
    float rawAngle = _normalizeAngle(__raw_angle);

    // index of the bucket that rawAngle is part of. 
    // e.g. rawAngle = 0 --> bucketIndex = 0.
    // e.g. rawAngle = 2PI --> bucketIndex = 128.
    int bucketIndex = floor(rawAngle/(_2PI/N_LUT));
    float remainder = rawAngle - ((_2PI/N_LUT)*bucketIndex);

    // Extract the lower and upper LUT value in counts
    float y0 = _calibrationLut[bucketIndex]; 
    float y1 = _calibrationLut[(bucketIndex+1)%N_LUT]; 

    // Linear Interpolation Between LUT values y0 and y1 using the remainder
    // If remainder = 0, interpolated offset = y0
    // If remainder = 2PI/n_lut, interpolated offset = y1
    float interpolatedOffset = (((_2PI/N_LUT)-remainder)/(_2PI/N_LUT))*y0 + (remainder/(_2PI/N_LUT))*y1; 

    // add offset to the raw sensor count. Divide multiply by 2PI/CPR to get radians
    float calibratedAngle = rawAngle+interpolatedOffset; 

    // return calibrated angle in radians
    return _normalizeAngle(calibratedAngle);
}


void position_sensor_sample(MT6701_sensor * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	/* Shift around previous samples */
	encoder->last_shaft_angle = encoder->shaft_angle;
	for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->_angle_multiturn[i] = encoder->_angle_multiturn[i-1];}



	/* Real angles in radians */
	encoder->shaft_angle = get_Calibrated_Angle(encoder->raw_angle);
	encoder->elec_angle = electricalAngle(encoder->shaft_angle, encoder);
//	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
//	int int_angle = encoder->angle_singleturn;
//	encoder->angle_singleturn = TWO_PI_F*(encoder->angle_singleturn - (float)int_angle);
	//encoder->angle_singleturn = TWO_PI_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
//	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;

//	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
//	int_angle = (int)encoder->elec_angle;
//	encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
//	//encoder->elec_angle = TWO_PI_F*fmodf((encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR), 1.0f);
//	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers

	/* Rollover */
	int rollover = 0;
	float angle_diff = encoder->shaft_angle - encoder->last_shaft_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->_turns += rollover;
	if(!encoder->_first_sample){
		encoder->_turns = 0;
		encoder->_first_sample = 1;
	}



	/* Multi-turn position */
	encoder->_angle_multiturn[0] = encoder->shaft_angle + TWO_PI_F*(float)encoder->_turns;

	/* Velocity */
	/*
	// Attempt at a moving least squares.  Wasn't any better
		float m = (float)N_POS_SAMPLES;
		float w = 1.0f/m;
		float q = 12.0f/(m*m*m - m);
		float c1 = 0.0f;
		float ibar = (m - 1.0f)/2.0f;
		for(int i = 0; i<N_POS_SAMPLES; i++){
			c1 += encoder->angle_multiturn[i]*q*(i - ibar);
		}
		encoder->vel2 = -c1/dt;
*/
	//encoder->velocity = vel2
	encoder->velocity = (encoder->_angle_multiturn[0] - encoder->_angle_multiturn[N_POS_SAMPLES-1])/(dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = pole_pairs*encoder->velocity;
	encoder->rpm = 9.5492968f*encoder->velocity;

}

