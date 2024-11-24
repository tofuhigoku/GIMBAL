#ifndef __OWOC_H
#define __OWOC_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "MT6701_SSI.h"

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


#define TIM_PWM 			htim1
#define ADC_CH_MAIN		hadc1				// ADC channel handle which drives simultaneous mode
#define ADC_CH_IA			hadc1					// Phase A current sense ADC channel handle.  0 = unused
#define ADC_CH_IB			hadc1				// Phase B current sense ADC channel handle.  0 = unused
#define ADC_CH_IC			hadc1				// Phase C current sense ADC channel handle.  0 = unused
#define ADC_CH_VBUS		hadc2				// Bus voltage ADC channel handle.  0 = unused

#define INLA	GPIO_PIN_13
#define INLB	GPIO_PIN_14
#define INLC	GPIO_PIN_15
#define DR_ENABLE	GPIO_PIN_6

#define CURRENT_FILT_ALPHA 0.05f
typedef struct{
	uint32_t tim_ch_w;								// Terminal W timer channel
    int adc_a_raw, adc_b_raw, adc_c_raw, adc_vbus_raw;      // Raw ADC Values
    float i_a, i_b, i_c;                                    // Phase currents
		float CS_GAIN, R_shunt, current_gain;
    float v_bus, v_bus_filt;                                // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // Rotor mechanical and electrical angular velocit
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q currents
    float i_mag;																						// Current magnitude
    float v_d, v_q;                                         // D/Q voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float i_scale, k_d, k_q, ki_d, ki_q, ki_fw, alpha;               // Current loop gains, current reference filter coefficient
    float d_int, q_int;                                     // Current error integrals
    int adc_a_offset, adc_b_offset, adc_c_offset, adc_vbus_offset; 		// ADC offsets
    float i_d_des, i_q_des, i_d_des_filt, i_q_des_filt;     // Current references
		
		float R_phase, L_phase;
		float KV,K_bemf,KT,GR;
//    int loop_count;                                         // Degubbing counter
//    int timeout;                                            // Watchdog counter
//    int mode;
//    int ovp_flag;                                           // Over-voltage flag
    int oc_flag;											// Over-current flag
    int phase_order;
		
		float _dt_;
		float p_des, v_des, kp, kd, t_ff;                   // Desired position, velocity, gains, torque
		//    union{
		//    	float commands[5];									// Making this easier to pass around without including foc.h everywhere
		//    	struct{
		//    		float p_des, v_des, kp, kd, t_ff;                   // Desired position, velocity, gains, torque
		//    	};
		//    };
    float v_calib, v_max, v_ref, fw_int;                                    // output voltage magnitude, field-weakening integral
//    int otw_flag;                                           // Over-temp warning
    float i_max;											// Maximum current
//    float inverter_tab[128];								// Inverter linearization table
//    uint8_t invert_dtc;										// Inverter duty cycle inverting/non-inverting
    } Motor_struct;

void driver_enable();
void driver_disable();		
void BLDC_param_init(Motor_struct* MOTOR, float VBUS, float I_max, float Current_sensor_gain, float Rshunt_ , float _R_phase, float _L_phase, float KV, float GEAR_RATIO );	
void set_dtc(Motor_struct* MOTOR);
void setPhaseVoltage(float Uq, float Ud, float angle_el,	Motor_struct* MOTOR );		
uint8_t phase_order(Motor_struct *MOTOR, MT6701_sensor * encoder, float calib_voltage);
void sensor_calibrate(Motor_struct *MOTOR, MT6701_sensor * encoder, float calib_voltage);		
void get_current(Motor_struct* MOTOR, uint16_t ADC_iA, uint16_t ADC_iB, uint16_t ADC_iC);
void get_current_offset(Motor_struct* MOTOR, uint16_t ADC_iA_offset, uint16_t ADC_iB_offset, uint16_t ADC_iC_offset);
void get_DQ_current(Motor_struct* MOTOR, float electric_angle);

void torque_control(Motor_struct* MOTOR);		
void commutate(Motor_struct* MOTOR, MT6701_sensor * encoder );
		
#ifdef __cplusplus
}
#endif

#endif   /*__MT6701_SSI_H*/

