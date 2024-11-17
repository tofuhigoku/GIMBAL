/*
 * foc.h
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
//#include "MT6701_SSI.h"
//#include "math.h"

//#include "position_sensor.h"
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

#define TIM_PWM 			htim1
#define ADC_CH_MAIN		hadc1				// ADC channel handle which drives simultaneous mode
#define ADC_CH_IA			hadc1					// Phase A current sense ADC channel handle.  0 = unused
#define ADC_CH_IB			hadc1				// Phase B current sense ADC channel handle.  0 = unused
#define ADC_CH_IC			hadc1				// Phase C current sense ADC channel handle.  0 = unused
#define ADC_CH_VBUS		hadc2				// Bus voltage ADC channel handle.  0 = unused


typedef struct{
	uint32_t tim_ch_w;								// Terminal W timer channel
    int adc_a_raw, adc_b_raw, adc_c_raw, adc_vbus_raw;      // Raw ADC Values
    float i_a, i_b, i_c;                                    // Phase currents
    float v_bus, v_bus_filt;                                // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // Rotor mechanical and electrical angular velocit
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q currents
    float i_mag;											// Current magnitude
    float v_d, v_q;                                         // D/Q voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float i_scale, k_d, k_q, ki_d, ki_q, ki_fw, alpha;               // Current loop gains, current reference filter coefficient
    float d_int, q_int;                                     // Current error integrals
    int adc_a_offset, adc_b_offset, adc_c_offset, adc_vbus_offset; 		// ADC offsets
    float i_d_des, i_q_des, i_d_des_filt, i_q_des_filt;     // Current references
    int loop_count;                                         // Degubbing counter
    int timeout;                                            // Watchdog counter
    int mode;
    int ovp_flag;                                           // Over-voltage flag
    int oc_flag;											// Over-current flag
    int phase_order;
    union{
    	float commands[5];									// Making this easier to pass around without including foc.h everywhere
    	struct{
    		float p_des, v_des, kp, kd, t_ff;                   // Desired position, velocity, gains, torque
    	};
    };
    float v_max, v_ref, fw_int;                                    // output voltage magnitude, field-weakening integral
    int otw_flag;                                           // Over-temp warning
    float i_max;											// Maximum current
    float inverter_tab[128];								// Inverter linearization table
    uint8_t invert_dtc;										// Inverter duty cycle inverting/non-inverting
    } ControllerStruct;

typedef struct{
    double temperature;                                     // Estimated temperature
    float temp_measured;									// "Measured" temperature computed from resistance
    float qd_in, qd_out;									// Thermal power in and out
    float resistance;										// Motor resistance
    float k;												// Temperature observer gain
    float trust;											// Temperature observer "trust' (kind of like 1/covariance)
    float delta_t;											// Temperature rise
    }   ObserverStruct;

void set_dtc(ControllerStruct *controller);
void setPhaseVoltage(float Uq, float Ud, float angle_el, ControllerStruct *controller) ;
void analog_sample(ControllerStruct *controller);
void abc(float theta, float d, float q, float *a, float *b, float *c);
void dq0(float theta, float a, float b, float c, float *d, float *q);
void svm(float v_max, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);
void zero_current(ControllerStruct *controller);
void reset_foc(ControllerStruct *controller);
void reset_observer(ObserverStruct *observer);
void init_controller_params(ControllerStruct *controller);
void commutate(ControllerStruct *controller, float elec_angle, float elec_velocity, float velocity, float mech_angle);
void torque_control(ControllerStruct *controller);
void limit_current_ref (ControllerStruct *controller);
void update_observer(ControllerStruct *controller, ObserverStruct *observer);
void field_weaken(ControllerStruct *controller);
float linearize_dtc(ControllerStruct *controller, float dtc);
void zero_commands(ControllerStruct * controller);


#endif /* INC_FOC_H_ */
