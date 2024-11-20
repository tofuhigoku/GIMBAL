#include "stm32g4xx_hal.h"
#include "OwOc.h"
#include <stdio.h>
#include <string.h>
#include "math_ops.h"
//#include "lookup.h"
#include "math.h"


void BLDC_param_init(Motor_struct* MOTOR, float VBUS, float I_max, float Current_sensor_gain, float Rshunt_ , float _R_phase, float _L_phase, float KV, float GEAR_RATIO ){
	MOTOR->v_bus = VBUS;
	
	MOTOR->dtc_u = 0;
	MOTOR->dtc_v = 0;
	MOTOR->dtc_w = 0;
	
	MOTOR->v_u = 0;
	MOTOR->v_v = 0;
	MOTOR->v_w = 0;
	
	MOTOR->phase_order = 0;
	MOTOR->v_max = VBUS;
	MOTOR->v_calib = 0.1*VBUS;
	
	MOTOR->adc_a_offset = 0;
	MOTOR->adc_b_offset = 0;
	MOTOR->adc_c_offset = 0;
	
	MOTOR->i_max = I_max;
	
	MOTOR->d_int = 0;
	MOTOR->q_int = 0;
	MOTOR->i_q_des = 0;
	MOTOR->i_d_des = 0;
	
	MOTOR->p_des = 0;
	MOTOR->v_des = 0;
	MOTOR->kp = 0;
	MOTOR->kd = 0;
	MOTOR->t_ff = 0;
	
	MOTOR->KV = KV;													// 			rpm/V
	MOTOR->K_bemf =  1.2f  ; //			V/rad/s
	MOTOR->KT  = 1.0f/(MOTOR->KV*0.1047198);											// Nm/A
	MOTOR->GR = GEAR_RATIO;
	
	MOTOR->R_phase = _R_phase;
	MOTOR->L_phase = _L_phase;
	
	MOTOR->CS_GAIN = Current_sensor_gain;
	MOTOR->R_shunt = Rshunt_;
	MOTOR->current_gain =(3.3f/4096.0f)/(MOTOR->CS_GAIN * MOTOR->R_shunt);
}
void driver_enable(){
//	HAL_GPIO_WritePin(GPIOB, DR_ENABLE|INLA|INLB|INLC, 1);
	HAL_GPIO_WritePin(GPIOC, DR_ENABLE, 1);
	
}
void driver_disable(){
	
//	HAL_GPIO_WritePin(GPIOB, DR_ENABLE|INLA|INLB|INLC, 0);
	HAL_GPIO_WritePin(GPIOC, DR_ENABLE, 0);
	
}
void set_dtc(Motor_struct* MOTOR){

//		uint16_t MAX_CNT__ = htim1.Instance->ARR;
	
		float dtc_u = 1.0f - MOTOR->dtc_u;
		float dtc_v = 1.0f - MOTOR->dtc_v;
		float dtc_w = 1.0f - MOTOR->dtc_w;

	/* Handle phase order swapping so that voltage/current/torque match encoder direction */
	if(!MOTOR->phase_order){ 
		
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((htim1.Instance->ARR))*dtc_u);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ((htim1.Instance->ARR))*dtc_v);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ((htim1.Instance->ARR))*dtc_w);
	}
	else{					// dao thu tu pha
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ((htim1.Instance->ARR))*dtc_u);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((htim1.Instance->ARR))*dtc_v);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ((htim1.Instance->ARR))*dtc_w);
	}
}

void setPhaseVoltage(float _Uq, float _Ud, float angle_el,	Motor_struct* MOTOR ) {
	// Uq, Ud = 0 -> 1
	float Uq = _Uq/MOTOR->v_max;
	float Ud = _Ud/MOTOR->v_max;
	float center, Ua, Ub, Uc;
  float _ca,_sa;

		angle_el = _normalizeAngle(angle_el);
		_sa = sin_lut(angle_el);
		_ca = cos_lut(angle_el);
    float Ualpha =  _ca * Ud - _sa * Uq;
		float Ubeta  =  _sa * Ud + _ca * Uq;
		
		center = 0.5f;
		// invert Clarke transform
		Ua = Ualpha + center;
		Ub = -0.5f * Ualpha  + SQRT3_2 * Ubeta + center;
		Uc = -0.5f * Ualpha  - SQRT3_2 * Ubeta + center;
		
		Ua = fminf(fmaxf(Ua, 0), 1);
		Ub = fminf(fmaxf(Ub, 0), 1);
		Uc = fminf(fmaxf(Uc, 0), 1);
		
		MOTOR->dtc_u = Ua;
		MOTOR->dtc_v = Ub;
		MOTOR->dtc_w = Uc;

//		set_dtc(MOTOR);
}

uint8_t phase_order(Motor_struct *MOTOR, MT6701_sensor * encoder, float calib_voltage)
{
	uint8_t swap_phase_order = 0;
		    // find natural direction (this is copy of the init code)
    // move one electrical revolution forward
    for (int i = 0; i <=500; i++ ) {
      float angle = 0 + _2PI * i / 500.0f;
      setPhaseVoltage(0, calib_voltage,  angle, MOTOR);
//		set_dtc();
			MT6701_read_data(encoder);
//        _wrapped.update();
      HAL_Delay(2);
    }
    // take and angle in the middle
//    _wrapped.update();
		MT6701_read_data(encoder);
    float mid_angle = encoder->raw_angle;
    // move one electrical revolution backwards
    for (int i = 500; i >=0; i-- ) {
      float angle = 0 + _2PI * i / 500.0f ;
      setPhaseVoltage(0, calib_voltage,  angle, MOTOR);
//		set_dtc();			
			MT6701_read_data(encoder);
//        _wrapped.update();
      HAL_Delay(2);
    }
//    _wrapped.update();
		MT6701_read_data(encoder);
    float end_angle = encoder->raw_angle;
    setPhaseVoltage(0, 0,  0, MOTOR);
//		set_dtc();
    HAL_Delay(200);
    // determine the direction the sensor moved
    int directionSensor;
    if (mid_angle < end_angle) {
//      Serial.println("MOT: sensor_direction==CW");
      directionSensor = -1;
//      motor.sensor_direction = Direction::CW;

    } 
		else{
//      Serial.println("MOT: sensor_direction==CCW");
      directionSensor = 1;
//      motor.sensor_direction = Direction::CCW;

    }
//		encoder->directionSensor = directionSensor;
		swap_phase_order = directionSensor>0?0:1;
	return swap_phase_order;
}


									//void sensor_calibrate(Motor_struct *MOTOR, MT6701_sensor * encoder, float calib_voltage)
									//{
									//	float _theta_actual = 0.0;
									//	float _elecAngle = 0.0;
									//	float _elec_angle = 0.0;
									//	float _theta_absolute_post = 0.0;
									//	float _theta_absolute_init = 0.0;
									//	float _theta_init = 0.0;
									//	float _avg_elec_angle = 0.0;
									//	
									//	int _NPP = pole_pairs;																					// number of pole pairs which is user input
									//	const int n_ticks = 128*_NPP;                                   // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
									//	const int n2_ticks = 40;                                        // increments between saved samples (for smoothing motion)
									//	float deltaElectricalAngle = _2PI*_NPP/(n_ticks*n2_ticks);      // Electrical Angle increments for calibration steps
									//	
									//	float error_f[n_ticks];                        // pointer to error array rotating forwards
									//	float error_b[n_ticks];                        // pointer to error array rotating forwards
									//	
									//	float error[n_ticks];                            // pointer to error array (average of forward & backward)
									//	float error_filt[n_ticks];                       // pointer to filtered error array (low pass filter)
									//	const int window = 128;                                         // window size for moving average filter of raw error
									//	encoder->zero_electric_angle = 0;                                  // Set position sensor offset
									//	
									//		setPhaseVoltage(0, calib_voltage,  _elec_angle, MOTOR);
									//    HAL_Delay(1000);
									////    _wrapped.update();
									//		MT6701_read_data(encoder);
									//    float theta_init = encoder->raw_angle;
									//    float theta_absolute_init = encoder->raw_angle;
									//		
									//		/* 
									//    Start Calibration
									//    Loop over  electrical angles from 0 to NPP*2PI, once forward, once backward
									//    store actual position and error as compared to electrical angle
									//    */
									//		
									//		/* 
									//				forwards rotation
									//				*/
									////				Serial.println("Rotating forwards");
									//				int k = 0;
									//				for(int i = 0; i<n_ticks; i++)
									//				{                                                 
									//					for(int j = 0; j<n2_ticks; j++)
									//						{   
									//							_elec_angle += deltaElectricalAngle;
									//							setPhaseVoltage(0, calib_voltage,  _elec_angle, MOTOR);
									//						}
									//					
									//					// delay to settle in position before taking a position sample
									//					HAL_Delay(20);
									////					_wrapped.update();
									//					MT6701_read_data(encoder);
									//					_theta_actual = encoder->raw_angle -theta_init;
									//					if (encoder->directionSensor == -1)
									//					{
									//						_theta_actual = - _theta_actual;
									//						error_f[i] = _theta_actual - _elec_angle/_NPP;

									//					}
									//					else
									//					{
									//						error_f[i] = _elec_angle/_NPP - _theta_actual;
									//					}
									//					// if overflow happened track it as full rotation
									//			//		raw_f[i] = theta_actual;

									//					// storing the normalized angle every time the electrical angle 3PI/2 to calculate average zero electrical angle
									//					if(i==(k*128+96))
									//						{
									//							HAL_Delay(50);
									//							_avg_elec_angle += _normalizeAngle(encoder->raw_angle*_NPP);
									//							k += 1;
									//						}
									//				}
									//				
									//				HAL_Delay(2000);
									//				
									//				
									//				/* 
									//				backwards rotation
									//				*/
									////				Serial.println("Rotating backwards");
									//				for(int i = 0; i<n_ticks; i++)
									//				{                                                 
									//					for(int j = 0; j<n2_ticks; j++)
									//						{   
									//							_elec_angle -= deltaElectricalAngle;
									//							setPhaseVoltage(0, calib_voltage,  _elec_angle, MOTOR);
									////							motor.setPhaseVoltage(voltage_calibration, 0 ,elec_angle);
									//						}

									//						// delay to settle in position before taking a position sample
									//						HAL_Delay(20);
									////						_wrapped.update();
									//						MT6701_read_data(encoder);
									//						_theta_actual = encoder->raw_angle-theta_init;
									//						if (encoder->directionSensor == -1)
									//						{
									//						_theta_actual = -_theta_actual;
									//						error_b[i] =  _theta_actual - _elec_angle/_NPP;
									//						}
									//						else
									//						{
									//						error_b[i] = _elec_angle/_NPP - _theta_actual;
									//						}
									//			//			raw_b[i] = theta_actual;
									//				}
									//				
									//				
									//					// get post calibration mechanical angle.
									////					_wrapped.update();
									//					MT6701_read_data(encoder);
									//					_theta_absolute_post = encoder->raw_angle;

									//					// done with the measurement
									//					setPhaseVoltage(0, 0,  0, MOTOR);
									//				
									//					// raw offset from initial position in absolute radians between 0-2PI
									//					float raw_offset = (_theta_absolute_init + _theta_absolute_post)*0.5;
									//				
									//				
									//					// calculating the average zero electrica angle from the forward calibration.
									//					encoder->zero_electric_angle  = _avg_elec_angle/_NPP;
									////					Serial.print( "Average Zero Electrical Angle: ");
									////					Serial.println( motor.zero_electric_angle); 
									//				
									//				
									//				
									//				// Perform filtering to linearize position sensor eccentricity
									//				// FIR n-sample average, where n = number of samples in one electrical cycle
									//				// This filter has zero gain at electrical frequency and all integer multiples
									//				// So cogging effects should be completely filtered out
									//				float mean = 0;
									//				for (int i = 0; i<n_ticks; i++){                                            //Average the forward and back directions
									//					error[i] = 0.5f*(error_f[i] + error_b[n_ticks-i-1]);
									//					}
									//				for (int i = 0; i<n_ticks; i++){
									//					for(int j = 0; j<window; j++){
									//						int ind = -window/2 + j + i;                                    	// Indexes from -window/2 to + window/2
									//						if(ind<0){
									//							ind += n_ticks;}                                                // Moving average wraps around
									//						else if(ind > n_ticks-1) {
									//							ind -= n_ticks;}
									//						error_filt[i] += error[ind]/(float)window;
									//					}
									//					mean += error_filt[i]/n_ticks;
									//				}
									//				
									//				// calculate offset index
									//				int index_offset = floor(raw_offset/(_2PI/N_LUT));
									//				
									//				
									//				// Build Look Up Table
									//				for (int i = 0; i<N_LUT; i++){                                          
									//					int ind =  index_offset + i*encoder->directionSensor;
									//					if(ind > (N_LUT-1)){ 
									//						ind -= N_LUT;
									//					}
									//					if(ind < 0 ){
									//						ind += N_LUT;
									//					}
									//					encoder->_calibrationLut[ind] = (float) (error_filt[i*_NPP] - mean);
									//					//Serial.print(ind);
									//					//Serial.print('\t');
									//					//Serial.println(calibrationLut[ind],5);
									//					HAL_Delay(10);
									//					
									//				}
									//				
									////   // de-allocate memory
									////    delete error_filt;
									////    delete error;
									////  //  delete raw_b;
									////    delete error_b;
									////  //  delete raw_f;
									////    delete error_f;

									////    Serial.println("Sensor Calibration Done.");
									//			
									//}

////////////////////////
void get_current(Motor_struct* MOTOR, float dt)
{
//	float LPF_ = 0.92f;
	if(!MOTOR->phase_order)
	{
		MOTOR->adc_b_raw = HAL_ADC_GetValue(&hadc1);
		MOTOR->adc_a_raw = HAL_ADC_GetValue(&hadc2);
	}
	else
	{
		MOTOR->adc_b_raw = HAL_ADC_GetValue(&hadc2);
		MOTOR->adc_a_raw = HAL_ADC_GetValue(&hadc1);
		
	}
	HAL_ADC_Start(&hadc1);
//	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc1, 100);
//	MOTOR->adc_a_raw = ADC_BUFFER[0]*(1-LPF_) + MOTOR->adc_a_raw*LPF_;
//	MOTOR->adc_b_raw = ADC_BUFFER[1]*(1-LPF_) + MOTOR->adc_b_raw*LPF_;
//	MOTOR->adc_c_raw = ADC_BUFFER[2]*(1-LPF_) + MOTOR->adc_c_raw*LPF_;
	
	
	MOTOR->i_a = ( (MOTOR->adc_a_raw - MOTOR->adc_a_offset))*MOTOR->current_gain;
	MOTOR->i_b = ( (MOTOR->adc_b_raw - MOTOR->adc_b_offset))*MOTOR->current_gain;
//	MOTOR->i_c = ( (MOTOR->adc_a_offset - MOTOR->adc_c_raw))*MOTOR->current_gain;
	MOTOR->i_c = - MOTOR->i_a - MOTOR->i_b;
}
void get_current_offset(Motor_struct* MOTOR, uint16_t* ADC_offset_BUFFER)
{

	MOTOR->adc_a_offset = ADC_offset_BUFFER[0];
	MOTOR->adc_b_offset = ADC_offset_BUFFER[1];
	MOTOR->adc_c_offset = ADC_offset_BUFFER[2];


}

void get_DQ_current(Motor_struct* MOTOR, float electric_angle)
{
//	float i_alpha = MOTOR->i_a;  
//  float i_beta  = SQRT1_3 * (MOTOR->i_b - MOTOR->i_c);
	
	float sf = sin_lut(electric_angle);
	float cf = cos_lut(electric_angle);
//	MOTOR->i_d =  __ca * i_alpha + __sa * i_beta;
//	MOTOR->i_q = -__sa * i_alpha + __ca * i_beta;
	
	MOTOR->i_d = 0.6666667f*( cf*MOTOR->i_a + ( SQRT3_2*sf-.5f*cf)*MOTOR->i_b + (-SQRT3_2*sf-.5f*cf)*MOTOR->i_c);   ///Faster DQ0 Transform
  MOTOR->i_q = 0.6666667f*(-sf*MOTOR->i_a - (-SQRT3_2*cf-.5f*sf)*MOTOR->i_b - ( SQRT3_2*cf-.5f*sf)*MOTOR->i_c);
	
	MOTOR->i_q_filt = (1.0f-CURRENT_FILT_ALPHA)*MOTOR->i_q_filt + CURRENT_FILT_ALPHA*MOTOR->i_q;	// these aren't used for control but are sometimes nice for debugging
  MOTOR->i_d_filt = (1.0f-CURRENT_FILT_ALPHA)*MOTOR->i_d_filt + CURRENT_FILT_ALPHA*MOTOR->i_d;
}
void torque_control(Motor_struct* MOTOR)
	{
//		MOTOR->v_des = (MOTOR->p_des - MOTOR->theta_mech)*5000;
    float torque_des = MOTOR->kp*(MOTOR->p_des - MOTOR->theta_mech) + MOTOR->t_ff + MOTOR->kd*(MOTOR->v_des - MOTOR->dtheta_mech);
    MOTOR->i_q_des = fast_fmaxf(fast_fminf(torque_des/(MOTOR->KT*MOTOR->GR), MOTOR->i_max), -MOTOR->i_max);
//		MOTOR->i_q_des = fast_fmaxf(fast_fminf(torque_des/(MOTOR->KT*MOTOR->GR), 5), -5);
    MOTOR->i_d_des = 0.0f;

}

void commutate(Motor_struct* MOTOR, MT6701_sensor * encoder )
{
	/* Do Field Oriented Control */

		MOTOR->theta_elec  = encoder->elec_angle;
		MOTOR->dtheta_elec = encoder->elec_velocity;
		MOTOR->dtheta_mech = encoder->velocity/MOTOR->GR;
		MOTOR->theta_mech  = encoder->_angle_multiturn[0]/MOTOR->GR;
		
		torque_control(MOTOR);
       /// Commutation  ///
//       dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents - 3.8 us
			 get_DQ_current(MOTOR, MOTOR->theta_elec);
	
       
//       MOTOR->v_bus_filt = (1.0f-VBUS_FILT_ALPHA)*controller->v_bus_filt + VBUS_FILT_ALPHA*controller->v_bus;	// used for voltage saturation

//       MOTOR->v_max = OVERMODULATION*controller->v_bus_filt*(DTC_MAX-DTC_MIN)*SQRT1_3;
//       MOTOR->i_max = I_MAX; //I_MAX*(!controller->otw_flag) + I_MAX_CONT*controller->otw_flag;

       limit_norm(&MOTOR->i_d_des, &MOTOR->i_q_des, MOTOR->i_max);	// 2.3 us

       /// PI Controller ///
       float i_d_error = MOTOR->i_d_des - MOTOR->i_d;
       float i_q_error = MOTOR->i_q_des - MOTOR->i_q;


       // Calculate decoupling feed-forward voltages //
       float v_d_ff = 0.0f;//-controller->dtheta_elec*L_Q*controller->i_q;
       float v_q_ff = 0.0f;//controller->dtheta_elec*L_D*controller->i_d;

       MOTOR->v_d = MOTOR->k_d*i_d_error + MOTOR->d_int + v_d_ff;

       MOTOR->v_d = fast_fmaxf(fast_fminf(MOTOR->v_d, MOTOR->v_max), -MOTOR->v_max);
//			 MOTOR->v_d = fast_fmaxf(fast_fminf(MOTOR->v_d, 4), -4);

       MOTOR->d_int += MOTOR->k_d*MOTOR->ki_d*i_d_error;
       MOTOR->d_int = fast_fmaxf(fast_fminf(MOTOR->d_int, MOTOR->v_max), -MOTOR->v_max);
//			 MOTOR->d_int = fast_fmaxf(fast_fminf(MOTOR->d_int, 4), -4);
			 
       float vq_max = sqrtf(MOTOR->v_max*MOTOR->v_max - MOTOR->v_d*MOTOR->v_d);
//			 float vq_max = sqrtf(4*4 - MOTOR->v_d*MOTOR->v_d);

       MOTOR->v_q = MOTOR->k_q*i_q_error + MOTOR->q_int + v_q_ff;
			 MOTOR->v_q = fast_fmaxf(fast_fminf(MOTOR->v_q, vq_max), -vq_max);
       MOTOR->q_int += MOTOR->k_q*MOTOR->ki_q*i_q_error;
       MOTOR->q_int = fast_fmaxf(fast_fminf(MOTOR->q_int, MOTOR->v_max), -MOTOR->v_max);
//			 MOTOR->q_int = fast_fmaxf(fast_fminf(MOTOR->q_int, 4), -4);
			 
//       MOTOR->v_ref = sqrtf(MOTOR->v_d*MOTOR->v_d + MOTOR->v_q*MOTOR->v_q);
       

       limit_norm(&MOTOR->v_d, &MOTOR->v_q, MOTOR->v_max);
			 
			 setPhaseVoltage(MOTOR->v_q, MOTOR->v_d, MOTOR->theta_elec, MOTOR);

//       abc(controller->theta_elec + 1.5f*DT*controller->dtheta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
//       svm(controller->v_max, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

//       set_dtc(controller);

}

