#include "monitor.h"
#include "stm32g4xx_hal_fdcan.h"
#include "math_ops.h"

/*///////////////////// CAN //////////////////////////////////////////////////*/
//void can_rx_init(CANRxMessage *msg){
//	msg->filter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 	// set fifo assignment
//	msg->filter.FilterIdHigh=CAN_ID<<5; 				// CAN ID
//	msg->filter.FilterIdLow=0x0;
//	msg->filter.FilterMaskIdHigh=0xFFF;
//	msg->filter.FilterMaskIdLow=0;
//	msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
//	msg->filter.FilterScale=CAN_FILTERSCALE_32BIT;
//	msg->filter.FilterActivation=ENABLE;
//	HAL_CAN_ConfigFilter(&CAN_H, &msg->filter);
//}

//void can_tx_init(CANTxMessage *msg){
//	msg->tx_header.DLC = 6; 			// message size of 8 byte
//	msg->tx_header.IDE=CAN_ID_STD; 		// set identifier to standard
//	msg->tx_header.RTR=CAN_RTR_DATA; 	// set data type to remote transmission request?
//	msg->tx_header.StdId = CAN_MASTER;  // recipient CAN ID
//}

///*
///// CAN Reply Packet Structure ///
///// 16 bit position, between -4*pi and 4*pi
///// 12 bit velocity, between -30 and + 30 rad/s
///// 12 bit current, between -40 and 40;
///// CAN Packet is 5 8-bit words
///// Formatted as follows.  For each quantity, bit 0 is LSB
///// 0: [position[15-8]]
///// 1: [position[7-0]]
///// 2: [velocity[11-4]]
///// 3: [velocity[3-0], current[11-8]]
///// 4: [current[7-0]]
//*/
void pack_reply(CANTxMessage *msg, uint8_t id, float pos, float vel, float torque){
    int p_int = float_to_uint(pos, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(vel, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(torque, T_MIN, T_MAX, 12);
    msg->data[0] = id;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    }
///*
///// CAN Command Packet Structure ///
///// 16 bit position command, between -4*pi and 4*pi
///// 12 bit velocity command, between -30 and + 30 rad/s
///// 12 bit kp, between 0 and 500 N-m/rad
///// 12 bit kd, between 0 and 100 N-m*s/rad
///// 12 bit feed forward torque, between -18 and 18 N-m
///// CAN Packet is 8 8-bit words
///// Formatted as follows.  For each quantity, bit 0 is LSB
///// 0: [position[15-8]]
///// 1: [position[7-0]]
///// 2: [velocity[11-4]]
///// 3: [velocity[3-0], kp[11-8]]
///// 4: [kp[7-0]]
///// 5: [kd[11-4]]
///// 6: [kd[3-0], torque[11-8]]
///// 7: [torque[7-0]]
//		*/
void unpack_cmd(CANRxMessage* msg, Motor_struct* MOTOR){// ControllerStruct * controller){
        int p_int = (msg->data[0]<<8)|msg->data[1];
        int v_int = (msg->data[2]<<4)|(msg->data[3]>>4);
        int kp_int = ((msg->data[3]&0xF)<<8)|msg->data[4];
        int kd_int = (msg->data[5]<<4)|(msg->data[6]>>4);
        int t_int = ((msg->data[6]&0xF)<<8)|msg->data[7];

        MOTOR->p_des = uint_to_float(p_int, P_MIN, P_MAX, 16);
        MOTOR->v_des = uint_to_float(v_int, V_MIN, V_MAX, 12);
        float kp_ = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
        float kd_ = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
//        MOTOR->t_ff = uint_to_float(t_int, -MOTOR->i_max*MOTOR->KT*MOTOR->GR, MOTOR->i_max*MOTOR->KT*MOTOR->GR, 12);
				MOTOR->t_ff = uint_to_float(t_int, T_MIN, T_MAX, 12);
	
    //printf("Received   ");
    //printf("%.3f  %.3f  %.3f  %.3f  %.3f   %.3f", controller->p_des, controller->v_des, controller->kp, controller->kd, controller->t_ff, controller->i_q_ref);
    //printf("\n\r");
}
//void can_tx_rx(){

//	int no_mesage = HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN
//	if(!no_mesage){
//		uint32_t TxMailbox;
//		pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR);	// Pack response
//		HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

//		/* Check for special Commands */
//		if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) & (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFC))){
//			  update_fsm(&state, MOTOR_CMD);
//			}
//		else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFD))){
//			update_fsm(&state, MENU_CMD);
//			}
//		else if(((can_rx.data[0]==0xFF) & (can_rx.data[1]==0xFF) & (can_rx.data[2]==0xFF) & (can_rx.data[3]==0xFF) * (can_rx.data[4]==0xFF) & (can_rx.data[5]==0xFF) & (can_rx.data[6]==0xFF) & (can_rx.data[7]==0xFE))){
//			  update_fsm(&state, ZERO_CMD);
//			}
//		else{
//			  unpack_cmd(can_rx, controller.commands);	// Unpack commands
//			  controller.timeout = 0;					// Reset timeout counter
//		}
//	}
/*///////////////////// UART //////////////////////////////////////////////////*/		
void pack_TX_serial(SerialMessage *msg, float pos, float vel, float torque, float ia, float ib, float ic, float id, float iq, float i_max)
{
		int p_int = float_to_uint(pos, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(vel, V_MIN, V_MAX, 16);
    int t_int = float_to_uint(torque, T_MIN, T_MAX, 16);
		int ia_int = float_to_uint(ia, -i_max, i_max, 16);
		int ib_int = float_to_uint(ib, -i_max, i_max, 16);
		int ic_int = float_to_uint(ic, -i_max, i_max, 16);
		int id_int = float_to_uint(id, -i_max, i_max, 16);
		int iq_int = float_to_uint(iq, -i_max, i_max, 16);
		
    msg->TXdata[0] = p_int>>8;
    msg->TXdata[1] = p_int&0xFF;
	
    msg->TXdata[2] = v_int>>8;
    msg->TXdata[3] = (v_int&0xFF);
	
		msg->TXdata[4] = t_int>>8;
    msg->TXdata[5] = t_int&0xFF;
	
		msg->TXdata[6] = ia_int>>8;
    msg->TXdata[7] = ia_int&0xFF;
	
		msg->TXdata[8] = ib_int>>8;
    msg->TXdata[9] = ib_int&0xFF;
		
		msg->TXdata[10] = ic_int>>8;
    msg->TXdata[11] = ic_int&0xFF;
		
		msg->TXdata[12] = id_int>>8;
    msg->TXdata[13] = id_int&0xFF;
		
		msg->TXdata[14] = iq_int>>8;
    msg->TXdata[15] = iq_int&0xFF;
		
		msg->TXdata[16] = 0x0A; // end of line
		
}

void pack_RX_serial(SerialMessage *msg, float *commands)
{
	
	
}

