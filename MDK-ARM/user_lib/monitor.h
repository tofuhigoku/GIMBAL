#ifndef __MONITOR_H
#define __MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif
	
//#include <stdint.h>
//#include "stm32f1xx_hal.h"
//#include "main.h"
#include "OwOc.h"
	
extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
	
/* USER CODE BEGIN Private defines */
#define P_MIN -10.0f
#define P_MAX  10.0f
#define V_MIN -15.0f
#define V_MAX  15.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -20.0f
#define T_MAX  20.0f

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct{
	uint8_t id;
	uint8_t RTR_, DLC_;
	uint8_t data[8];
//	CAN_RxHeaderTypeDef rx_header;
//	CAN_FilterTypeDef filter;
}CANRxMessage ;

typedef struct{
	uint8_t id;
	uint8_t data[6];
//	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

typedef struct{
	uint8_t TXdata[17];
	uint8_t RXdata[6];
}SerialMessage ;
//void can_rx_init(CANRxMessage *msg);
//void can_tx_init(CANTxMessage *msg);
void pack_reply(CANTxMessage *msg, uint8_t id, float pos, float vel, float torque);
void unpack_cmd(CANRxMessage* msg, Motor_struct* MOTOR);
//void can_tx_rx();

void pack_TX_serial(SerialMessage *msg, float pos, float vel, float torque, float ia, float ib, float ic, float id, float iq, float i_max);
void pack_RX_serial(SerialMessage *msg, float *commands);

#ifdef __cplusplus
}
#endif

#endif   /*__MONITOR_H*/