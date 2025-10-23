#ifndef USER_CONNECT_SENSOR_H__
#define USER_CONNECT_SENSOR_H__

#define USING_CONNECT_SENSOR

#include "user_util.h"
#include "event_driven.h"

/*==================Define UART sensor===================*/
extern DMA_HandleTypeDef        hdma_usart2_rx;

#define uart_rs232	                huart2
#define uart_dma_rs232              hdma_usart2_rx

#define MX_UART_RS232_Init          MX_USART2_UART_Init            //Func init Uart

/* Definition for USARTx's NVIC */
#define USART_RS232_IRQn            USART2_IRQn
#define USART_RS232_Priority        1

/*-- Rx uart: DMA with IDLE line | Interupt line  ----*/
#define UART_RS232_DMA              0
#define UART_RS232_IT               1

#define UART_RS232_MODE             UART_RS232_IT

/*======================Function======================*/
void RS232_Init_Data (void);
void RS232_Init_Uart (void);
void RS232_Init_RX_Mode(void);
void RS232_Stop_RX_Mode (void);
void RS232_Rx_Callback (uint16_t Size);


#endif
