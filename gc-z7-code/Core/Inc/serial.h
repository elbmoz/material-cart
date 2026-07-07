#ifndef SERIAL_H
#define SERIAL_H

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

extern uint8_t Serial_TxPacket[4];
extern uint8_t Serial_RxPacket[4];
extern volatile uint8_t Serial_RxFlag;
extern uint8_t Serial_RxByte;
void Serial_Init(UART_HandleTypeDef *huart);
void Serial_SendByte(uint8_t Byte);
void serial_Handler(UART_HandleTypeDef *huart);
void serial_ErrorHandler(UART_HandleTypeDef *huart);
void Vision_UART_StartCircle(void);
void Vision_UART_StartTask(uint8_t task);
void Vision_UART_StartStream(void);
void Vision_UART_StopStream(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendString(char *String);
void Serial_Printf(char *format, ...);
#endif
