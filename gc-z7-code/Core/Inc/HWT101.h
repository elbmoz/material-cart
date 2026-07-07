#ifndef HWT101_H
#define HWT101_H

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include "serial.h"
#include "stdint.h"
#include "delay.h"
extern volatile double global_angle;
extern volatile uint8_t new_data_received;
extern volatile float angular_velocity_y;
extern volatile float angular_velocity_z;
extern uint8_t received_data_packet[11];
struct SAngle
{
	short Angle[3];
	short T;
};
// ������������
extern uint8_t unlock_register[];
extern uint8_t reset_z_axis[];
extern uint8_t set_output_200Hz[];
extern uint8_t set_baudrate_115200[];
extern uint8_t save_settings[];
extern uint8_t restart_device[];
uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type);
void ParseAndPrintData(uint8_t *data, uint16_t length);
void hwt_Handler(UART_HandleTypeDef *huart) ;
void USART1_SendArray(uint8_t *array, uint16_t length);
void USART1_Printf(char *format, ...);
void USART1_SendString(char *String);
void USART1_SendByte(uint8_t Byte);
void HWT101_Init(UART_HandleTypeDef *huart);
void HWT101_ProcessData(UART_HandleTypeDef *huart);
void HWT101_Clear();
void CopeSerial2Data(unsigned char ucData);
uint8_t HWT101_ReadData(void);
//double GetCurrentAngle(void);
//uint8_t IsGyroNormal(void);
//uint8_t IsDataValid(uint8_t *data, uint16_t length) ;
//uint8_t IsAngleValid(float angle);
//GyroState GetGyroState(void);
//void ResetGyroError(void) ;
#endif