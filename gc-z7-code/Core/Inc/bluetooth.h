#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "stm32f4xx_hal.h"

void Bluetooth_Init(UART_HandleTypeDef *huart);
void Bluetooth_SendAngle(float angle);
void Bluetooth_SendPos(const char *label, int32_t cur, int32_t target);
void Bluetooth_SendQRCode(uint8_t task1, uint8_t task2, uint8_t task3,
                           uint8_t task4, uint8_t task5, uint8_t task6);
void Bluetooth_TxCplt(void);

#endif
