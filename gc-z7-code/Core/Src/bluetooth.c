#include "bluetooth.h"
#include "encoder_f407.h"
#include <stdio.h>

static UART_HandleTypeDef *bt_huart;
static volatile uint8_t tx_busy = 0U;
static char tx_buffer[64];

void Bluetooth_Init(UART_HandleTypeDef *huart)
{
    bt_huart = huart;
    tx_busy = 0U;
}

void Bluetooth_TxCplt(void)
{
    tx_busy = 0U;
}

void Bluetooth_SendAngle(float angle)
{
    if (bt_huart == NULL) return;

    int len = snprintf(tx_buffer, sizeof(tx_buffer),
        "Angle: %.1f Enc: %d\r\n", angle, (int)get_encoder_point()->total_count);

    if (len <= 0) return;
    if (len >= (int)sizeof(tx_buffer)) len = sizeof(tx_buffer) - 1;

    HAL_UART_Transmit(bt_huart, (uint8_t *)tx_buffer, (uint16_t)len, 100);
}

void Bluetooth_SendPos(const char *label, int32_t cur, int32_t target)
{
    if (bt_huart == NULL) return;
    int len = snprintf(tx_buffer, sizeof(tx_buffer),
        "%s: cur=%d target=%d diff=%d\r\n",
        label,
        (int)cur,
        (int)target,
        (int)(cur > target ? cur - target : target - cur));
    if (len <= 0) return;
    if (len >= (int)sizeof(tx_buffer)) len = sizeof(tx_buffer) - 1;
    HAL_UART_Transmit(bt_huart, (uint8_t *)tx_buffer, (uint16_t)len, 100);
}

void Bluetooth_SendQRCode(uint8_t task1, uint8_t task2, uint8_t task3,
                           uint8_t task4, uint8_t task5, uint8_t task6)
{
    if (bt_huart == NULL) return;

    int len = snprintf(tx_buffer, sizeof(tx_buffer),
        "QR: %u%u%u+%u%u%u\r\n",
        task1, task2, task3, task4, task5, task6);

    if (len <= 0) return;
    if (len >= (int)sizeof(tx_buffer)) len = sizeof(tx_buffer) - 1;

    HAL_UART_Transmit(bt_huart, (uint8_t *)tx_buffer, (uint16_t)len, 100);
}
