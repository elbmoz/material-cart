#ifndef __QRCODE_H
#define __QRCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define QRCODE_TASK_COUNT       3U
#define QRCODE_TASK_GROUP_COUNT 2U
#define QRCODE_TASK_TOTAL_COUNT (QRCODE_TASK_COUNT * QRCODE_TASK_GROUP_COUNT)
#define QRCODE_ERROR_NONE       0U
#define QRCODE_ERROR_DUPLICATE  1U
#define QRCODE_ERROR_UART       2U

extern volatile uint8_t qrcode_task_array[QRCODE_TASK_TOTAL_COUNT];
extern volatile uint8_t qrcode_task_ready;
extern volatile uint32_t qrcode_update_count;
extern volatile uint8_t qrcode_last_error;

HAL_StatusTypeDef QRcode_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef QRcode_Stop(void);
HAL_StatusTypeDef QRcode_WaitTasksAndStop(volatile uint8_t *first_task,
                                          volatile uint8_t *second_task,
                                          volatile uint8_t *third_task,
                                          volatile uint8_t *ready);
HAL_StatusTypeDef QRcode_WaitTaskGroupsAndStop(volatile uint8_t *task1,
                                               volatile uint8_t *task2,
                                               volatile uint8_t *task3,
                                               volatile uint8_t *task4,
                                               volatile uint8_t *task5,
                                               volatile uint8_t *task6,
                                               volatile uint8_t *ready);
void QRcode_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void QRcode_UART_ErrorCallback(UART_HandleTypeDef *huart);
uint8_t QRcode_IsTaskReady(void);
void QRcode_ClearTaskReady(void);
uint8_t QRcode_GetTask(uint8_t index);
void QRcode_GetTasks(uint8_t *first_task, uint8_t *second_task, uint8_t *third_task);
void QRcode_GetTaskGroups(uint8_t *task1,
                          uint8_t *task2,
                          uint8_t *task3,
                          uint8_t *task4,
                          uint8_t *task5,
                          uint8_t *task6);

#ifdef __cplusplus
}
#endif

#endif
