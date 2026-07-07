#include "QRcode.h"

static UART_HandleTypeDef *qrcode_huart;

volatile uint8_t qrcode_task_array[QRCODE_TASK_TOTAL_COUNT] = {0U, 0U, 0U, 0U, 0U, 0U};
volatile uint8_t qrcode_task_ready = 0U;
volatile uint32_t qrcode_update_count = 0U;
volatile uint8_t qrcode_last_error = QRCODE_ERROR_NONE;

static uint8_t qrcode_rx_byte = 0U;
static uint8_t qrcode_parse_buffer[QRCODE_TASK_TOTAL_COUNT] = {0U, 0U, 0U, 0U, 0U, 0U};
static uint8_t qrcode_parse_index = 0U;
static uint8_t qrcode_parse_group_separator = 0U;

static void QRcode_ResetParser(void)
{
    uint8_t i;

    qrcode_parse_index = 0U;
    qrcode_parse_group_separator = 0U;
    for (i = 0U; i < QRCODE_TASK_TOTAL_COUNT; i++) {
        qrcode_parse_buffer[i] = 0U;
    }
}

static uint8_t QRcode_IsTaskDigit(uint8_t data)
{
    return (data >= '1' && data <= '3') ? 1U : 0U;
}

static uint8_t QRcode_IsSeparator(uint8_t data)
{
    if (data == '\r' || data == '\n' || data == ' ' || data == '\t') {
        return 1U;
    }

    return 0U;
}

static uint8_t QRcode_IsGroupSeparator(uint8_t data)
{
    return (data == '+') ? 1U : 0U;
}

static void QRcode_SaveTaskGroups(void)
{
    uint8_t i;

    for (i = 0U; i < QRCODE_TASK_TOTAL_COUNT; i++) {
        qrcode_task_array[i] = qrcode_parse_buffer[i];
    }
    qrcode_task_ready = 1U;
    qrcode_update_count++;
    qrcode_last_error = QRCODE_ERROR_NONE;
    QRcode_ResetParser();
}

static void QRcode_ParseByte(uint8_t data)
{
    uint8_t task_value;

    if (QRcode_IsTaskDigit(data) != 0U) {
        task_value = (uint8_t)(data - '0');
        if (qrcode_parse_index >= QRCODE_TASK_TOTAL_COUNT) {
            QRcode_ResetParser();
        }
        qrcode_parse_buffer[qrcode_parse_index++] = task_value;

        if (qrcode_parse_index >= QRCODE_TASK_TOTAL_COUNT) {
            QRcode_SaveTaskGroups();
        }
        return;
    }

    if (QRcode_IsGroupSeparator(data) != 0U) {
        if (qrcode_parse_index == QRCODE_TASK_COUNT && qrcode_parse_group_separator == 0U) {
            qrcode_parse_group_separator = 1U;
        } else {
            QRcode_ResetParser();
        }
        return;
    }

    if (QRcode_IsSeparator(data) != 0U) {
        QRcode_ResetParser();
    }
}

HAL_StatusTypeDef QRcode_Init(UART_HandleTypeDef *huart)
{
    qrcode_huart = huart;
    QRcode_ResetParser();
    qrcode_task_ready = 0U;
    qrcode_last_error = QRCODE_ERROR_NONE;

    __HAL_UART_CLEAR_OREFLAG(qrcode_huart);
    __HAL_UART_FLUSH_DRREGISTER(qrcode_huart);

    return HAL_UART_Receive_IT(qrcode_huart, &qrcode_rx_byte, 1U);
}

HAL_StatusTypeDef QRcode_Stop(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (qrcode_huart != 0) {
        status = HAL_UART_AbortReceive_IT(qrcode_huart);
        qrcode_huart = 0;
    }

    QRcode_ResetParser();
    qrcode_task_ready = 0U;
    return status;
}

HAL_StatusTypeDef QRcode_WaitTasksAndStop(volatile uint8_t *first_task,
                                          volatile uint8_t *second_task,
                                          volatile uint8_t *third_task,
                                          volatile uint8_t *ready)
{
    uint8_t task1;
    uint8_t task2;
    uint8_t task3;

    while (QRcode_IsTaskReady() == 0U) {
        HAL_Delay(1U);
    }

    QRcode_GetTasks(&task1, &task2, &task3);
    QRcode_ClearTaskReady();

    if (first_task != 0) {
        *first_task = task1;
    }
    if (second_task != 0) {
        *second_task = task2;
    }
    if (third_task != 0) {
        *third_task = task3;
    }
    if (ready != 0) {
        *ready = 1U;
    }

    return QRcode_Stop();
}

HAL_StatusTypeDef QRcode_WaitTaskGroupsAndStop(volatile uint8_t *task1,
                                               volatile uint8_t *task2,
                                               volatile uint8_t *task3,
                                               volatile uint8_t *task4,
                                               volatile uint8_t *task5,
                                               volatile uint8_t *task6,
                                               volatile uint8_t *ready)
{
    uint8_t local_task1;
    uint8_t local_task2;
    uint8_t local_task3;
    uint8_t local_task4;
    uint8_t local_task5;
    uint8_t local_task6;

    while (QRcode_IsTaskReady() == 0U) {
        HAL_Delay(1U);
    }

    QRcode_GetTaskGroups(&local_task1,
                         &local_task2,
                         &local_task3,
                         &local_task4,
                         &local_task5,
                         &local_task6);
    QRcode_ClearTaskReady();

    if (task1 != 0) {
        *task1 = local_task1;
    }
    if (task2 != 0) {
        *task2 = local_task2;
    }
    if (task3 != 0) {
        *task3 = local_task3;
    }
    if (task4 != 0) {
        *task4 = local_task4;
    }
    if (task5 != 0) {
        *task5 = local_task5;
    }
    if (task6 != 0) {
        *task6 = local_task6;
    }
    if (ready != 0) {
        *ready = 1U;
    }

    return QRcode_Stop();
}

void QRcode_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (qrcode_huart == 0 || huart != qrcode_huart) {
        return;
    }

    QRcode_ParseByte(qrcode_rx_byte);
    HAL_UART_Receive_IT(qrcode_huart, &qrcode_rx_byte, 1U);
}

void QRcode_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (qrcode_huart == 0 || huart != qrcode_huart) {
        return;
    }

    qrcode_last_error = QRCODE_ERROR_UART;
    QRcode_ResetParser();
    __HAL_UART_CLEAR_OREFLAG(qrcode_huart);
    __HAL_UART_FLUSH_DRREGISTER(qrcode_huart);
    HAL_UART_Receive_IT(qrcode_huart, &qrcode_rx_byte, 1U);
}

uint8_t QRcode_IsTaskReady(void)
{
    return qrcode_task_ready;
}

void QRcode_ClearTaskReady(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    qrcode_task_ready = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

uint8_t QRcode_GetTask(uint8_t index)
{
    if (index >= QRCODE_TASK_TOTAL_COUNT) {
        return 0U;
    }

    return qrcode_task_array[index];
}

void QRcode_GetTaskGroups(uint8_t *task1,
                          uint8_t *task2,
                          uint8_t *task3,
                          uint8_t *task4,
                          uint8_t *task5,
                          uint8_t *task6)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t local_task1;
    uint8_t local_task2;
    uint8_t local_task3;
    uint8_t local_task4;
    uint8_t local_task5;
    uint8_t local_task6;

    __disable_irq();
    local_task1 = qrcode_task_array[0];
    local_task2 = qrcode_task_array[1];
    local_task3 = qrcode_task_array[2];
    local_task4 = qrcode_task_array[3];
    local_task5 = qrcode_task_array[4];
    local_task6 = qrcode_task_array[5];
    if (primask == 0U) {
        __enable_irq();
    }

    if (task1 != 0) {
        *task1 = local_task1;
    }
    if (task2 != 0) {
        *task2 = local_task2;
    }
    if (task3 != 0) {
        *task3 = local_task3;
    }
    if (task4 != 0) {
        *task4 = local_task4;
    }
    if (task5 != 0) {
        *task5 = local_task5;
    }
    if (task6 != 0) {
        *task6 = local_task6;
    }
}

void QRcode_GetTasks(uint8_t *first_task, uint8_t *second_task, uint8_t *third_task)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t first_value;
    uint8_t second_value;
    uint8_t third_value;

    __disable_irq();
    first_value = qrcode_task_array[0];
    second_value = qrcode_task_array[1];
    third_value = qrcode_task_array[2];
    if (primask == 0U) {
        __enable_irq();
    }

    if (first_task != 0) {
        *first_task = first_value;
    }

    if (second_task != 0) {
        *second_task = second_value;
    }

    if (third_task != 0) {
        *third_task = third_value;
    }
}
