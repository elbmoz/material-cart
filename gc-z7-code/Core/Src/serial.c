#include "serial.h"
#include "GC_Chassis_Control.h"
#include "ServoMotorControl.h"

static UART_HandleTypeDef *serial_huart;

uint8_t Serial_TxPacket[4];
uint8_t Serial_RxPacket[4];
volatile uint8_t Serial_RxFlag = 0;
uint8_t Serial_RxByte;

static char vision_rx_line[32];
static uint8_t vision_rx_index = 0U;

static uint8_t Serial_ParseInt(const char **text, int32_t *value)
{
    int32_t sign = 1;
    int32_t result = 0;
    uint8_t has_digit = 0U;

    while (**text == ' ') {
        (*text)++;
    }

    if (**text == '-') {
        sign = -1;
        (*text)++;
    } else if (**text == '+') {
        (*text)++;
    }

    while (**text >= '0' && **text <= '9') {
        has_digit = 1U;
        result = result * 10 + (int32_t)(**text - '0');
        (*text)++;
    }

    if (has_digit == 0U) {
        return 0U;
    }

    *value = result * sign;
    return 1U;
}

static void Serial_ProcessVisionLine(char *line)
{
    const char *text = line;
    int32_t x_error;
    int32_t y_error;

    if (line[0] == 'n' && line[1] == 'o' && line[2] == 'n' &&
        line[3] == 'e' && line[4] == '\0') {
        VisionPrecision_Update(&vision_precision, 0.0f, 0.0f, 0U);
        ServoMotorVision_Update(&servo_motor_vision, 0.0f, 0.0f, 0U);
        return;
    }

    if (Serial_ParseInt(&text, &x_error) == 0U) {
        return;
    }

    while (*text == ' ') {
        text++;
    }

    if (*text != ',') {
        return;
    }
    text++;

    if (Serial_ParseInt(&text, &y_error) == 0U) {
        return;
    }

    VisionPrecision_Update(&vision_precision, (float)x_error, (float)y_error, 1U);
    ServoMotorVision_Update(&servo_motor_vision, (float)x_error, (float)y_error, 1U);
}

void Serial_Init(UART_HandleTypeDef *huart)
{
    serial_huart = huart;
    HAL_UART_Receive_IT(serial_huart, &Serial_RxByte, 1U);
}

void Serial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(serial_huart, &Byte, 1U, HAL_MAX_DELAY);
}

void Vision_UART_StartCircle(void)
{
    uint8_t command = 'j';
    HAL_UART_Transmit(serial_huart, &command, 1U, 10000U);
}

void Vision_UART_StartTask(uint8_t task)
{
    uint8_t command[] = {'c', '3'};

    if (task >= 1U && task <= 3U) {
        command[1] = (uint8_t)('0' + task);
    }

    HAL_UART_Transmit(serial_huart, command, sizeof(command), 10000U);
}

void Vision_UART_StartStream(void)
{
    Vision_UART_StartTask(3U);
}

void Vision_UART_StopStream(void)
{
    uint8_t command[] = {'o', 'k'};
    HAL_UART_Transmit(serial_huart, command, sizeof(command), 1000U);
}

void Serial_SendString(char *String)
{
    while (*String) {
        Serial_SendByte((uint8_t)*String++);
    }
}

void Serial_Printf(char *format, ...)
{
    char buffer[100];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial_SendString(buffer);
}

void serial_Handler(UART_HandleTypeDef *huart)
{
    uint8_t rx_data;

    if (huart == serial_huart) {
        rx_data = Serial_RxByte;

        if (rx_data == '\n') {
            vision_rx_line[vision_rx_index] = '\0';
            Serial_ProcessVisionLine(vision_rx_line);
            vision_rx_index = 0U;
        } else if (rx_data != '\r') {
            if (vision_rx_index < (sizeof(vision_rx_line) - 1U)) {
                vision_rx_line[vision_rx_index++] = (char)rx_data;
            } else {
                vision_rx_index = 0U;
            }
        }

        HAL_UART_Receive_IT(serial_huart, &Serial_RxByte, 1U);
    }
}

void serial_ErrorHandler(UART_HandleTypeDef *huart)
{
    if (huart == serial_huart) {
        vision_rx_index = 0U;
        HAL_UART_Receive_IT(serial_huart, &Serial_RxByte, 1U);
    }
}

int fputc(int ch, FILE *f)
{
    Serial_SendByte((uint8_t)ch);
    return ch;
}
