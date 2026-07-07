#include "HWT101.h"
#include "String.h"

static UART_HandleTypeDef *hwt_huart;
volatile double global_angle;
volatile uint8_t new_data_received;
volatile float angular_velocity_y;
volatile float angular_velocity_z;
uint8_t received_data_packet[11];
uint8_t unlock_register[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
uint8_t reset_z_axis[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
uint8_t set_output_200Hz[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
uint8_t set_baudrate_115200[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
uint8_t save_settings[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
uint8_t huifu[]={0xFF, 0xAA, 0x00, 0x01, 0x00};
uint8_t restart_device[] = {0xFF, 0xAA, 0x00, 0xFF, 0x00};
uint8_t guiling[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
struct SAngle 	stcAngle;
static uint16_t packet_counter = 0;
void HWT101_Init(UART_HandleTypeDef *huart)
{
    hwt_huart = huart;

	 USART1_SendArray(guiling, sizeof(guiling));
	 delay_ms(1000);

    HAL_UART_Receive_IT(hwt_huart, &received_data_packet[0], 1);
     delay_ms(200);
  
    global_angle = 0.0f;
    new_data_received = 0;
}

void USART1_SendByte(uint8_t Byte) {
    HAL_UART_Transmit(hwt_huart, &Byte, 1, HAL_MAX_DELAY);
}

void USART1_SendString(char *String) {
    while (*String) {
        USART1_SendByte((uint8_t)*String++);
    }
}

void USART1_Printf(char *format, ...) {
    char buffer[100];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    USART1_SendString(buffer);
}

void USART1_SendArray(uint8_t *array, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        USART1_SendByte(array[i]);; // �ֽڼ���ʱ
    }
	 
	  
        // �������ʱ
}

// �޸ĺ�� hwt_Handler��������Ľṹ���ؼ�������У��͸�λ��
void hwt_Handler(UART_HandleTypeDef *huart) {
    static uint8_t rx_buffer[11];
    static uint8_t rx_index = 0;
    static uint8_t state = 0; // 0: �Ұ�ͷ, 1: ������

    if (huart != hwt_huart) return;

    uint8_t data = received_data_packet[0];

    if (state == 0) {                 // �Ұ�ͷ
        if (data == 0x55) {
            rx_buffer[0] = data;
            rx_index = 1;
            state = 1;
        }
    } else {                           // ������
        rx_buffer[rx_index++] = data;

        if (rx_index == 2) {
            // ֻҪ�Ƕ�(0x53)����ٶ�(0x52)�������������������Ұ�ͷ
            if (rx_buffer[1] != 0x53 && rx_buffer[1] != 0x52) {
                if (rx_buffer[1] == 0x55) { // ���� 0x55�����°�ͷ
                    rx_index = 1;
                } else {
                    state = 0;
                    rx_index = 0;
                }
            }
        } else if (rx_index == 11) {   // �յ�����֡
    // ��ȷ����У��ͣ�SUM = 0x55 + 0x53 + YawH + YawL + VL + VH
    uint8_t sum = rx_buffer[0] + rx_buffer[1] + rx_buffer[6] + rx_buffer[7] + rx_buffer[8] + rx_buffer[9];
    if (sum == rx_buffer[10]) {
        ParseAndPrintData(rx_buffer, 11);
    }
    // ����У���Ƿ�ͨ������λ״̬����������һ֡
    state = 0;
    rx_index = 0;
}
    }
    HAL_UART_Receive_IT(hwt_huart, &received_data_packet[0], 1);
}


void ParseAndPrintData(uint8_t *data, uint16_t length) {
    if (length == 11) {

        if (data[0] == 0x55 && data[1] == 0x53) {
            // ���ݲ��ִ����� 6 ��ʼ�����ֽ���ǰ�����ֽ��ں�
            uint8_t yaw_l = data[6];
            uint8_t yaw_h = data[7];
            int16_t yaw = (int16_t)((yaw_h << 8) | yaw_l);
 
            // �������������ֵת��Ϊ�Ƕ�
            float angle = ((float)yaw / 32768.0) * 180.0;
            global_angle = angle;
            new_data_received = 1;
        } else if (data[0] == 0x55 && data[1] == 0x52) {
            // �������ٶ�����
            uint8_t wy_l = data[4];
            uint8_t wy_h = data[5];
            int16_t wy = (int16_t)((wy_h << 8) | wy_l);
            angular_velocity_y = ((float)wy / 32768.0) * 2000.0;
 
            uint8_t wz_l = data[6];
            uint8_t wz_h = data[7];
            int16_t wz = (int16_t)((wz_h << 8) | wz_l);
            angular_velocity_z = ((float)wz / 32768.0) * 2000.0;
 
            new_data_received = 1;
        }
    }
}

 
uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type) 
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;

}
void HWT101_Clear()
{
//	USART1_SendArray(unlock_register, sizeof(unlock_register));
//	 delay_ms(600);
//	USART1_SendArray(huifu,sizeof(huifu));
//	delay_ms(600);

	USART1_SendArray(guiling, sizeof(guiling));
	 delay_ms(1000);
//USART1_SendArray(save_settings, sizeof(save_settings));
//   delay_ms(200);
}

