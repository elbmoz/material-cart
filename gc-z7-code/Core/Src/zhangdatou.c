#include "zhangdatou.h"

// ���ھ��ָ��
static UART_HandleTypeDef *motor_huart = NULL;
#define CMD_TIMEOUT 1000000000  // ���ʱʱ��(ms)
typedef enum {
    MOTOR_COM_IDLE = 0,
    MOTOR_COM_WAITING_FOR_REPLY
} MotorComState;

typedef enum {
    MOTOR_REQ_NONE = 0,
    MOTOR_REQ_SPEED,
    MOTOR_REQ_POSITION
} MotorRequestType;

#define MOTOR_CACHE_MAX_ADDR 8U
#define MOTOR_RX_BUF_LEN 8U
#define MOTOR_IT_TIMEOUT_MS 20U
#define MOTOR_CLEAR_POSITION_TIMEOUT_MS 100U

static volatile MotorComState g_motor_com_state = MOTOR_COM_IDLE;
static volatile MotorRequestType g_motor_req_type = MOTOR_REQ_NONE;
static volatile uint8_t g_current_motor_addr = 0U;
static volatile uint8_t g_current_motor_cmd = 0U;
static volatile uint8_t g_motor_rx_index = 0U;
static volatile uint8_t g_motor_expected_len = 0U;
static uint8_t g_motor_rx_buf[MOTOR_RX_BUF_LEN];
static uint8_t g_motor_rx_byte;
static volatile HAL_StatusTypeDef g_motor_last_status = HAL_OK;
static volatile uint32_t g_motor_request_tick = 0U;
static volatile int32_t g_motor_speed_cache[MOTOR_CACHE_MAX_ADDR + 1U];
static volatile int32_t g_motor_position_cache[MOTOR_CACHE_MAX_ADDR + 1U];
static volatile uint8_t g_motor_speed_valid[MOTOR_CACHE_MAX_ADDR + 1U];
static volatile uint8_t g_motor_position_valid[MOTOR_CACHE_MAX_ADDR + 1U];
static volatile uint32_t g_motor_speed_tx_count = 0U;
static volatile HAL_StatusTypeDef g_motor_speed_tx_status = HAL_OK;
static uint8_t g_motor_last_speed_cmd[8];

static uint8_t Motor_AddressIsValid(uint8_t address)
{
    return (address > 0U && address <= MOTOR_CACHE_MAX_ADDR) ? 1U : 0U;
}

static void Motor_FinishCom(HAL_StatusTypeDef status)
{
    g_motor_last_status = status;
    g_motor_com_state = MOTOR_COM_IDLE;
    g_motor_req_type = MOTOR_REQ_NONE;
}

static void Motor_ParseReply(void)
{
    uint8_t address = g_current_motor_addr;
    uint8_t sign;
    uint32_t raw_value;
    int32_t signed_value;

    if (g_motor_rx_buf[0] != address || g_motor_rx_buf[g_motor_expected_len - 1U] != 0x6B) {
        Motor_FinishCom(HAL_ERROR);
        return;
    }

    if (g_motor_rx_buf[1] == 0x00 && g_motor_rx_buf[2] == 0xEE) {
        Motor_FinishCom(HAL_ERROR);
        return;
    }

    if (g_motor_rx_buf[1] != g_current_motor_cmd ||
        (g_motor_rx_buf[2] != 0x00 && g_motor_rx_buf[2] != 0x01)) {
        Motor_FinishCom(HAL_ERROR);
        return;
    }

    if (!Motor_AddressIsValid(address)) {
        Motor_FinishCom(HAL_ERROR);
        return;
    }

    sign = g_motor_rx_buf[2];

    if (g_motor_req_type == MOTOR_REQ_SPEED && g_motor_expected_len == 6U) {
        raw_value = ((uint32_t)g_motor_rx_buf[3] << 8) | g_motor_rx_buf[4];
        signed_value = (sign == 0x01) ? -(int32_t)raw_value : (int32_t)raw_value;
        g_motor_speed_cache[address] = signed_value;
        g_motor_speed_valid[address] = 1U;
        Motor_FinishCom(HAL_OK);
        return;
    }

    if (g_motor_req_type == MOTOR_REQ_POSITION && g_motor_expected_len == 8U) {
        raw_value = ((uint32_t)g_motor_rx_buf[3] << 24) |
                    ((uint32_t)g_motor_rx_buf[4] << 16) |
                    ((uint32_t)g_motor_rx_buf[5] << 8) |
                    g_motor_rx_buf[6];
        signed_value = (sign == 0x01) ? -(int32_t)raw_value : (int32_t)raw_value;
        g_motor_position_cache[address] = signed_value;
        g_motor_position_valid[address] = 1U;
        Motor_FinishCom(HAL_OK);
        return;
    }

    Motor_FinishCom(HAL_ERROR);
}

static HAL_StatusTypeDef Motor_StartRequest(uint8_t address,
                                            uint8_t command,
                                            MotorRequestType request_type,
                                            uint8_t expected_len)
{
    uint8_t tx_buf[3];
    HAL_StatusTypeDef status;

    if (motor_huart == NULL || !Motor_AddressIsValid(address)) {
        return HAL_ERROR;
    }

    if (g_motor_com_state != MOTOR_COM_IDLE) {
        return HAL_BUSY;
    }

    tx_buf[0] = address;
    tx_buf[1] = command;
    tx_buf[2] = 0x6B;

    g_current_motor_addr = address;
    g_current_motor_cmd = command;
    g_motor_req_type = request_type;
    g_motor_rx_index = 0U;
    g_motor_expected_len = expected_len;
    g_motor_last_status = HAL_BUSY;
    g_motor_request_tick = HAL_GetTick();
    g_motor_com_state = MOTOR_COM_WAITING_FOR_REPLY;

    HAL_UART_AbortReceive_IT(motor_huart);
    __HAL_UART_CLEAR_OREFLAG(motor_huart);
    __HAL_UART_FLUSH_DRREGISTER(motor_huart);

    status = HAL_UART_Receive_IT(motor_huart, &g_motor_rx_byte, 1U);
    if (status != HAL_OK) {
        Motor_FinishCom(status);
        return status;
    }

    status = HAL_UART_Transmit(motor_huart, tx_buf, sizeof(tx_buf), 100U);
    if (status != HAL_OK) {
        Motor_FinishCom(status);
    }

    return status;
}

/**
  * @brief ��ʼ���������ģ��
  * @param huart: ����ͨ�ŵ�UART���ָ��
  */
void Motor_Init(UART_HandleTypeDef *huart) {
    motor_huart = huart;
}

/**
  * @brief ���ʹ�ܿ���
  * @param address: �����ַ (1-255)
  * @param state: ���״̬ (MOTOR_ENABLE �� MOTOR_DISABLE)
  * @param SNF: ������У�SNF_ENABLE��SNF_DISENABLE)
  */
void Motor_Enable(uint8_t address, MotorState state,SNFMODE SNF) {
    uint8_t cmd[] = {address, 0xF3, 0xAB, state, SNF, 0x6B};
    HAL_UART_Transmit(motor_huart, cmd, sizeof(cmd), CMD_TIMEOUT);
}

/**
  * @brief �ٶ�ģʽ����
  * @param address: �����ַ (1-255)
  * @param dir: ���� (DIRECTION_POSITIVE �� DIRECTION_NEGATIVE)
  * @param slope: ���ٶ� (��λ: RPM/s)
  * @param speed: Ŀ���ٶ� (��λ: 0.1 RPM, ����2000.0 RPM = 20000)*/
/********************************************************************************************************
** 	   SNF:���ͬ�����÷�:������ģʽдSNF_ENABLE,Ȼ�����Motor_SyncStart�Ϳ���ͬʱ�������߹ر�       ****
*********************************************************************************************************/
  
void Motor_SpeedControl(uint8_t address, MotorDirection dir, uint16_t slope, uint16_t speed, SNFMODE SNF) {
    uint8_t cmd[8] = {
        address,
        0xF6,
        dir,
        (uint8_t)(speed >> 8),
        (uint8_t)(speed & 0xFF),
        (uint8_t)slope,
        (uint8_t)SNF,
        0x6B
    };
    HAL_StatusTypeDef status;

    for (uint8_t i = 0U; i < sizeof(cmd); i++) {
        g_motor_last_speed_cmd[i] = cmd[i];
    }
    g_motor_speed_tx_count++;
    status = HAL_UART_Transmit(motor_huart, cmd, sizeof(cmd), CMD_TIMEOUT);
    g_motor_speed_tx_status = status;
}

/**
  * @brief ��������λ��ģʽ����
  * @param address: �����ַ (1-255)
  * @param dir: ���� (DIRECTION_POSITIVE �� DIRECTION_NEGATIVE)
  * @param accel: ���ټ��ٶ� (��λ: RPM/s)
  * @param decel: ���ټ��ٶ� (��λ: RPM/s)
  * @param max_speed: ����ٶ� (��λ: 0.1 RPM)
  * @param position: Ŀ��λ�� (��λ: 0.1��, ����360.0�� = 3600)
  * @param mode: λ��ģʽ (RELATIVE_POSITION �� ABSOLUTE_POSITION)
  */
void Motor_PositionControl(uint8_t address, MotorDirection dir, uint16_t speed, uint8_t acceleration, uint32_t pulses, PositionMode position_mode, SNFMODE SNF) {
    uint8_t cmd[13] = {
        address, 
        0xFD, 
        dir,
        (uint8_t)(speed >> 8),           // �ٶȸ��ֽ�
        (uint8_t)(speed & 0xFF),         // �ٶȵ��ֽ�
        acceleration,                    // ���ٶȵ�λ
        (uint8_t)(pulses >> 24),         // �������ֽ�3�����λ��
        (uint8_t)(pulses >> 16),         // �������ֽ�2
        (uint8_t)(pulses >> 8),          // �������ֽ�1
        (uint8_t)(pulses & 0xFF),        // �������ֽ�0�����λ��
        position_mode,                   // ���/����ģʽ��־
        SNF,                       // ���ͬ����־
        0x6B                            // У���ֽ�
    };
    HAL_UART_Transmit(motor_huart, cmd, sizeof(cmd), CMD_TIMEOUT);
}
/**
  * @brief ����ֹͣ���
  * @param address: �����ַ (1-255)
  */
void Motor_Stop(uint8_t address,SNFMODE SNF) {
    uint8_t cmd[] = {address, 0xFE, 0x98, SNF, 0x6B};
    HAL_UART_Transmit(motor_huart, cmd, sizeof(cmd), CMD_TIMEOUT);
}

/**
  * @brief ���ͬ������
  * @param address: �����ַ (1-255)
  */
void Motor_SyncStart(void) {
    uint8_t cmd[] = {0x00, 0xFF, 0x66, 0x6B};
    HAL_UART_Transmit(motor_huart, cmd, sizeof(cmd), CMD_TIMEOUT);
}

HAL_StatusTypeDef Motor_ReadSpeed(uint8_t address, int32_t *speed_rpm)
{
    if (speed_rpm == NULL || !Motor_AddressIsValid(address)) {
        return HAL_ERROR;
    }

    if (g_motor_speed_valid[address] == 0U) {
        return HAL_BUSY;
    }

    *speed_rpm = g_motor_speed_cache[address];
    return HAL_OK;
}

HAL_StatusTypeDef Motor_ReadPosition(uint8_t address, int32_t *position, float *angle)
{
    int32_t signed_value;

    if (position == NULL || !Motor_AddressIsValid(address)) {
        return HAL_ERROR;
    }

    if (g_motor_position_valid[address] == 0U) {
        return HAL_BUSY;
    }

    signed_value = g_motor_position_cache[address];
    *position = signed_value;
    if (angle != NULL) {
        *angle = (float)signed_value * 360.0f / 65536.0f;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Motor_RequestSpeedUpdate(uint8_t address)
{
    return Motor_StartRequest(address, 0x35, MOTOR_REQ_SPEED, 6U);
}

HAL_StatusTypeDef Motor_RequestPositionUpdate(uint8_t address)
{
    return Motor_StartRequest(address, 0x36, MOTOR_REQ_POSITION, 8U);
}

uint8_t Motor_IsComBusy(void)
{
    if (g_motor_com_state != MOTOR_COM_IDLE &&
        (HAL_GetTick() - g_motor_request_tick) > MOTOR_IT_TIMEOUT_MS) {
        Motor_FinishCom(HAL_TIMEOUT);
    }

    return (g_motor_com_state != MOTOR_COM_IDLE) ? 1U : 0U;
}

HAL_StatusTypeDef Motor_GetLastComStatus(void)
{
    (void)Motor_IsComBusy();
    return g_motor_last_status;
}

uint32_t Motor_GetSpeedTxCount(void)
{
    return g_motor_speed_tx_count;
}

HAL_StatusTypeDef Motor_GetLastSpeedTxStatus(void)
{
    return g_motor_speed_tx_status;
}

void Motor_GetLastSpeedTxCommand(uint8_t *cmd, uint8_t len)
{
    if (cmd == NULL) {
        return;
    }

    if (len > sizeof(g_motor_last_speed_cmd)) {
        len = sizeof(g_motor_last_speed_cmd);
    }

    for (uint8_t i = 0U; i < len; i++) {
        cmd[i] = g_motor_last_speed_cmd[i];
    }
}

void Motor_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (motor_huart == NULL || huart->Instance != motor_huart->Instance ||
        g_motor_com_state != MOTOR_COM_WAITING_FOR_REPLY) {
        return;
    }

    if (g_motor_rx_index >= MOTOR_RX_BUF_LEN) {
        Motor_FinishCom(HAL_ERROR);
        return;
    }

    g_motor_rx_buf[g_motor_rx_index++] = g_motor_rx_byte;

    if (g_motor_rx_index == 3U) {
        if (g_motor_rx_buf[0] != g_current_motor_addr) {
            Motor_FinishCom(HAL_ERROR);
            return;
        }

        if (g_motor_rx_buf[1] == 0x00 && g_motor_rx_buf[2] == 0xEE) {
            g_motor_expected_len = 4U;
        } else if (g_motor_rx_buf[1] != g_current_motor_cmd ||
                   (g_motor_rx_buf[2] != 0x00 && g_motor_rx_buf[2] != 0x01)) {
            Motor_FinishCom(HAL_ERROR);
            return;
        }
    }

    if (g_motor_rx_index >= g_motor_expected_len) {
        Motor_ParseReply();
        return;
    }

    if (HAL_UART_Receive_IT(motor_huart, &g_motor_rx_byte, 1U) != HAL_OK) {
        Motor_FinishCom(HAL_ERROR);
    }
}

void Motor_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (motor_huart != NULL && huart->Instance == motor_huart->Instance) {
        Motor_FinishCom(HAL_ERROR);
    }
}

HAL_StatusTypeDef Motor_ClearPosition(uint8_t address, uint8_t *state_code)
{
    uint8_t cmd[] = {address, 0x0A, 0x6D, 0x6B};
    uint8_t response[4];
    HAL_StatusTypeDef status;

    if (motor_huart == NULL) {
        return HAL_ERROR;
    }

    HAL_UART_AbortReceive_IT(motor_huart);

    status = HAL_UART_Transmit(motor_huart, cmd, sizeof(cmd), CMD_TIMEOUT);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_UART_Receive(motor_huart,
                              response,
                              sizeof(response),
                              MOTOR_CLEAR_POSITION_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    if (response[0] != address || response[3] != 0x6B) {
        return HAL_ERROR;
    }

    if (response[1] == 0x00 && response[2] == 0xEE) {
        return HAL_ERROR;
    }

    if (response[1] != 0x0A) {
        return HAL_ERROR;
    }

    if (state_code != NULL) {
        *state_code = response[2];
    }

    return (response[2] == 0x02) ? HAL_OK : HAL_ERROR;
}

/*��ȡ�ٶȻ��ͽǶȻ�pid������أ���ַ + 0x21 + �ٶȻ���λ�û� PID ���� + У����
����ʾ�������� 01 21 6B����ȷ���� 01 21 00 01 EE B0 00 07 1E D0 00 00 3C F0
00 00 00 1A 6B ����������� 01 00 EE 6B
���ݽ�����
44
��������λ��ģʽλ�û� Kp = 0x0001EEB0 = 126640��
ֱͨ����λ��ģʽλ�û� Kp = 0x00071ED0 = 466640��
�ٶȻ� Kp = 0x00003CF0 = 15600��
�ٶȻ� Ki = 0x0000001A = 26
*/
void Motor_PID(uint8_t address){
	uint8_t cmd[]={address,0x21,0x6B};
}

/*��ȡ������ԭʼֵ
����أ���ַ + 0x29 + ������ԭʼֵ + У���ֽ�
����ʾ�������� 01 29 6B����ȷ���� 01 29 26 72 6B����������� 01 00 EE 6B
���ݽ�����������ԭʼֵ = 0x2672 = 9842
��ע��������ԭʼֵһȦ����ֵ�� 0-16383���� 0-16383 ��ʾ 0-360�㣩
*/
void Motor_bianmaqi(uint8_t address){
	uint8_t cmd[]={address,0x29,0x6B};
}

/*��ܣ���ȡ���λ�����
�����ʽ����ַ + 0x37 + У���ֽ�
����أ���ַ + 0x37 + ���� + ���λ����� + У���ֽ�
����ʾ�������� 01 37 6B����ȷ���� 01 37 01 00 00 00 08 6B����������� 01
00 EE 6B
*/
void Motor_error(uint8_t address){
	uint8_t cmd[]={address,0x37,0x6B};
}

void Returnzero(uint8_t address,SNFMODE SNF){
	uint8_t cmd[]={address,0x9A,0x00,SNF,0x6B};
}
