#include "servo.h"

/* ---- platform parameters (change these for different STM32) ---- */
#define SERVO_TIM              TIM3
#define SERVO_TIM_CH_PA6       TIM_CHANNEL_1
#define SERVO_TIM_CH_PA7       TIM_CHANNEL_2
#define SERVO_GPIO_PORT        GPIOA
#define SERVO_GPIO_PIN_PA6     GPIO_PIN_6
#define SERVO_GPIO_PIN_PA7     GPIO_PIN_7
#define SERVO_GPIO_AF          GPIO_AF2_TIM3

static TIM_HandleTypeDef servo_tim;
static uint8_t servo_initialized = 0U;
static uint16_t servo_pa6_pulse_us = SERVO_CENTER_PULSE_US;
static uint16_t servo_pa7_pulse_us = SERVO_CENTER_PULSE_US;

static uint32_t Servo_GetTimerClockHz(void)
{
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1) {
        timer_clock *= 2U;
    }

    return timer_clock;
}

static uint16_t Servo_ClampPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) {
        return SERVO_MIN_PULSE_US;
    }

    if (pulse_us > SERVO_MAX_PULSE_US) {
        return SERVO_MAX_PULSE_US;
    }

    return pulse_us;
}

static float Servo_ClampAngle(float angle_deg)
{
    if (angle_deg < 0.0f) {
        return 0.0f;
    }

    if (angle_deg > SERVO_MAX_ANGLE_DEG) {
        return SERVO_MAX_ANGLE_DEG;
    }

    return angle_deg;
}

static uint32_t Servo_GetTimChannel(ServoChannel channel)
{
    return (channel == SERVO_PA7) ? SERVO_TIM_CH_PA7 : SERVO_TIM_CH_PA6;
}

static uint16_t *Servo_GetPulseStorage(ServoChannel channel)
{
    return (channel == SERVO_PA7) ? &servo_pa7_pulse_us : &servo_pa6_pulse_us;
}

static void Servo_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = SERVO_GPIO_PIN_PA6 | SERVO_GPIO_PIN_PA7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = SERVO_GPIO_AF;
    HAL_GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStruct);
}

HAL_StatusTypeDef Servo_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    uint32_t timer_clock = Servo_GetTimerClockHz();

    if (timer_clock < 1000000U) {
        return HAL_ERROR;
    }

    Servo_GPIO_Init();
    __HAL_RCC_TIM3_CLK_ENABLE();

    servo_tim.Instance = SERVO_TIM;
    servo_tim.Init.Prescaler = (timer_clock / 1000000U) - 1U;
    servo_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    servo_tim.Init.Period = 20000U - 1U;
    servo_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    servo_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&servo_tim) != HAL_OK) {
        return HAL_ERROR;
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&servo_tim, &sClockSourceConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&servo_tim, &sMasterConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = SERVO_CENTER_PULSE_US;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&servo_tim, &sConfigOC, SERVO_TIM_CH_PA6) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_TIM_PWM_ConfigChannel(&servo_tim, &sConfigOC, SERVO_TIM_CH_PA7) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&servo_tim, SERVO_TIM_CH_PA6) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&servo_tim, SERVO_TIM_CH_PA7) != HAL_OK) {
        return HAL_ERROR;
    }

    servo_pa6_pulse_us = SERVO_CENTER_PULSE_US;
    servo_pa7_pulse_us = SERVO_CENTER_PULSE_US;
    servo_initialized = 1U;

    return HAL_OK;
}

HAL_StatusTypeDef Servo_Start(ServoChannel channel)
{
    if (servo_initialized == 0U) {
        return HAL_ERROR;
    }

    return HAL_TIM_PWM_Start(&servo_tim, Servo_GetTimChannel(channel));
}

HAL_StatusTypeDef Servo_StartAll(void)
{
    if (servo_initialized == 0U) {
        return HAL_ERROR;
    }

    if (Servo_Start(SERVO_PA6) != HAL_OK) {
        return HAL_ERROR;
    }

    return Servo_Start(SERVO_PA7);
}

HAL_StatusTypeDef Servo_SetPulseUs(ServoChannel channel, uint16_t pulse_us)
{
    uint16_t clamped_pulse = Servo_ClampPulse(pulse_us);
    uint16_t *pulse_storage = Servo_GetPulseStorage(channel);

    if (servo_initialized == 0U) {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COMPARE(&servo_tim, Servo_GetTimChannel(channel), clamped_pulse);
    *pulse_storage = clamped_pulse;

    return HAL_OK;
}

HAL_StatusTypeDef Servo_SetAngle(ServoChannel channel, float angle_deg)
{
    float clamped_angle = Servo_ClampAngle(angle_deg);
    float pulse_range = (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    uint16_t pulse_us = (uint16_t)((clamped_angle * pulse_range / SERVO_MAX_ANGLE_DEG)
                                  + (float)SERVO_MIN_PULSE_US + 0.5f);

    return Servo_SetPulseUs(channel, pulse_us);
}

HAL_StatusTypeDef Servo_SetBothPulseUs(uint16_t pa6_pulse_us, uint16_t pa7_pulse_us)
{
    if (Servo_SetPulseUs(SERVO_PA6, pa6_pulse_us) != HAL_OK) {
        return HAL_ERROR;
    }

    return Servo_SetPulseUs(SERVO_PA7, pa7_pulse_us);
}

HAL_StatusTypeDef Servo_SetBothAngle(float pa6_angle_deg, float pa7_angle_deg)
{
    if (Servo_SetAngle(SERVO_PA6, pa6_angle_deg) != HAL_OK) {
        return HAL_ERROR;
    }

    return Servo_SetAngle(SERVO_PA7, pa7_angle_deg);
}

void Servo_Stop(ServoChannel channel)
{
    if (servo_initialized == 0U) {
        return;
    }

    HAL_TIM_PWM_Stop(&servo_tim, Servo_GetTimChannel(channel));
}

void Servo_StopAll(void)
{
    Servo_Stop(SERVO_PA6);
    Servo_Stop(SERVO_PA7);
}

uint16_t Servo_GetPulseUs(ServoChannel channel)
{
    return *Servo_GetPulseStorage(channel);
}

float Servo_GetAngle(ServoChannel channel)
{
    uint16_t pulse_us = Servo_GetPulseUs(channel);
    float pulse_offset = (float)(pulse_us - SERVO_MIN_PULSE_US);
    float pulse_range = (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

    return pulse_offset * SERVO_MAX_ANGLE_DEG / pulse_range;
}
