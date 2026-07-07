#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define SERVO_MIN_PULSE_US      500U
#define SERVO_CENTER_PULSE_US   1500U
#define SERVO_MAX_PULSE_US      2500U
#define SERVO_MAX_ANGLE_DEG     180.0f

typedef enum
{
    SERVO_PA6 = 0,
    SERVO_PA7 = 1
} ServoChannel;

HAL_StatusTypeDef Servo_Init(void);
HAL_StatusTypeDef Servo_Start(ServoChannel channel);
HAL_StatusTypeDef Servo_StartAll(void);
HAL_StatusTypeDef Servo_SetPulseUs(ServoChannel channel, uint16_t pulse_us);
HAL_StatusTypeDef Servo_SetAngle(ServoChannel channel, float angle_deg);
HAL_StatusTypeDef Servo_SetBothPulseUs(uint16_t pa6_pulse_us, uint16_t pa7_pulse_us);
HAL_StatusTypeDef Servo_SetBothAngle(float pa6_angle_deg, float pa7_angle_deg);
void Servo_Stop(ServoChannel channel);
void Servo_StopAll(void);
uint16_t Servo_GetPulseUs(ServoChannel channel);
float Servo_GetAngle(ServoChannel channel);

#ifdef __cplusplus
}
#endif

#endif
