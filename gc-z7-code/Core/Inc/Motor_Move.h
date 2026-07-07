#ifndef MOTOR_MOVE_H
#define MOTOR_MOVE_H
#include "stm32f4xx_hal.h"
#include "zhangdatou.h"
#include "HWT101.h"

#define LF 0x01
#define RF 0x02
#define LB 0x03
#define RB 0x04
#define Z_AXIS_MOTOR 0x05

#define Z_AXIS_DEFAULT_SPEED 500U
#define Z_AXIS_DEFAULT_ACCEL 20U
#define Z_AXIS_HOME_SPEED 200U
#define Z_AXIS_HOME_SLOPE 10U
#define Z_AXIS_HOME_TIMEOUT_MS 10000U
#define POS_RUN_REACH_DEADBAND 350
#define POS_RUN_TIMEOUT_MS 2000U
// �������ֵ����ַ
#define MOTOR_FL 0x01  // ǰ����
#define MOTOR_FR 0x02  // ǰ����
#define MOTOR_RL 0x03  // ������
#define MOTOR_RR 0x04  // ������

// ת�䷽��ö��
typedef enum {
    TURN_LEFT,
    TURN_RIGHT
} TurnDirection;

typedef enum {
    F = 0,
    B,
    L,
    R
} PosRunDirection;

void Enable();
void Pos_zuoxie(uint16_t acc,uint16_t maxspeed,uint32_t step);
void Pos_youxie(uint16_t acc,uint16_t maxspeed,uint32_t step);
void Pos_youxiaxie(uint16_t acc,uint16_t maxspeed,uint32_t step);
void Pos_Run(PosRunDirection direction,uint16_t acc,uint16_t maxspeed,uint32_t step);
void Pos_RunRight(uint16_t acc,uint16_t maxspeed,uint32_t step);
void Pos_RunLeft(uint16_t acc,uint16_t maxspeed,uint32_t step);
void Speed_RunStraight(uint16_t slope,uint16_t speed);
void Speed_RunBack(uint16_t slope,uint16_t speed);
void Speed_RunLeft(uint16_t slope,uint16_t speed,uint16_t x);
void Speed_RunRight(uint16_t slope,uint16_t speed,uint16_t x);
void Speed_TurnLeft(uint16_t slope,uint16_t speed);
void Speed_TurnRight(uint16_t slope,uint16_t speed);
void Pos_TurnLeft(uint16_t slope,uint16_t speed);
void DifferentialTurn(uint16_t base_speed, uint8_t acceleration, uint32_t base_pulses, 
                     float turn_ratio, TurnDirection turn_dir);
void Pos_TurnRight(uint16_t slope,uint16_t speed);
void Pos_RunStraight(uint16_t acc,uint16_t maxspeed,uint32_t step);
void Pos_RunBack(uint16_t acc,uint16_t maxspeed,uint32_t step);
void Stop_car();
void zero();
void Syn_MoveStraight(uint16_t acc,uint16_t decel,uint16_t maxspeed,uint32_t step);
void Syn_MoveBack(uint16_t acc,uint16_t decel,uint16_t maxspeed,uint32_t step);
void Middle_MoveLeft(uint16_t acc,uint16_t decel,uint16_t maxspeed,uint32_t step);
void Middle_MoveRight(uint16_t acc,uint16_t decel,uint16_t maxspeed,uint32_t step);
void bi_up(uint16_t acc,uint16_t maxspeed,uint32_t step);
void bi_down(uint16_t acc,uint16_t maxspeed,uint32_t step);
void bi_stop();
void ZAxis_Enable(void);
void ZAxis_Disable(void);
void ZAxis_MoveUp(uint8_t acceleration, uint16_t maxspeed, uint32_t pulses);
void ZAxis_MoveDown(uint8_t acceleration, uint16_t maxspeed, uint32_t pulses);
void ZAxis_MoveRelative(int32_t pulses, uint8_t acceleration, uint16_t maxspeed);
void ZAxis_MoveTo(int32_t position, uint8_t acceleration, uint16_t maxspeed);
void ZAxis_Stop(void);
HAL_StatusTypeDef ZAxis_ClearPosition(void);
HAL_StatusTypeDef ZAxis_RequestPositionUpdate(void);
HAL_StatusTypeDef ZAxis_ReadPosition(int32_t *position, float *angle);
uint8_t ZAxis_IsLimitPressed(void);
HAL_StatusTypeDef ZAxis_Home(uint16_t speed, uint16_t slope, uint32_t timeout_ms);
HAL_StatusTypeDef ZAxis_HomeDefault(void);
HAL_StatusTypeDef Motor_Move_ReadPos(uint8_t addr, int32_t *pos);
void Speed_backLeft(uint16_t slope,uint16_t speed,uint16_t x);
void Speed_backRight(uint16_t slope,uint16_t speed,uint16_t x);
#endif
