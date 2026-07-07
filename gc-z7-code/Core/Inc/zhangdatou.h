#ifndef __ZHANGDATOU_H
#define __ZHANGDATOU_H

#include "stm32f4xx_hal.h"

// 定义控制命令枚举
typedef enum {
    MOTOR_DISABLE = 0x00,
    MOTOR_ENABLE = 0x01
} MotorState;

typedef enum {
    DIRECTION_POSITIVE = 0x00,
    DIRECTION_NEGATIVE = 0x01
} MotorDirection;

typedef enum {
    RELATIVE_POSITION = 0x00,
    ABSOLUTE_POSITION = 0x01
} PositionMode;

typedef enum {
	SNF_ENABLE=0x01,
	SNF_DISABLE=0x00
} SNFMODE;

// 声明控制函数
void Motor_Init(UART_HandleTypeDef *huart);//初始化
void Motor_Enable(uint8_t address, MotorState state,SNFMODE SNF);//电机使能，想用电机必须先使能
void Motor_SpeedControl(uint8_t address, MotorDirection dir, uint16_t slope, uint16_t speed,SNFMODE SNF);//速度模式
void Motor_PositionControl(uint8_t address, MotorDirection dir, uint16_t speed, uint8_t acceleration, uint32_t pulses, PositionMode position_mode, SNFMODE SNF);
void Motor_Stop(uint8_t address,SNFMODE SNF);//停下
void Motor_SyncStart(void);//同步
HAL_StatusTypeDef Motor_ReadSpeed(uint8_t address, int32_t *speed_rpm);//读取电机实时转速(RPM)
HAL_StatusTypeDef Motor_ReadPosition(uint8_t address, int32_t *position, float *angle);//读取电机实时位置和角度
HAL_StatusTypeDef Motor_RequestSpeedUpdate(uint8_t address);//中断方式请求更新速度
HAL_StatusTypeDef Motor_RequestPositionUpdate(uint8_t address);//中断方式请求更新位置
uint8_t Motor_IsComBusy(void);//电机通信是否忙
HAL_StatusTypeDef Motor_GetLastComStatus(void);//最近一次电机通信状态
uint32_t Motor_GetSpeedTxCount(void);//速度命令发送次数
HAL_StatusTypeDef Motor_GetLastSpeedTxStatus(void);//最近一次速度命令发送状态
void Motor_GetLastSpeedTxCommand(uint8_t *cmd, uint8_t len);//最近一次速度命令内容
void Motor_UART_RxCpltCallback(UART_HandleTypeDef *huart);//电机串口接收中断回调入口
void Motor_UART_ErrorCallback(UART_HandleTypeDef *huart);//电机串口错误回调入口
HAL_StatusTypeDef Motor_ClearPosition(uint8_t address, uint8_t *state_code);//将当前位置角度、位置误差、脉冲数清零
void Motor_PID(uint8_t address);//pid
void Motor_bianmaqi(uint8_t address);//给编码器初始值
void Motor_error(uint8_t address);//目标差值
void Returnzero(uint8_t address,SNFMODE SNF);
#endif 
