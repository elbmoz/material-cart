/*******************************************************************************
* �ļ���: LobotServoController.h
* ����: �����ֻ������Ƽ�
* ���ڣ�20160806
* LSCϵ�ж�����ư���ο���ʾ��
*******************************************************************************/

#ifndef LOBOTSERVOCONTROLLER_H_
#define LOBOTSERVOCONTROLLER_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define FRAME_HEADER 0x55             //֡ͷ
#define CMD_SERVO_MOVE 0x03           //����ƶ�ָ��
#define CMD_ACTION_GROUP_RUN 0x06     //���ж�����ָ��
#define CMD_ACTION_GROUP_STOP 0x07    //ֹͣ������ָ��
#define CMD_ACTION_GROUP_SPEED 0x0B   //���ö����������ٶ�
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //��ȡ��ص�ѹָ��

extern bool isUartRxCompleted;
extern uint8_t LobotRxBuf[16];
extern uint16_t batteryVolt;
extern void receiveHandle(void);

typedef struct _lobot_servo_ {  //���ID,���Ŀ��λ��
	uint8_t ID;
	uint16_t Position;
} LobotServo;


void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
void stopActionGroup(void);
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
void setAllActionGroupSpeed(uint16_t Speed);
void getBatteryVoltage(void);
void LobotServo_Init(void);

/**
 * @brief 13号360度连续旋转舵机复位
 * @param backOffMs 撞到限位后退回的时间(ms)
 * 
 * 流程: 正转(2500)撞PE7限位 → 停止 → 反转(500)后退 backOffMs → 停止
 */
void servo13_Home(uint16_t backOffMs);

/**
 * @brief 13号360度连续旋转舵机速度控制
 * @param speed 速度值 -1000 ~ +1000
 *        +1000 = 最大速度正转(PWM=2500)
 *            0 = 停止(PWM=1500)
 *        -1000 = 最大速度反转(PWM=500)
 * 
 * Time=0 持续运行，直到下次发送新指令
 */
void servo13_SetSpeed(int16_t speed);

#endif
