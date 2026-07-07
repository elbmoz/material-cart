#ifndef ACTION_H
#define ACTION_H
#include "stm32f4xx_hal.h"
#include "Motor_Move.h"

void zhuafang1(void);
void zhuafang2(void);
void zhuafang3(void);
void Action_RunQRCodeTasks(uint8_t task1, uint8_t task2, uint8_t task3);
void Action_RunQRCodeTasks_Pos(uint8_t task1, uint8_t task2, uint8_t task3);
void Action_PlaceLoadedQRCodeTasks1(uint8_t task1, uint8_t task2, uint8_t task3);
void Action_PlaceLoadedQRCodeTasks1_C(uint8_t task1, uint8_t task2, uint8_t task3);
void Action_PlaceLoadedQRCodeTasks2(uint8_t task1, uint8_t task2, uint8_t task3);
void Action_RecoverGroundQRCodeTasks(uint8_t task1, uint8_t task2, uint8_t task3);
void Action_PlaceHeldToGround1(void);
void zhuafang7_2(void);
void zhuafang4(void);
void zhuafang5(void);
void zhuafang6(void);
void zhuafang7(void);
void zhuafang8(void);
void zhuafang9(void);
void zhuafang10(void);
#endif
