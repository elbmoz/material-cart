#ifndef __GC_TASK_H
#define __GC_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum {
    GC_IDLE = 0,
    GC_WAIT_BTN,
    GC_RESET,
    GC_SERVO_HOME,
    GC_Z_HOME,
    GC_Z_DOWN,
    GC_OUT,
    GC_saoma,
    GC_wuliaopan1,
    GC_zhuaquA1,
    GC_kaojinA1,

    
    GC_yuanliA1,
    GC_tuihou1,
    GC_xuanzhuan_1,
    GC_cujiagong1,
    
    
    GC_xuanzhuan_2,
    GC_kaojinB1,
    GC_fangzhiB1,
    GC_zhuaquB1,


    GC_yuanliB1,
    GC_guaijiaoA1,
    GC_xuanzhuan_3,
    GC_jingjiagong1,
    GC_kaojinC1,
    GC_fangzhiC1,
    
    GC_yuanliC1,
    GC_guaijiaoB1,
    GC_xuanzhuan_4,
    GC_wuliaopan2,
    GC_kaojinA2,
    GC_zhuaquA2,
    
    GC_yuanliA2,
    GC_tuihou2,
    GC_xuanzhuan_5,
    GC_cujiagong2,
    GC_kaojinB2,
    GC_fangzhiB2,
    GC_zhuaquB2,
    
    
    GC_yuanliB2,
    GC_xuanzhuan_6,
    GC_guaijiaoA2,
    GC_xuanzhuan_7,
    GC_jingjiagong2,
    GC_kaojinC2,
    GC_fangzhiC2,
    
    GC_yuanliC2,
    GC_guaijiaoB2,
    GC_xuanzhuan_8,
    
    GC_zhongdian,
    GC_in,
    GC_DONE,
    GC_FAIL
} GC_TaskState;

void GC_Task_Init(void);
void GC_Task_Reset(void);
void GC_Task_Run(void);
void GC_Task_Run_Pos(void);
GC_TaskState GC_Task_GetState(void);
HAL_StatusTypeDef GC_Task_GetLastStatus(void);
uint8_t GC_Task_IsFinished(void);

#ifdef __cplusplus
}
#endif

#endif
