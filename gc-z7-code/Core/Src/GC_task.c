#include "GC_task.h"

#include "bluetooth.h"
#include "button.h"
#include "encoder_f407.h"
#include "GC_Chassis_Control.h"
#include "laser.h"
#include "LobotServoController.h"
#include "Motor_Move.h"
#include "QRcode.h"
#include "screen.h"
#include "serial.h"
#include "usart.h"
#include "WS2812.h"
#include "action.h"
#include "huaner_servo.h"

#define GC_SERVO13_BACKOFF_MS     80U
#define GC_Z_HOME_SPEED           1000U
#define GC_Z_HOME_SLOPE           200U
#define GC_Z_HOME_TIMEOUT_MS      5000U
#define GC_Z_DOWN_PULSES          (-500)
#define GC_Z_DOWN_SLOPE           200U
#define GC_Z_DOWN_SPEED           1000U
#define GC_Z_DOWN_WAIT_MS         1000U

static GC_TaskState gc_task_state = GC_IDLE;
static HAL_StatusTypeDef gc_task_last_status = HAL_OK;
static uint8_t gc_qrcode_task1 = 0U;
static uint8_t gc_qrcode_task2 = 0U;
static uint8_t gc_qrcode_task3 = 0U;
static uint8_t gc_qrcode_task4 = 0U;
static uint8_t gc_qrcode_task5 = 0U;
static uint8_t gc_qrcode_task6 = 0U;
static uint8_t gc_qrcode_ready = 0U;

static void GC_Task_EnterState(GC_TaskState state)
{
    gc_task_state = state;
}

static HAL_StatusTypeDef GC_Task_ReadQRCodeTasks(void)
{
    volatile uint8_t task1 = 0U;
    volatile uint8_t task2 = 0U;
    volatile uint8_t task3 = 0U;
    volatile uint8_t task4 = 0U;
    volatile uint8_t task5 = 0U;
    volatile uint8_t task6 = 0U;
    volatile uint8_t ready = 0U;
    HAL_StatusTypeDef status;

    status = QRcode_Init(&huart5);
    if (status != HAL_OK) {
        return status;
    }

    status = QRcode_WaitTaskGroupsAndStop(&task1,
                                          &task2,
                                          &task3,
                                          &task4,
                                          &task5,
                                          &task6,
                                          &ready);
    if (status != HAL_OK) {
        return status;
    }

    gc_qrcode_task1 = (uint8_t)task1;
    gc_qrcode_task2 = (uint8_t)task2;
    gc_qrcode_task3 = (uint8_t)task3;
    gc_qrcode_task4 = (uint8_t)task4;
    gc_qrcode_task5 = (uint8_t)task5;
    gc_qrcode_task6 = (uint8_t)task6;
    gc_qrcode_ready = (uint8_t)ready;

    return HAL_OK;
}

void GC_Task_Init(void)
{
    Button_Init();
    Bluetooth_Init(&huart5);
    Screen_Init(&huart4);
    Laser_Init();
    Laser_On();
    gc_task_last_status = HAL_OK;
    GC_Task_EnterState(GC_WAIT_BTN);
}

void GC_Task_Reset(void)
{
    Laser_On();
    gc_task_last_status = HAL_OK;
    GC_Task_EnterState(GC_WAIT_BTN);
}

void GC_Task_Run(void)
{
    HAL_StatusTypeDef status;

    switch (gc_task_state) {
    case GC_IDLE:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_WAIT_BTN);
        break;

    case GC_WAIT_BTN:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        Laser_On();
        if (Button_IsStartPressed() != 0U) {
            Laser_Off();
            GC_Task_EnterState(GC_RESET);
        }
        break;

    case GC_RESET:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        Encoder_Reset();
        GC_Task_EnterState(GC_SERVO_HOME);
        break;

    case GC_SERVO_HOME:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        servo13_Home(GC_SERVO13_BACKOFF_MS);
        GC_Task_EnterState(GC_Z_HOME);
        break;

    case GC_Z_HOME:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        status = ZAxis_Home(GC_Z_HOME_SPEED, GC_Z_HOME_SLOPE, GC_Z_HOME_TIMEOUT_MS);
        if (status != HAL_OK) {
            gc_task_last_status = status;
            WS2812_SetColor(0xFF0000U);
            GC_Task_EnterState(GC_FAIL);
            break;
        }
        GC_Task_EnterState(GC_Z_DOWN);
        break;

    case GC_Z_DOWN:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        ZAxis_MoveRelative(GC_Z_DOWN_PULSES, GC_Z_DOWN_SLOPE, GC_Z_DOWN_SPEED);
        HAL_Delay(GC_Z_DOWN_WAIT_MS);
        GC_Task_EnterState(GC_OUT);
        break;

    case GC_OUT:
        /* TODO: fill GC_OUT action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
		
        Pos_zuoxie(200,2000,4000);//**
		
        //FieldPos_Block(&field_pos, 200.0f, 40.0f, 0.0f);
		
        GC_Task_EnterState(GC_saoma);
        break;

    case GC_saoma:
        /* TODO: fill GC_saoma action. */
       
		
		
        FieldPos_BlockAbs(&field_pos, 630.0f, 0.0f, 0.0f);
		
		
        status = GC_Task_ReadQRCodeTasks();
        if (status != HAL_OK) {
            gc_task_last_status = status;
            WS2812_SetColor(0xFF0000U);
            GC_Task_EnterState(GC_FAIL);
            break;
        }
        Serial_Init(&huart5);
        (void)Screen_SendQRCodeTasks(gc_qrcode_task1,
                                     gc_qrcode_task2,
                                     gc_qrcode_task3,
                                     gc_qrcode_task4,
                                     gc_qrcode_task5,
                                     gc_qrcode_task6);
       
        Bluetooth_SendQRCode(gc_qrcode_task1,
                             gc_qrcode_task2,
                             gc_qrcode_task3,
                             gc_qrcode_task4,
                             gc_qrcode_task5,
                             gc_qrcode_task6);
        GC_Task_EnterState(GC_wuliaopan1);
        break;

    case GC_wuliaopan1:
        /* TODO: fill GC_wuliaopan1 action. */
        
        //FieldPos_BlockAbs(&field_pos, 690.0f,0.0f, 0.0f);
		
		
        //InPlaceTurn_Block(&in_place_turn, 90.0f);
        Pos_RunStraight(150,2000,9000);
				FieldPos_BlockAbs(&field_pos,0.0f,0.0f, 0.0f);
		
        GC_Task_EnterState(GC_kaojinA1);
        break;
		case GC_kaojinA1:
        /* TODO: fill GC_kaojinA1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        //FieldPos_BlockAbs(&field_pos, 0.0f, -75.0f, 0.0f);
				Pos_RunRight(200,2000,1020);
				
        //InPlaceTurn_Block(&in_place_turn, 90.0f);
        //Pos_RunStraight(150,200,6000);
				GC_Task_EnterState(GC_DONE);
        //GC_Task_EnterState(GC_zhuaquA1);
        break;
    case GC_zhuaquA1:
        /* TODO: fill GC_zhuaqu1 action. *///执行抓取程序
				Action_RunQRCodeTasks(gc_qrcode_task1,
                      gc_qrcode_task2,
                      gc_qrcode_task3);
	
        GC_Task_EnterState(GC_yuanliA1);
		
        break;
    
    

    case GC_yuanliA1:
        /* TODO: fill GC_yuanliA1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
         FieldPos_BlockAbs(&field_pos, 0.0f, 145.0f, 0.0f);
        GC_Task_EnterState(GC_tuihou1);
        break;

    case GC_tuihou1:
        /* TODO: fill GC_tuihou1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        FieldPos_BlockAbs(&field_pos, -430.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_xuanzhuan_1);
        break;

    case GC_xuanzhuan_1:
        /* TODO: fill GC_xuanzhuan_1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_cujiagong1);
        break;

    case GC_cujiagong1:
        /* TODO: fill GC_cujiagong1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        FieldPos_BlockAbs(&field_pos, 855.0f, 0.0f, 90.0f);
				FieldPos_BlockAbs(&field_pos, 855.0f, 0.0f, 90.0f);
		
        GC_Task_EnterState(GC_xuanzhuan_2);
        break;

    case GC_xuanzhuan_2:
        /* TODO: fill GC_xuanzhuan_2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        FieldPos_BlockAbs(&field_pos, -80.0f, 0.0f, -180.0f);
        GC_Task_EnterState(GC_kaojinB1);
        break;

    case GC_kaojinB1:
        /* TODO: fill GC_kaojinB1 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, -30.0f, -180.0f);
        GC_Task_EnterState(GC_fangzhiB1);
        break;

    case GC_fangzhiB1:
        /* TODO: fill GC_fangzhiB1 action. */
		
				VisionPrecision_Calibrate();//视觉矫正
		
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
		
				Action_PlaceLoadedQRCodeTasks1(gc_qrcode_task1,
                               gc_qrcode_task2,
                               gc_qrcode_task3);
				
        GC_Task_EnterState(GC_zhuaquB1);
        break;

    case GC_zhuaquB1:
        /* TODO: fill GC_zhuaquB1 action. */
		
				Action_RecoverGroundQRCodeTasks(gc_qrcode_task1,
                                gc_qrcode_task2,
                                gc_qrcode_task3);
		
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
        GC_Task_EnterState(GC_yuanliB1);
        break;

    case GC_yuanliB1:
        /* TODO: fill GC_yuanliB1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_guaijiaoA1);
        break;

    case GC_guaijiaoA1:
        /* TODO: fill GC_guaijiaoA1 action. */
        FieldPos_Block(&field_pos, -780.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_xuanzhuan_3);
        break;

    case GC_xuanzhuan_3:
        /* TODO: fill GC_xuanzhuan_3 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_jingjiagong1);
        break;

    case GC_jingjiagong1:
        /* TODO: fill GC_jingjiagong1 action. */
        FieldPos_BlockAbs(&field_pos, -820.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_kaojinC1);
        break;

    case GC_kaojinC1:
        /* TODO: fill GC_kaojinC1 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, -140.0f, 90.0f);
        GC_Task_EnterState(GC_fangzhiC1);
        break;

    case GC_fangzhiC1:
        /* TODO: fill GC_fangzhiC1 action. */
				VisionPrecision_Calibrate();//视觉矫正
		
			Action_PlaceLoadedQRCodeTasks1(gc_qrcode_task1,
                               gc_qrcode_task2,
                               gc_qrcode_task3);
		
		FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
		
        GC_Task_EnterState(GC_yuanliC1);
		
        break;

    case GC_yuanliC1:
        /* TODO: fill GC_yuanliC1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_guaijiaoB1);
        break;

    case GC_guaijiaoB1:
        /* TODO: fill GC_guaijiaoB1 action. */
        FieldPos_BlockAbs(&field_pos, -(gc_qrcode_task3-1)*152-690.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_xuanzhuan_4);
        break;

    case GC_xuanzhuan_4:
        /* TODO: fill GC_xuanzhuan_4 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_wuliaopan2);
        break;

    case GC_wuliaopan2:
        /* TODO: fill GC_wuliaopan2 action. */
        FieldPos_BlockAbs(&field_pos, -425.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_kaojinA2);
        break;

    case GC_kaojinA2:
        /* TODO: fill GC_kaojinA2 action. */
        FieldPos_BlockAbs(&field_pos,0.0f, -100.0f, 0.0f);
        GC_Task_EnterState(GC_zhuaquA2);
        break;

    case GC_zhuaquA2:
        /* TODO: fill GC_zhuaquA2 action. */
		     status = ZAxis_Home(GC_Z_HOME_SPEED, GC_Z_HOME_SLOPE, GC_Z_HOME_TIMEOUT_MS);
        if (status != HAL_OK) {
            gc_task_last_status = status;
            WS2812_SetColor(0xFF0000U);
            GC_Task_EnterState(GC_FAIL);
            break;
        }
					ZAxis_MoveRelative(GC_Z_DOWN_PULSES, GC_Z_DOWN_SLOPE, GC_Z_DOWN_SPEED);
          HAL_Delay(GC_Z_DOWN_WAIT_MS);
					
					Action_RunQRCodeTasks(gc_qrcode_task4,
                      gc_qrcode_task5,
                      gc_qrcode_task6);
		
        GC_Task_EnterState(GC_yuanliA2);
        break;

    case GC_yuanliA2:
        /* TODO: fill GC_yuanliA2 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 130.0f, 0.0f);
        GC_Task_EnterState(GC_tuihou2);
        break;

    case GC_tuihou2:
        /* TODO: fill GC_tuihou2 action. */
        FieldPos_BlockAbs(&field_pos, -410.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_xuanzhuan_5);
        break;

    case GC_xuanzhuan_5:
        /* TODO: fill GC_xuanzhuan_5 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_cujiagong2);
        break;

    case GC_cujiagong2:
        /* TODO: fill GC_cujiagong2 action. */
        FieldPos_Block(&field_pos, 850.0f, 0.0f, 0.0f);
		
				FieldPos_Block(&field_pos, 850.0f, 0.0f, 0.0f);
		
        GC_Task_EnterState(GC_xuanzhuan_6);
        break;
		
		case GC_xuanzhuan_6:
        /* TODO: fill GC_xuanzhuan_6 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
        GC_Task_EnterState(GC_kaojinB2);
        break;

    case GC_kaojinB2:
        /* TODO: fill GC_kaojinB2 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, -30.0f, -180.0f);
        GC_Task_EnterState(GC_fangzhiB2);
        break;

    case GC_fangzhiB2:
        /* TODO: fill GC_fangzhiB2 action. */
			VisionPrecision_Calibrate();//视觉矫正
				Action_PlaceLoadedQRCodeTasks1(gc_qrcode_task4,
                               gc_qrcode_task5,
                               gc_qrcode_task6);
//		
        GC_Task_EnterState(GC_zhuaquB2);
        break;

    case GC_zhuaquB2:
        /* TODO: fill GC_zhuaquB2 action. */
				Action_RecoverGroundQRCodeTasks(gc_qrcode_task4,
                                gc_qrcode_task5,
                                gc_qrcode_task6);
		
        GC_Task_EnterState(GC_yuanliB2);
        break;

    case GC_yuanliB2:
        /* TODO: fill GC_yuanliB2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_guaijiaoA2);
        break;

   
    case GC_guaijiaoA2:
        /* TODO: fill GC_guaijiaoA2 action. */
        FieldPos_Block(&field_pos, -780.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_xuanzhuan_7);
        break;

    case GC_xuanzhuan_7:
        /* TODO: fill GC_xuanzhuan_7 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_jingjiagong2);
        break;

    case GC_jingjiagong2:
        /* TODO: fill GC_jingjiagong2 action. */
        FieldPos_BlockAbs(&field_pos, -900.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_kaojinC2);
        break;

    case GC_kaojinC2:
        /* TODO: fill GC_kaojinC2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_fangzhiC2);
        break;

    case GC_fangzhiC2:
        /* TODO: fill GC_fangzhiC2 action. */
		

	
				Action_PlaceLoadedQRCodeTasks2(gc_qrcode_task4,
                               gc_qrcode_task5,
                               gc_qrcode_task6);
		
        GC_Task_EnterState(GC_yuanliC2);
        break;

    case GC_yuanliC2:
        /* TODO: fill GC_yuanliC2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_guaijiaoB2);
        break;

    case GC_guaijiaoB2:
        /* TODO: fill GC_guaijiaoB2 action. */
        FieldPos_BlockAbs(&field_pos, -(gc_qrcode_task6-1)*152-718.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_xuanzhuan_8);
        break;

    case GC_xuanzhuan_8:
        /* TODO: fill GC_xuanzhuan_8 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_zhongdian);
        break;

    case GC_zhongdian:
        /* TODO: fill GC_zhongdian action. */
        FieldPos_BlockAbs(&field_pos, -1520.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_in);
        break;

    case GC_in:
        /* TODO: fill GC_in action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
				 Pos_youxiaxie(200,2000,4000);
				GC_Task_EnterState(GC_DONE);
        break;

    case GC_DONE:
    case GC_FAIL:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
    default:
        break;
    }
}

void GC_Task_Run_Pos(void)
{
    HAL_StatusTypeDef status;

    switch (gc_task_state) {
    case GC_IDLE:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_WAIT_BTN);
        break;

    case GC_WAIT_BTN:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        Laser_On();
        if (Button_IsStartPressed() != 0U) {
            Laser_Off();
            GC_Task_EnterState(GC_RESET);
        }
        break;

    case GC_RESET:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        Encoder_Reset();
        GC_Task_EnterState(GC_SERVO_HOME);
        break;

    case GC_SERVO_HOME:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        servo13_Home(GC_SERVO13_BACKOFF_MS);
        GC_Task_EnterState(GC_Z_HOME);
        break;

    case GC_Z_HOME:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        status = ZAxis_Home(GC_Z_HOME_SPEED, GC_Z_HOME_SLOPE, GC_Z_HOME_TIMEOUT_MS);
        if (status != HAL_OK) {
            gc_task_last_status = status;
            WS2812_SetColor(0xFF0000U);
            GC_Task_EnterState(GC_FAIL);
            break;
        }
        GC_Task_EnterState(GC_Z_DOWN);
        break;

    case GC_Z_DOWN:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        ZAxis_MoveRelative(GC_Z_DOWN_PULSES, GC_Z_DOWN_SLOPE, GC_Z_DOWN_SPEED);
        HAL_Delay(GC_Z_DOWN_WAIT_MS);
        GC_Task_EnterState(GC_OUT);
        break;

    case GC_OUT:
        /* TODO: fill GC_OUT action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
		
        Pos_zuoxie(200,2000,4000);//**
		
        //FieldPos_Block(&field_pos, 200.0f, 40.0f, 0.0f);
		
        GC_Task_EnterState(GC_saoma);
        break;

    case GC_saoma:
        /* TODO: fill GC_saoma action. */
       
				delay_ms(100);
		
        //FieldPos_BlockAbs(&field_pos, 630.0f, 0.0f, 0.0f);
				Pos_Run(F,200,2000,8379);//**
		
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
		
        status = GC_Task_ReadQRCodeTasks();
		
        if (status != HAL_OK) {
            gc_task_last_status = status;
            WS2812_SetColor(0xFF0000U);
            GC_Task_EnterState(GC_FAIL);
            break;
        }
        Serial_Init(&huart5);
				
        (void)Screen_SendQRCodeTasks(gc_qrcode_task1,
                                     gc_qrcode_task2,
                                     gc_qrcode_task3,
                                     gc_qrcode_task4,
                                     gc_qrcode_task5,
                                     gc_qrcode_task6);
       
        Bluetooth_SendQRCode(gc_qrcode_task1,
                             gc_qrcode_task2,
                             gc_qrcode_task3,
                             gc_qrcode_task4,
                             gc_qrcode_task5,
                             gc_qrcode_task6);
        GC_Task_EnterState(GC_wuliaopan1);
        break;

    case GC_wuliaopan1:
        /* TODO: fill GC_wuliaopan1 action. */
        
        //FieldPos_BlockAbs(&field_pos, 690.0f,0.0f, 0.0f);
		
		
        //InPlaceTurn_Block(&in_place_turn, 90.0f);
        Pos_RunStraight(150,2000,9000-1330);
				FieldPos_BlockAbs(&field_pos,0.0f,0.0f, 0.0f);
		
        GC_Task_EnterState(GC_kaojinA1);
        break;
		case GC_kaojinA1:
        /* TODO: fill GC_kaojinA1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        //FieldPos_BlockAbs(&field_pos, 0.0f, -75.0f, 0.0f);
				Pos_RunRight(200,2000,500);
				FieldPos_BlockAbs(&field_pos,0.0f,0.0f, 0.0f);
        //InPlaceTurn_Block(&in_place_turn, 90.0f);
        //Pos_RunStraight(150,200,6000);
				//GC_Task_EnterState(GC_DONE);
        GC_Task_EnterState(GC_zhuaquA1);
					//GC_Task_EnterState(GC_yuanliA1);
        break;
    case GC_zhuaquA1:
        /* TODO: fill GC_zhuaqu1 action. *///执行抓取程序
				Action_RunQRCodeTasks_Pos(gc_qrcode_task1,
                      gc_qrcode_task2,
                      gc_qrcode_task3);
	
        GC_Task_EnterState(GC_yuanliA1);
					//GC_Task_EnterState(GC_DONE);
		
        break;
    
    

    case GC_yuanliA1:
        /* TODO: fill GC_yuanliA1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
         //FieldPos_BlockAbs(&field_pos, 0.0f, 145.0f, 0.0f);
				 Pos_RunLeft(150,2000,1930-1020);
				FieldPos_BlockAbs(&field_pos,0.0f,0.0f, 0.0f);
		
		
        GC_Task_EnterState(GC_tuihou1);
        break;

    case GC_tuihou1:
        /* TODO: fill GC_tuihou1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        FieldPos_BlockAbs(&field_pos, -430.0f+20, 0.0f, 0.0f);
				//Pos_RunBack(150,2000,5719);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
		
        GC_Task_EnterState(GC_xuanzhuan_1);
        break;

    case GC_xuanzhuan_1:
        /* TODO: fill GC_xuanzhuan_1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
		
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
				//InPlaceTurn_Block(&in_place_turn, 90.0f);
		
        GC_Task_EnterState(GC_cujiagong1);
        break;

    case GC_cujiagong1:
        /* TODO: fill GC_cujiagong1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        FieldPos_BlockAbs(&field_pos, 845.0f, 0.0f, 90.0f);
				FieldPos_BlockAbs(&field_pos, 845.0f, 0.0f, 90.0f);
				//Pos_Run(F,200,2000,22740-1330);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
		
        GC_Task_EnterState(GC_xuanzhuan_2);
        break;

    case GC_xuanzhuan_2:
        /* TODO: fill GC_xuanzhuan_2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        FieldPos_BlockAbs(&field_pos, -40.0f, 0.0f, -180.0f);
		
        GC_Task_EnterState(GC_kaojinB1);
        break;

    case GC_kaojinB1:
        /* TODO: fill GC_kaojinB1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, -30.0f, -180.0f);
				Pos_Run(R,200,2000,400);
				//Pos_Run(F,200,2000,800);
				//FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
		
			
        GC_Task_EnterState(GC_fangzhiB1);
        break;

    case GC_fangzhiB1:
        /* TODO: fill GC_fangzhiB1 action. */
				//Serial_Init(&huart5);
				VisionPrecision_Calibrate();//视觉矫正
		    //kanche();
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
		
				Action_PlaceLoadedQRCodeTasks1(gc_qrcode_task1,
                               gc_qrcode_task2,
                               gc_qrcode_task3);
				
        GC_Task_EnterState(GC_zhuaquB1);
        break;

    case GC_zhuaquB1:
        /* TODO: fill GC_zhuaquB1 action. */
		
				Action_RecoverGroundQRCodeTasks(gc_qrcode_task1,
                                gc_qrcode_task2,
                                gc_qrcode_task3);
		
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
        GC_Task_EnterState(GC_yuanliB1);
        break;

    case GC_yuanliB1:
        /* TODO: fill GC_yuanliB1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_guaijiaoA1);
        break;

    case GC_guaijiaoA1:
        /* TODO: fill GC_guaijiaoA1 action. */
        FieldPos_Block(&field_pos, -780.0f, 0.0f, 0.0f);
				//Pos_Run(B,200,2000,10374);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
        GC_Task_EnterState(GC_xuanzhuan_3);
        break;

    case GC_xuanzhuan_3:
        /* TODO: fill GC_xuanzhuan_3 action. */
				//InPlaceTurn_Block(&in_place_turn, 90.0f);
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_jingjiagong1);
        break;

    case GC_jingjiagong1:
        /* TODO: fill GC_jingjiagong1 action. */
        FieldPos_BlockAbs(&field_pos, -920.0f, 0.0f, 90.0f);
				//Pos_Run(B,200,2000,12230);
				//FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_kaojinC1);
        break;

    case GC_kaojinC1:
        /* TODO: fill GC_kaojinC1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, -40.0f, 90.0f);
				Pos_Run(R,200,2000,700);
				WS2812_SetRGB(0U, 20U, 0U);
				//FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_fangzhiC1);
        break;

    case GC_fangzhiC1:
        /* TODO: fill GC_fangzhiC1 action. */
				VisionPrecision_Calibrate();//视觉矫正
		    
			Action_PlaceLoadedQRCodeTasks1_C(gc_qrcode_task1,
                               gc_qrcode_task2,
                               gc_qrcode_task3);
		
		FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
		
        GC_Task_EnterState(GC_yuanliC1);
		
        break;

    case GC_yuanliC1:
        /* TODO: fill GC_yuanliC1 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
				kanche();
        GC_Task_EnterState(GC_guaijiaoB1);
        break;

    case GC_guaijiaoB1:
        /* TODO: fill GC_guaijiaoB1 action. */
        //FieldPos_BlockAbs(&field_pos, -(gc_qrcode_task3-1)*152-690.0f, 0.0f, 90.0f);
				FieldPos_BlockAbs(&field_pos, -(gc_qrcode_task3-1)*152-690.0f-50, 0.0f, 90.0f);
				//Pos_Run(B,200,2000,(2-1)*2020+9100);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_xuanzhuan_4);
        break;

    case GC_xuanzhuan_4:
        /* TODO: fill GC_xuanzhuan_4 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_wuliaopan2);
        break;

    case GC_wuliaopan2:
        /* TODO: fill GC_wuliaopan2 action. */
        //FieldPos_BlockAbs(&field_pos, -425.0f, 0.0f, 0.0f);
				Pos_Run(B,200,2000,5650+1330-266);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_kaojinA2);
        break;

    case GC_kaojinA2:
        /* TODO: fill GC_kaojinA2 action. */
        //FieldPos_BlockAbs(&field_pos,0.0f, -100.0f, 0.0f);
				//Pos_Run(R,200,2000,1356);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
				// GC_Task_EnterState(GC_DONE);
		
		
        GC_Task_EnterState(GC_zhuaquA2);
        break;

    case GC_zhuaquA2:
        /* TODO: fill GC_zhuaquA2 action. */
		
		     status = ZAxis_Home(GC_Z_HOME_SPEED, GC_Z_HOME_SLOPE, GC_Z_HOME_TIMEOUT_MS);
        if (status != HAL_OK) {
            gc_task_last_status = status;
            WS2812_SetColor(0xFF0000U);
            GC_Task_EnterState(GC_FAIL);
            break;
        }
					ZAxis_MoveRelative(GC_Z_DOWN_PULSES, GC_Z_DOWN_SLOPE, GC_Z_DOWN_SPEED);
          HAL_Delay(GC_Z_DOWN_WAIT_MS);
					
					Action_RunQRCodeTasks_Pos(gc_qrcode_task4,
                      gc_qrcode_task5,
                      gc_qrcode_task6);
		
        GC_Task_EnterState(GC_yuanliA2);
        break;

    case GC_yuanliA2:
        /* TODO: fill GC_yuanliA2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 130.0f, 0.0f);
				Pos_Run(L,200,2000,1930-1020);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_tuihou2);
        break;

    case GC_tuihou2:
        /* TODO: fill GC_tuihou2 action. */
        FieldPos_BlockAbs(&field_pos, -430.0f+100, 0.0f, 0.0f);
				//Pos_Run(B,200,2000,4800);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_xuanzhuan_5);
        break;

    case GC_xuanzhuan_5:
        /* TODO: fill GC_xuanzhuan_5 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_cujiagong2);
        break;

    case GC_cujiagong2:
        /* TODO: fill GC_cujiagong2 action. */
        FieldPos_BlockAbs(&field_pos, 845.0f, 0.0f, 90.0f);
				FieldPos_BlockAbs(&field_pos, 845.0f, 0.0f, 90.0f);
				//Pos_Run(F,200,2000,21330);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
		
        GC_Task_EnterState(GC_xuanzhuan_6);
        break;
		
		case GC_xuanzhuan_6:
        /* TODO: fill GC_xuanzhuan_6 action. */
        FieldPos_BlockAbs(&field_pos, -40.0f, 0.0f, -180.0f);
        GC_Task_EnterState(GC_kaojinB2);
        break;

    case GC_kaojinB2:
        /* TODO: fill GC_kaojinB2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, -30.0f, -180.0f);
				Pos_Run(R,200,2000,400);
				//Pos_Run(F,200,2000,600);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
        GC_Task_EnterState(GC_fangzhiB2);
        break;

    case GC_fangzhiB2:
        /* TODO: fill GC_fangzhiB2 action. */
			VisionPrecision_Calibrate();//视觉矫正
			//kanche();
				Action_PlaceLoadedQRCodeTasks1(gc_qrcode_task4,
                               gc_qrcode_task5,
                               gc_qrcode_task6);
		
        GC_Task_EnterState(GC_zhuaquB2);
        break;

    case GC_zhuaquB2:
        /* TODO: fill GC_zhuaquB2 action. */
		
				Action_RecoverGroundQRCodeTasks(gc_qrcode_task4,
                                gc_qrcode_task5,
                                gc_qrcode_task6);
		
        GC_Task_EnterState(GC_yuanliB2);
        break;

    case GC_yuanliB2:
        /* TODO: fill GC_yuanliB2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
				
        GC_Task_EnterState(GC_guaijiaoA2);
        break;

   
    case GC_guaijiaoA2:
        /* TODO: fill GC_guaijiaoA2 action. */
        FieldPos_BlockAbs(&field_pos, -750.0f, 0.0f, -180.0f);
				//Pos_Run(B,200,2000,10374);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
        GC_Task_EnterState(GC_xuanzhuan_7);
        break;

    case GC_xuanzhuan_7:
        /* TODO: fill GC_xuanzhuan_7 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_jingjiagong2);
        break;

    case GC_jingjiagong2:
        /* TODO: fill GC_jingjiagong2 action. */
        FieldPos_BlockAbs(&field_pos, -900.0f, 0.0f, 90.0f);
				//Pos_Run(B,200,2000,12230);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_kaojinC2);
        break;

    case GC_kaojinC2:
        /* TODO: fill GC_kaojinC2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
				Pos_Run(R,200,2000,1200);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_fangzhiC2);
        break;

    case GC_fangzhiC2:
        /* TODO: fill GC_fangzhiC2 action. */
		
				VisionPrecision_Calibrate2();//视觉矫正
				
				Action_PlaceLoadedQRCodeTasks2(gc_qrcode_task4,
                               gc_qrcode_task5,
                               gc_qrcode_task6);
		
        GC_Task_EnterState(GC_yuanliC2);
        break;

    case GC_yuanliC2:
        /* TODO: fill GC_yuanliC2 action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
				kanche();
        GC_Task_EnterState(GC_guaijiaoB2);
        break;

    case GC_guaijiaoB2:
        /* TODO: fill GC_guaijiaoB2 action. */
        FieldPos_BlockAbs(&field_pos, -(gc_qrcode_task6-1)*152-718.0f, 0.0f, 90.0f);
				//Pos_Run(B,200,2000,(2-1)*2020+9100);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
        GC_Task_EnterState(GC_xuanzhuan_8);
        break;

    case GC_xuanzhuan_8:
        /* TODO: fill GC_xuanzhuan_8 action. */
        FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_zhongdian);
        break;

    case GC_zhongdian:
        /* TODO: fill GC_zhongdian action. */
        FieldPos_BlockAbs(&field_pos, -1720.0f, 0.0f, 0.0f);
				//Pos_Run(B,200,2000,20216);
				FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
        GC_Task_EnterState(GC_in);
        break;

    case GC_in:
        /* TODO: fill GC_in action. */
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
				 Pos_youxiaxie(200,2000,4700);
				GC_Task_EnterState(GC_DONE);
        break;

    case GC_DONE:
    case GC_FAIL:
        //FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 0.0f);
    default:
        break;
    }
}

GC_TaskState GC_Task_GetState(void)
{
    return gc_task_state;
}

HAL_StatusTypeDef GC_Task_GetLastStatus(void)
{
    return gc_task_last_status;
}

uint8_t GC_Task_IsFinished(void)
{
    return (gc_task_state == GC_DONE) ? 1U : 0U;
}
