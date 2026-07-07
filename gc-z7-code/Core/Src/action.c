#include "huaner_servo.h"
#include "Motor_Move.h"
#include "delay.h"
#include "ServoMotorControl.h"
#include "GC_Chassis_Control.h"
#include "action.h"
#include "WS2812.h"
#define ACTION_QRCODE_TASK_COUNT 3U
#define ACTION_PLACE_SIDE_OFFSET_MM 158.0f
#define ACTION_PLACE_USE_POS_RUN 1U
#define ACTION_PLACE_SIDE_OFFSET_STEP 1890L
#define ACTION_PLACE_POS_ACC 200U
#define ACTION_PLACE_POS_SPEED 2000U


static uint8_t Action_IsColorTask(uint8_t task)
{
	return (task >= 1U && task <= 3U) ? 1U : 0U;
}

static void Action_SelectCarSlot(uint8_t slot)
{
	switch (slot) {
	case 1U:
		fangche1();
		break;
	case 2U:
		fangche2();
		break;
	case 3U:
	default:
		fangche3();
		break;
	}
}

static void Action_GrabToCarSlot(uint8_t slot)
{
	switch (slot) {
	case 1U:
		zhuafang1();
		break;
	case 2U:
		zhuafang2();
		break;
	case 3U:
	default:
		zhuafang3();
		break;
	}
}

static void Action_GrabToCarSlot1(uint8_t slot)
{
	switch (slot) {
	case 1U:
		zhuafang8();
		break;
	case 2U:
		zhuafang9();
		break;
	case 3U:
	default:
		zhuafang10();
		break;
	}
}

void Action_RunQRCodeTasks(uint8_t task1, uint8_t task2, uint8_t task3)
{
	uint8_t tasks[ACTION_QRCODE_TASK_COUNT] = {task1, task2, task3};
	uint8_t i;

	for (i = 0U; i < ACTION_QRCODE_TASK_COUNT; i++) {
		if (Action_IsColorTask(tasks[i]) == 0U) {
			continue;
		}

		ServoMotorVision_Calibrate(tasks[i]);
//		delay_ms(1000);
		Action_GrabToCarSlot((uint8_t)(i + 1U));
	}
}

void Action_RunQRCodeTasks_Pos(uint8_t task1, uint8_t task2, uint8_t task3)
{
	uint8_t tasks[ACTION_QRCODE_TASK_COUNT] = {task1, task2, task3};
	uint8_t i;

	for (i = 0U; i < ACTION_QRCODE_TASK_COUNT; i++) {
		if (Action_IsColorTask(tasks[i]) == 0U) {
			continue;
		}

		ServoMotorVision_Calibrate_Pos(tasks[i]);
		//servo13_Home(80);
//		delay_ms(1000);
		Action_GrabToCarSlot((uint8_t)(i + 1U));
	}
}

static float Action_GetPlaceXByTask(uint8_t task)
{
	switch (task) {
	case 1U:
		return -ACTION_PLACE_SIDE_OFFSET_MM;    /* red: left */
	case 3U:
		return ACTION_PLACE_SIDE_OFFSET_MM;     /* blue: right */
	case 2U:
	default:
		return 0.0f;                            /* green: middle */
	}
}

static int32_t Action_GetPlaceStepByTask(uint8_t task)
{
	switch (task) {
	case 1U:
		return -ACTION_PLACE_SIDE_OFFSET_STEP;    /* red: left */
	case 3U:
		return ACTION_PLACE_SIDE_OFFSET_STEP;     /* blue: right */
	case 2U:
	default:
		return 0;                                /* green: middle */
	}
}

static void Action_MoveStepByDelta(int32_t step_delta)
{
	if (step_delta < 0) {
		Pos_Run(B, ACTION_PLACE_POS_ACC, ACTION_PLACE_POS_SPEED, (uint32_t)(-step_delta));
	} else if (step_delta > 0) {
		Pos_Run(F, ACTION_PLACE_POS_ACC, ACTION_PLACE_POS_SPEED, (uint32_t)step_delta);
	}
}

static void Action_MoveToPlaceX(int32_t *current_step, uint8_t task)
{
#if ACTION_PLACE_USE_POS_RUN
	int32_t target_step = Action_GetPlaceStepByTask(task);
	int32_t step_delta = target_step - *current_step;

	if (step_delta != 0) {
		Action_MoveStepByDelta(step_delta);
		*current_step = target_step;
	}
#else
	float target_x = Action_GetPlaceXByTask(task);
	float dx = target_x - (float)(*current_step);

	if (dx > 0.5f || dx < -0.5f) {
		FieldPos_Block(&field_pos, dx, 0.0f, 0.0f);
		*current_step = (int32_t)target_x;
	}
#endif
}

static void Action_MoveToMiddleX(int32_t *current_step)
{
#if ACTION_PLACE_USE_POS_RUN
	int32_t step_delta = 0 - *current_step;

	if (step_delta != 0) {
		Action_MoveStepByDelta(step_delta);
		*current_step = 0;
	}
#else
	float dx = 0.0f - (float)(*current_step);

	if (dx > 0.5f || dx < -0.5f) {
		FieldPos_Block(&field_pos, dx, 0.0f, 0.0f);
		*current_step = 0;
	}
#endif
}

static void Action_GrabFromCarSlot(uint8_t slot)
{
	servo13_Home(95);
	Action_SelectCarSlot(slot);
	delay_ms(500);
	fang();
	ZAxis_MoveRelative(5500, 200, 4000);
	delay_ms(500);
	kanche();
	delay_ms(1000);
	ZAxis_MoveRelative(-1500, 200, 4000);
	delay_ms(1000);
	zhua();
	delay_ms(500);
	ZAxis_MoveRelative(1500, 200, 4000);
	delay_ms(500);
}

void Action_PlaceHeldToGround1(void)
{
	kandi();
	delay_ms(10);
	servo13_Home(100);
	ZAxis_MoveRelative(-13000, 200, 4000);
	delay_ms(2000);
	fang();
	delay_ms(1000);
	ZAxis_MoveRelative(7500, 200, 4000);
}

static void Action_PlaceHeldToGround2(void)
{
	kandi();
	delay_ms(10);
	servo13_Home(100);
	ZAxis_MoveRelative(-7000, 200, 4000);
	delay_ms(2000);
	fang();
	delay_ms(1000);
	ZAxis_MoveRelative(1500, 200, 4000);
}

void Action_PlaceLoadedQRCodeTasks1(uint8_t task1, uint8_t task2, uint8_t task3)
{
	uint8_t tasks[ACTION_QRCODE_TASK_COUNT] = {task1, task2, task3};
	int32_t current_step = 0;
	uint8_t i;

	for (i = 0U; i < ACTION_QRCODE_TASK_COUNT; i++) {
		if (Action_IsColorTask(tasks[i]) == 0U) {
			continue;
		}

		Action_GrabFromCarSlot((uint8_t)(i + 1U));
		Action_MoveToPlaceX(&current_step, tasks[i]);
		FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
		Action_PlaceHeldToGround1();
	}
}

void Action_PlaceLoadedQRCodeTasks1_C(uint8_t task1, uint8_t task2, uint8_t task3)
{
	uint8_t tasks[ACTION_QRCODE_TASK_COUNT] = {task1, task2, task3};
	int32_t current_step = 0;
	uint8_t i;

	for (i = 0U; i < ACTION_QRCODE_TASK_COUNT; i++) {
		if (Action_IsColorTask(tasks[i]) == 0U) {
			continue;
		}

		Action_GrabFromCarSlot((uint8_t)(i + 1U));
		Action_MoveToPlaceX(&current_step, tasks[i]);
		FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
		Action_PlaceHeldToGround1();
	}
}

void Action_PlaceLoadedQRCodeTasks2(uint8_t task1, uint8_t task2, uint8_t task3)
{
	uint8_t tasks[ACTION_QRCODE_TASK_COUNT] = {task1, task2, task3};
	int32_t current_step = 0;
	uint8_t i;

	for (i = 0U; i < ACTION_QRCODE_TASK_COUNT; i++) {
		if (Action_IsColorTask(tasks[i]) == 0U) {
			continue;
		}

		Action_GrabFromCarSlot((uint8_t)(i + 1U));
		Action_MoveToPlaceX(&current_step, tasks[i]);
			FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, 90.0f);
		Action_PlaceHeldToGround2();
	}
}

static int32_t Action_GetLastPlaceStep(uint8_t task1, uint8_t task2, uint8_t task3)
{
	uint8_t tasks[ACTION_QRCODE_TASK_COUNT] = {task1, task2, task3};
	int32_t current_step = 0;
	uint8_t i;

	for (i = 0U; i < ACTION_QRCODE_TASK_COUNT; i++) {
		if (Action_IsColorTask(tasks[i]) != 0U) {
#if ACTION_PLACE_USE_POS_RUN
			current_step = Action_GetPlaceStepByTask(tasks[i]);
#else
			current_step = (int32_t)Action_GetPlaceXByTask(tasks[i]);
#endif
		}
	}

	return current_step;
}

static void Action_GrabRecoverFirstToCarSlot(uint8_t slot)
{
	kandi();
	delay_ms(300);
	Action_SelectCarSlot(slot);
	ZAxis_MoveRelative(-7500, 200, 6000);
	delay_ms(1500);
	zhua();
	delay_ms(500);
	ZAxis_MoveRelative(13000, 200, 6000);
	servo13_Home(60);
	delay_ms(800);
	kanche();
	delay_ms(1000);
	fang();
	delay_ms(500);
}

void Action_RecoverGroundQRCodeTasks(uint8_t task1, uint8_t task2, uint8_t task3)
{
	uint8_t tasks[ACTION_QRCODE_TASK_COUNT] = {task1, task2, task3};
	int32_t current_step = Action_GetLastPlaceStep(task1, task2, task3);
	uint8_t first_recover = 1U;
	uint8_t i;

	for (i = 0U; i < ACTION_QRCODE_TASK_COUNT; i++) {
		if (Action_IsColorTask(tasks[i]) == 0U) {
			continue;
		}

		Action_MoveToPlaceX(&current_step, tasks[i]);
		FieldPos_BlockAbs(&field_pos, 0.0f, 0.0f, -180.0f);
		if (first_recover != 0U) {
			Action_GrabRecoverFirstToCarSlot((uint8_t)(i + 1));
			first_recover = 0U;
		} else {
			Action_GrabToCarSlot1((uint8_t)(i + 1));
		}
	}

	Action_MoveToMiddleX(&current_step);
}


// grab material to car slot 1
void zhuafang1(void){
	ZAxis_MoveRelative(-6500, 200, 6000);//得调
	delay_ms (800);
	zhua();
	delay_ms(500);
	ZAxis_MoveRelative(6500,200,6000);
	fangche1();
	servo13_Home(120);
	delay_ms(300);
	kanche();
	delay_ms(1000);
	fang();
}
void zhuafang2(void){
	ZAxis_MoveRelative(-6500, 200, 6000);//得调
	delay_ms (800);
	zhua();fangche2();
	delay_ms(500);
	ZAxis_MoveRelative(6500,200,6000);
	fangche2();
	servo13_Home(95);
	delay_ms(300);
	kanche();
	delay_ms(1000);
	fang();
}
void zhuafang3(void){
	ZAxis_MoveRelative(-6500, 200, 6000);//得调
	delay_ms (800);
	zhua();fangche3();
	delay_ms(500);
	ZAxis_MoveRelative(6500,200,6000);
	fangche3();
	servo13_Home(95);
	delay_ms(300);
	kanche2();
	delay_ms(2000);
	fang();
}

void zhuafang4(void){
	servo13_Home(75);
	fangche1();
	delay_ms(2000);
	fang();
	ZAxis_MoveRelative(5500, 200, 4000);//得?
	delay_ms(500);
	kanche();
	delay_ms (1000);
	ZAxis_MoveRelative(-1500, 200, 4000);//得调
	delay_ms(1000);
	zhua();
	delay_ms(500);
	ZAxis_MoveRelative(1500, 200, 4000);//得调
	delay_ms(500);
	kandi();
	servo13_Home(100);
	delay_ms(100);
	ZAxis_MoveRelative(-13000, 200, 4000);//得调
	delay_ms(2000);
	fang();
	delay_ms (1000);
	ZAxis_MoveRelative(8000, 200, 4000);//得调
}

void zhuafang5(void){
	servo13_Home(75);
	delay_ms(100);
	fangche2();
	ZAxis_MoveRelative(-1000, 200, 4000);//得调
	delay_ms (1000);
	zhua();
	delay_ms(500);
	ZAxis_MoveRelative(1000, 200, 4000);//得调
	delay_ms(1000);
}

void zhuafang6(void){
	servo13_Home(75);
	delay_ms(100);  
	fangche3();
	ZAxis_MoveRelative(-1000, 200, 4000);//得调
	delay_ms (1000);
	zhua();
	delay_ms(500);
	ZAxis_MoveRelative(1000, 200, 4000);//得调
	delay_ms(1000);
}

void zhuafang7(void){
	
	fang1();
	WS2812_SetRGB(50U, 0U, 0U);
	delay_ms(500);
	
	ZAxis_Home(1000, 200, 5000);
	WS2812_SetRGB(0U, 0U, 50U);
	delay_ms (10);
	
	ZAxis_MoveRelative(-6000, 200, 1000);
	
	kandi();
	
	fang();
	
	servo13_Home(80);
}
void zhuafang7_2(void){
	
	fang1();
	WS2812_SetRGB(50U, 0U, 0U);
	delay_ms(500);
	
	ZAxis_Home(1000, 200, 5000);
	WS2812_SetRGB(0U, 0U, 50U);
	delay_ms (10);
	
	ZAxis_MoveRelative(-1500, 200, 1000);
	
	kandi();
	
	fang();
	
	servo13_Home(80);
}

void zhuafang8(void){
	kandi();
	fangche1();
	delay_ms(1000);
	ZAxis_MoveRelative(-13000, 200, 6000);//謨址
	delay_ms (1500);
	zhua();
	delay_ms(500);
	ZAxis_MoveRelative(13000,200,6000);
	servo13_Home(120);
	delay_ms(800);
	kanche();
	delay_ms(1000);
	fang();
	delay_ms(500);
}

void zhuafang9(void){
	kandi();
	fangche2();
	delay_ms(1000);
	ZAxis_MoveRelative(-13000, 200, 6000);//謨址
	delay_ms (1500);
	zhua();
	delay_ms(500);
	ZAxis_MoveRelative(13000,200,6000);
	servo13_Home(95);
	delay_ms(800);
	kanche();
	delay_ms(1000);
	fang();
	delay_ms(500);
}

void zhuafang10(void){
	kandi();
	fangche3();
	delay_ms(1000);
	ZAxis_MoveRelative(-13000, 200, 6000);//謨址
	delay_ms (1500);
	zhua();
	delay_ms(500);
	ZAxis_MoveRelative(13000,200,6000);
	servo13_Home(95);
	delay_ms(800);
	kanche2();
	delay_ms(1000);
	fang();
	delay_ms(500);
}
