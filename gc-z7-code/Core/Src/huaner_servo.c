#include "huaner_servo.h"
#include "ServoMotorControl.h"
#include "delay.h"

#define FANGCHE_SERVO_ID        1U
#define FANGCHE_POS_1           2240U
#define FANGCHE_POS_2           1455U
#define FANGCHE_POS_3           720U

#define FANGCHE_USE_NONBLOCKING 0U
#define FANGCHE_MOVE_TIME_MS    2000U
#define FANGCHE_STEP_PWM        10U
#define FANGCHE_FAST_STEP_MS    5U
#define FANGCHE_SLOW_STEP_MS    50U

static uint16_t fangche_current_pos = FANGCHE_POS_1;
static uint8_t fangche_pos_valid = 0U;

static uint16_t fangche_abs_diff(uint16_t a, uint16_t b)
{
	return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

void fangche_smooth_move(uint16_t target_pos)
{
#if FANGCHE_USE_NONBLOCKING
	if (fangche_pos_valid != 0U && fangche_current_pos == target_pos) {
		return;
	}

	moveServo(FANGCHE_SERVO_ID, target_pos, FANGCHE_MOVE_TIME_MS);
	fangche_current_pos = target_pos;
	fangche_pos_valid = 1U;
#else
	uint16_t distance;
	uint16_t steps;
	uint16_t i;
	int32_t start;
	int32_t delta;

	if (fangche_pos_valid == 0U) {
		moveServo(FANGCHE_SERVO_ID, target_pos, 1000);
		fangche_current_pos = target_pos;
		fangche_pos_valid = 1U;
		return;
	}

	distance = fangche_abs_diff(fangche_current_pos, target_pos);
	if (distance == 0U) {
		return;
	}

	steps = (uint16_t)(distance / FANGCHE_STEP_PWM);
	if ((distance % FANGCHE_STEP_PWM) != 0U) {
		steps++;
	}
	if (steps < 1U) {
		steps = 1U;
	}

	start = (int32_t)fangche_current_pos;
	delta = (int32_t)target_pos - start;

	for (i = 1U; i <= steps; i++) {
		uint16_t next_pos = (uint16_t)(start + (delta * (int32_t)i) / (int32_t)steps);
		uint16_t step_time = FANGCHE_FAST_STEP_MS;

		if (i <= 2U || (steps > 2U && i > (uint16_t)(steps - 2U))) {
			step_time = FANGCHE_SLOW_STEP_MS;
		}

		moveServo(FANGCHE_SERVO_ID, next_pos, step_time);
		delay_ms(step_time);
	}

	fangche_current_pos = target_pos;
#endif
}

void zhua(){
	moveServo(12,500,100);
}

void fang(){
	moveServo(12,1250,1000);
}

void fangche1(){
	fangche_smooth_move(FANGCHE_POS_1);
}

void fangche2(){
	fangche_smooth_move(FANGCHE_POS_2);
}

void fangche3(){
	fangche_smooth_move(FANGCHE_POS_3);
}

void kanche(){
	ServoMotorVision_MoveServo0(660,600);
}

void kanche2(){
	ServoMotorVision_MoveServo0(660,600);
}
void kandi(){
	ServoMotorVision_MoveServo0(2115,100);
}

void fang1(){
	moveServo(12,800,1000);
}
//	moveServo(12, 1400, 1000);
//	delay_ms(1000);
//  moveServo(12, 400, 1000);
