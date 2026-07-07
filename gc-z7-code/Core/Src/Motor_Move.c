#include "Motor_Move.h"
#include "delay.h"

#define STEP_TO_READBACK(s) ((int32_t)(s) * 2048 / 100)
//ʹ��
void Enable(){
	Motor_Enable(LF,MOTOR_ENABLE,SNF_ENABLE);
	delay_ms(10);
	Motor_Enable(LB,MOTOR_ENABLE,SNF_ENABLE);
	delay_ms(10);
	Motor_Enable(RF,MOTOR_ENABLE,SNF_ENABLE);
	delay_ms(10);
	Motor_Enable(RB,MOTOR_ENABLE,SNF_ENABLE);
	delay_ms(10);
	Motor_Enable(Z_AXIS_MOTOR,MOTOR_ENABLE,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}


/*
 @param slope: ���ٶ� (��λ: RPM/s)
 @param speed: Ŀ���ٶ� (��λ: 0.1 RPM, ����2000.0 RPM = 20000)
*/
//�ٶ�ģʽֱ��
void Speed_RunStraight(uint16_t slope,uint16_t speed){
	Motor_SpeedControl(LF,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(LB,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RF,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RB,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}
//�ٶ�ģʽ����
void Speed_RunBack(uint16_t slope,uint16_t speed){
	Motor_SpeedControl(LF,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(LB,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RF,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RB,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}
//�ٶ�ģʽ�����
void Speed_RunLeft(uint16_t slope,uint16_t speed,uint16_t x){
	Motor_SpeedControl(LF,DIRECTION_POSITIVE,slope,speed/x,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(LB,DIRECTION_POSITIVE,slope,speed/x,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RF,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RB,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}
//�ٶ�ģʽ�Һ���
void Speed_RunRight(uint16_t slope,uint16_t speed,uint16_t x){
	Motor_SpeedControl(LF,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(LB,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RF,DIRECTION_NEGATIVE,slope,speed/x,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RB,DIRECTION_POSITIVE,slope,speed/x,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}

void Speed_backLeft(uint16_t slope,uint16_t speed,uint16_t x){
	Motor_SpeedControl(LF,DIRECTION_NEGATIVE,slope,speed/x,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(LB,DIRECTION_NEGATIVE,slope,speed/x,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RF,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RB,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}
//�ٶ�ģʽ�Һ���
void Speed_backRight(uint16_t slope,uint16_t speed,uint16_t x){
	Motor_SpeedControl(LF,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(LB,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RF,DIRECTION_POSITIVE,slope,speed/x,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RB,DIRECTION_POSITIVE,slope,speed/x,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}
//�ٶ�ģʽ��ת
void Speed_TurnLeft(uint16_t slope,uint16_t speed){
	Motor_SpeedControl(LF,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(LB,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RF,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RB,DIRECTION_NEGATIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}
//�ٶ�ģʽ��ת
void Speed_TurnRight(uint16_t slope,uint16_t speed){
	Motor_SpeedControl(LF,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(LB,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RF,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SpeedControl(RB,DIRECTION_POSITIVE,slope,speed,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}
//λ��ģʽֱ��
void Pos_RunStraight(uint16_t acc,uint16_t maxspeed,uint32_t step){
	int32_t lf_t = STEP_TO_READBACK(step);
	int32_t lb_t = STEP_TO_READBACK(step);
	int32_t rf_t = -STEP_TO_READBACK(step);
	int32_t rb_t = -STEP_TO_READBACK(step);
	Motor_ClearPosition(LF, NULL); delay_ms(1);
	Motor_ClearPosition(LB, NULL); delay_ms(1);
	Motor_ClearPosition(RF, NULL); delay_ms(1);
	Motor_ClearPosition(RB, NULL); delay_ms(1);
	Motor_PositionControl(LF,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(LB,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RF,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RB,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_SyncStart();
	delay_ms(1);
	uint32_t t0 = HAL_GetTick();
	while (1) {
		int32_t lf, lb, rf, rb;
		Motor_Move_ReadPos(LF, &lf);
		Motor_Move_ReadPos(LB, &lb);
		Motor_Move_ReadPos(RF, &rf);
		Motor_Move_ReadPos(RB, &rb);
		if ((lf > lf_t ? lf - lf_t : lf_t - lf) < POS_RUN_REACH_DEADBAND &&
		    (lb > lb_t ? lb - lb_t : lb_t - lb) < POS_RUN_REACH_DEADBAND &&
		    (rf > rf_t ? rf - rf_t : rf_t - rf) < POS_RUN_REACH_DEADBAND &&
		    (rb > rb_t ? rb - rb_t : rb_t - rb) < POS_RUN_REACH_DEADBAND) break;
		if (HAL_GetTick() - t0 > POS_RUN_TIMEOUT_MS) break;
		delay_ms(100);
	}
}
//λ��ģʽ����
void Pos_RunBack(uint16_t acc,uint16_t maxspeed,uint32_t step){
	int32_t lf_t = -STEP_TO_READBACK(step);
	int32_t lb_t = -STEP_TO_READBACK(step);
	int32_t rf_t = STEP_TO_READBACK(step);
	int32_t rb_t = STEP_TO_READBACK(step);
	Motor_ClearPosition(LF, NULL); delay_ms(1);
	Motor_ClearPosition(LB, NULL); delay_ms(1);
	Motor_ClearPosition(RF, NULL); delay_ms(1);
	Motor_ClearPosition(RB, NULL); delay_ms(1);
	Motor_PositionControl(LF,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(LB,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RF,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RB,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_SyncStart();
	delay_ms(1);
	uint32_t t0 = HAL_GetTick();
	while (1) {
		int32_t lf, lb, rf, rb;
		Motor_Move_ReadPos(LF, &lf);
		Motor_Move_ReadPos(LB, &lb);
		Motor_Move_ReadPos(RF, &rf);
		Motor_Move_ReadPos(RB, &rb);
		if ((lf > lf_t ? lf - lf_t : lf_t - lf) < POS_RUN_REACH_DEADBAND &&
		    (lb > lb_t ? lb - lb_t : lb_t - lb) < POS_RUN_REACH_DEADBAND &&
		    (rf > rf_t ? rf - rf_t : rf_t - rf) < POS_RUN_REACH_DEADBAND &&
		    (rb > rb_t ? rb - rb_t : rb_t - rb) < POS_RUN_REACH_DEADBAND) break;
		if (HAL_GetTick() - t0 > POS_RUN_TIMEOUT_MS) break;
		delay_ms(100);
	}
}
void Pos_RunRight(uint16_t acc,uint16_t maxspeed,uint32_t step){
	int32_t lf_t = -STEP_TO_READBACK(step);
	int32_t lb_t = STEP_TO_READBACK(step);
	int32_t rf_t = -STEP_TO_READBACK(step);
	int32_t rb_t = STEP_TO_READBACK(step);
	Motor_ClearPosition(LF, NULL); delay_ms(1);
	Motor_ClearPosition(LB, NULL); delay_ms(1);
	Motor_ClearPosition(RF, NULL); delay_ms(1);
	Motor_ClearPosition(RB, NULL); delay_ms(1);
	Motor_PositionControl(LF,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(LB,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RF,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RB,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_SyncStart();
	delay_ms(1);
	uint32_t t0 = HAL_GetTick();
	while (1) {
		int32_t lf, lb, rf, rb;
		Motor_Move_ReadPos(LF, &lf);
		Motor_Move_ReadPos(LB, &lb);
		Motor_Move_ReadPos(RF, &rf);
		Motor_Move_ReadPos(RB, &rb);
		if ((lf > lf_t ? lf - lf_t : lf_t - lf) < POS_RUN_REACH_DEADBAND &&
		    (lb > lb_t ? lb - lb_t : lb_t - lb) < POS_RUN_REACH_DEADBAND &&
		    (rf > rf_t ? rf - rf_t : rf_t - rf) < POS_RUN_REACH_DEADBAND &&
		    (rb > rb_t ? rb - rb_t : rb_t - rb) < POS_RUN_REACH_DEADBAND) break;
		if (HAL_GetTick() - t0 > POS_RUN_TIMEOUT_MS) break;
		delay_ms(100);
	}
}

void Pos_RunLeft(uint16_t acc,uint16_t maxspeed,uint32_t step){
	int32_t lf_t = -STEP_TO_READBACK(step);
	int32_t lb_t = STEP_TO_READBACK(step);
	int32_t rf_t = -STEP_TO_READBACK(step);
	int32_t rb_t = STEP_TO_READBACK(step);
	Motor_ClearPosition(LF, NULL); delay_ms(1);
	Motor_ClearPosition(LB, NULL); delay_ms(1);
	Motor_ClearPosition(RF, NULL); delay_ms(1);
	Motor_ClearPosition(RB, NULL); delay_ms(1);
	Motor_PositionControl(LF,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(LB,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RF,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RB,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_SyncStart();
	delay_ms(1);
	uint32_t t0 = HAL_GetTick();
	while (1) {
		int32_t lf, lb, rf, rb;
		Motor_Move_ReadPos(LF, &lf);
		Motor_Move_ReadPos(LB, &lb);
		Motor_Move_ReadPos(RF, &rf);
		Motor_Move_ReadPos(RB, &rb);
		if ((lf > lf_t ? lf - lf_t : lf_t - lf) < POS_RUN_REACH_DEADBAND &&
		    (lb > lb_t ? lb - lb_t : lb_t - lb) < POS_RUN_REACH_DEADBAND &&
		    (rf > rf_t ? rf - rf_t : rf_t - rf) < POS_RUN_REACH_DEADBAND &&
		    (rb > rb_t ? rb - rb_t : rb_t - rb) < POS_RUN_REACH_DEADBAND) break;
		if (HAL_GetTick() - t0 > POS_RUN_TIMEOUT_MS) break;
		delay_ms(100);
	}
}

void Pos_Run(PosRunDirection direction,uint16_t acc,uint16_t maxspeed,uint32_t step){
	switch (direction) {
	case F:
		Pos_RunStraight(acc,maxspeed,step);
		break;
	case B:
		Pos_RunBack(acc,maxspeed,step);
		break;
	case L:
		Pos_RunLeft(acc,maxspeed,step);
		break;
	case R:
		Pos_RunRight(acc,maxspeed,step);
		break;
	default:
		break;
	}
}

void Pos_zuoxie(uint16_t acc,uint16_t maxspeed,uint32_t step){
	int32_t lb_t = STEP_TO_READBACK(step);
	int32_t rf_t = -STEP_TO_READBACK(step);
	Motor_ClearPosition(LB, NULL); delay_ms(1);
	Motor_ClearPosition(RF, NULL); delay_ms(1);
	Motor_PositionControl(LF,DIRECTION_NEGATIVE,0,0,0,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(LB,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RF,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RB,DIRECTION_POSITIVE,0,0,0,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_SyncStart();
	delay_ms(1);
	uint32_t t0 = HAL_GetTick();
	while (1) {
		int32_t lb, rf;
		Motor_Move_ReadPos(LB, &lb);
		Motor_Move_ReadPos(RF, &rf);
		if ((lb > lb_t ? lb - lb_t : lb_t - lb) < POS_RUN_REACH_DEADBAND &&
		    (rf > rf_t ? rf - rf_t : rf_t - rf) < POS_RUN_REACH_DEADBAND) break;
		if (HAL_GetTick() - t0 > POS_RUN_TIMEOUT_MS) break;
		delay_ms(100);
	}
}

void Pos_youxiaxie(uint16_t acc,uint16_t maxspeed,uint32_t step){
	int32_t lb_t = STEP_TO_READBACK(step);
	int32_t rf_t = -STEP_TO_READBACK(step);
	Motor_ClearPosition(LB, NULL); delay_ms(1);
	Motor_ClearPosition(RF, NULL); delay_ms(1);
	Motor_PositionControl(LF,DIRECTION_POSITIVE,0,0,0,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(LB,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RF,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RB,DIRECTION_NEGATIVE,0,0,0,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_SyncStart();
	delay_ms(1);
	uint32_t t0 = HAL_GetTick();
	while (1) {
		int32_t lb, rf;
		Motor_Move_ReadPos(LB, &lb);
		Motor_Move_ReadPos(RF, &rf);
		if ((lb > lb_t ? lb - lb_t : lb_t - lb) < POS_RUN_REACH_DEADBAND &&
		    (rf > rf_t ? rf - rf_t : rf_t - rf) < POS_RUN_REACH_DEADBAND) break;
		if (HAL_GetTick() - t0 > POS_RUN_TIMEOUT_MS) break;
		delay_ms(100);
	}
}

void Pos_youxie(uint16_t acc,uint16_t maxspeed,uint32_t step){
	int32_t lf_t = STEP_TO_READBACK(step);
	int32_t rb_t = -STEP_TO_READBACK(step);
	Motor_ClearPosition(LF, NULL); delay_ms(1);
	Motor_ClearPosition(RB, NULL); delay_ms(1);
	Motor_PositionControl(LB,DIRECTION_NEGATIVE,0,0,0,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(LF,DIRECTION_POSITIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RB,DIRECTION_NEGATIVE,maxspeed,acc,step,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_PositionControl(RF,DIRECTION_POSITIVE,0,0,0,RELATIVE_POSITION,SNF_ENABLE);
	delay_ms(1);
	Motor_SyncStart();
	delay_ms(1);
	uint32_t t0 = HAL_GetTick();
	while (1) {
		int32_t lf, rb;
		Motor_Move_ReadPos(LF, &lf);
		Motor_Move_ReadPos(RB, &rb);
		if ((lf > lf_t ? lf - lf_t : lf_t - lf) < POS_RUN_REACH_DEADBAND &&
		    (rb > rb_t ? rb - rb_t : rb_t - rb) < POS_RUN_REACH_DEADBAND) break;
		if (HAL_GetTick() - t0 > POS_RUN_TIMEOUT_MS) break;
		delay_ms(100);
	}
}
// ����ת�亯��
void DifferentialTurn(uint16_t base_speed, uint8_t acceleration, uint32_t base_pulses, 
                     float turn_ratio, TurnDirection turn_dir) {
    
    // �����������ٶȲ�
    uint16_t inner_speed = (uint16_t)(base_speed * (1.0f - turn_ratio));
    uint16_t outer_speed = (uint16_t)(base_speed * (1.0f + turn_ratio));
    
    // ������������������
    uint32_t inner_pulses = (uint32_t)(base_pulses * (1.0f - turn_ratio));
    uint32_t outer_pulses = (uint32_t)(base_pulses * (1.0f + turn_ratio));
    
    if (turn_dir == TURN_LEFT) {
        // ��ת������Ϊ�ڲ࣬����Ϊ���
        
        // �������֣��ڲࣩ- �ϵ��ٶȣ���������
        Motor_PositionControl(MOTOR_FL, DIRECTION_POSITIVE, inner_speed, acceleration, inner_pulses, RELATIVE_POSITION, SNF_ENABLE);
		delay_ms(10);
        Motor_PositionControl(MOTOR_RL, DIRECTION_POSITIVE, inner_speed, acceleration, inner_pulses, RELATIVE_POSITION, SNF_ENABLE);
		delay_ms(10);
        // �������֣���ࣩ- �ϸ��ٶȣ��϶�����
        Motor_PositionControl(MOTOR_FR, DIRECTION_NEGATIVE, outer_speed, acceleration, outer_pulses, RELATIVE_POSITION, SNF_ENABLE);
		delay_ms(10);
        Motor_PositionControl(MOTOR_RR, DIRECTION_NEGATIVE, outer_speed, acceleration, outer_pulses, RELATIVE_POSITION, SNF_ENABLE);
		delay_ms(10);
		Motor_SyncStart();
		delay_ms(10);
    } 
    else { // TURN_RIGHT
        // ��ת������Ϊ�ڲ࣬����Ϊ���

        // �������֣��ڲࣩ- �ϵ��ٶȣ���������
        Motor_PositionControl(MOTOR_FR, DIRECTION_POSITIVE, inner_speed, acceleration, inner_pulses, RELATIVE_POSITION, SNF_ENABLE);
		delay_ms(10);
        Motor_PositionControl(MOTOR_RR, DIRECTION_POSITIVE, inner_speed, acceleration, inner_pulses, RELATIVE_POSITION, SNF_ENABLE);
		delay_ms(10);
        
        // �������֣���ࣩ- �ϸ��ٶȣ��϶�����
        Motor_PositionControl(MOTOR_FL, DIRECTION_NEGATIVE, outer_speed, acceleration, outer_pulses, RELATIVE_POSITION, SNF_ENABLE);
		delay_ms(10);
        Motor_PositionControl(MOTOR_RL, DIRECTION_NEGATIVE, outer_speed, acceleration, outer_pulses, RELATIVE_POSITION, SNF_ENABLE);
		delay_ms(10);
		Motor_SyncStart();
		delay_ms(10);
    }
}

void Stop_car(){
	Motor_Stop(LF,SNF_ENABLE);
	delay_ms(10);
	Motor_Stop(LB,SNF_ENABLE);
	delay_ms(10);
	Motor_Stop(RF,SNF_ENABLE);
	delay_ms(10);
	Motor_Stop(RB,SNF_ENABLE);
	delay_ms(10);
	Motor_SyncStart();
	delay_ms(10);
}



void ZAxis_Enable(void)
{
	Motor_Enable(Z_AXIS_MOTOR,MOTOR_ENABLE,SNF_DISABLE);
	delay_ms(10);
}

void ZAxis_Disable(void)
{
	Motor_Enable(Z_AXIS_MOTOR,MOTOR_DISABLE,SNF_DISABLE);
	delay_ms(10);
}

void ZAxis_MoveUp(uint8_t acceleration, uint16_t maxspeed, uint32_t pulses)
{
	Motor_PositionControl(Z_AXIS_MOTOR,DIRECTION_POSITIVE,maxspeed,acceleration,pulses,RELATIVE_POSITION,SNF_DISABLE);
	delay_ms(10);
}

void ZAxis_MoveDown(uint8_t acceleration, uint16_t maxspeed, uint32_t pulses)
{
	Motor_PositionControl(Z_AXIS_MOTOR,DIRECTION_NEGATIVE,maxspeed,acceleration,pulses,RELATIVE_POSITION,SNF_DISABLE);
	delay_ms(10);
}

void ZAxis_MoveRelative(int32_t pulses, uint8_t acceleration, uint16_t maxspeed)
{
	if (pulses > 0) {
		ZAxis_MoveUp(acceleration,maxspeed,(uint32_t)pulses);
	} else if (pulses < 0) {
		ZAxis_MoveDown(acceleration,maxspeed,(uint32_t)(-pulses));
	}
}

void ZAxis_MoveTo(int32_t position, uint8_t acceleration, uint16_t maxspeed)
{
	MotorDirection dir = DIRECTION_POSITIVE;
	uint32_t target_pulses;

	if (position < 0) {
		dir = DIRECTION_NEGATIVE;
		target_pulses = (uint32_t)(-position);
	} else {
		target_pulses = (uint32_t)position;
	}

	Motor_PositionControl(Z_AXIS_MOTOR,dir,maxspeed,acceleration,target_pulses,ABSOLUTE_POSITION,SNF_DISABLE);
	delay_ms(10);
}

void ZAxis_Stop(void)
{
	Motor_Stop(Z_AXIS_MOTOR,SNF_DISABLE);
	delay_ms(10);
}

HAL_StatusTypeDef ZAxis_ClearPosition(void)
{
	uint8_t state_code = 0U;

	return Motor_ClearPosition(Z_AXIS_MOTOR,&state_code);
}

HAL_StatusTypeDef ZAxis_RequestPositionUpdate(void)
{
	return Motor_RequestPositionUpdate(Z_AXIS_MOTOR);
}

HAL_StatusTypeDef ZAxis_ReadPosition(int32_t *position, float *angle)
{
	return Motor_ReadPosition(Z_AXIS_MOTOR,position,angle);
}

uint8_t ZAxis_IsLimitPressed(void)
{
	return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7) == GPIO_PIN_RESET) ? 1U : 0U;
}

HAL_StatusTypeDef ZAxis_Home(uint16_t speed, uint16_t slope, uint32_t timeout_ms)
{
	uint32_t start_tick = HAL_GetTick();
	uint8_t stable_count = 0U;

	if (Motor_IsComBusy() != 0U) {
		return HAL_BUSY;
	}

	if (speed == 0U) {
		speed = Z_AXIS_HOME_SPEED;
	}

	if (slope == 0U) {
		slope = Z_AXIS_HOME_SLOPE;
	}

	ZAxis_Enable();

	if (ZAxis_IsLimitPressed() == 0U) {
		Motor_SpeedControl(Z_AXIS_MOTOR, DIRECTION_POSITIVE, slope, speed, SNF_DISABLE);
		delay_ms(10);
	}

	while (stable_count < 5U) {
		if (ZAxis_IsLimitPressed() != 0U) {
			stable_count++;
		} else {
			stable_count = 0U;
		}

		if (timeout_ms > 0U && (HAL_GetTick() - start_tick) > timeout_ms) {
			ZAxis_Stop();
			return HAL_TIMEOUT;
		}

		delay_ms(2);
	}

	ZAxis_Stop();
	delay_ms(50);

	/* Some drivers do not reply to clear-position. Do not block the action group here. */
	(void)ZAxis_ClearPosition();
	return HAL_OK;
}

HAL_StatusTypeDef ZAxis_HomeDefault(void)
{
	return ZAxis_Home(Z_AXIS_HOME_SPEED, Z_AXIS_HOME_SLOPE, Z_AXIS_HOME_TIMEOUT_MS);
}

HAL_StatusTypeDef Motor_Move_ReadPos(uint8_t addr, int32_t *pos)
{
    uint32_t t0;
    uint8_t retry;
    HAL_StatusTypeDef s;

    if (pos == NULL) return HAL_ERROR;
    *pos = 0;

    for (retry = 0; retry < 3; retry++) {
        s = Motor_RequestPositionUpdate(addr);
        if (s != HAL_OK) { delay_ms(5); continue; }

        t0 = HAL_GetTick();
        while (Motor_IsComBusy()) {
            if (HAL_GetTick() - t0 > 50U) { s = HAL_TIMEOUT; break; }
        }
        if (s == HAL_TIMEOUT) continue;

        s = Motor_GetLastComStatus();
        if (s != HAL_OK) continue;

        s = Motor_ReadPosition(addr, pos, NULL);
        if (s == HAL_OK) return HAL_OK;
    }
    return s;
}
