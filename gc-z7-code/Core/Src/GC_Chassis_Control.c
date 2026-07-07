#include "GC_Chassis_Control.h"
#include "Motor_Move.h"
#include "HWT101.h"
#include "serial.h"
#include "delay.h"
#include "bluetooth.h"
#include "screen.h"
#include "ServoMotorControl.h"
#include "encoder_f407.h"
#include "huaner_servo.h"
#include "action.h"

CarStraight_Cascade car_straight;
InPlaceTurn_Cascade in_place_turn;
InPlaceTurnMm_Cascade in_place_turn_mm;
VisionPrecision_Cascade vision_precision;
FieldPos_Cascade field_pos;

volatile uint32_t pit_1ms_pending = 0;
volatile uint32_t system_count = 0;

uint16_t chassis_motor_slope = 10U;

#define VISION_PRECISION_START_DELAY_MS      10U
#define VISION_PRECISION_CALIBRATE_TIMEOUT_MS 50000U

static uint8_t pid_config_done = 0U;
static float mm_per_count;

/* Tune with: new = old * measured_mm / target_mm. */
#define FIELD_POS_X_MM_CORRECTION 1.05f
#define FIELD_POS_Y_MM_CORRECTION 3.99f
#define ENABLE_VISION_Y_AXIS_PID 1U

static float NormalizeAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float PositionToMm(int32_t position)
{
    return (float)position * mm_per_count;
}

static HAL_StatusTypeDef RefreshAllPositions(void)
{
    static const uint8_t addr[4] = {LF, LB, RF, RB};
    for (int i = 0; i < 4; i++) {
        HAL_StatusTypeDef s = Motor_RequestPositionUpdate(addr[i]);
        if (s != HAL_OK) return s;
        uint32_t t0 = HAL_GetTick();
        while (Motor_IsComBusy()) {
            if (HAL_GetTick() - t0 > 50U) return HAL_TIMEOUT;
        }
        s = Motor_GetLastComStatus();
        if (s != HAL_OK) return s;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef ReadWheelPositions(int32_t *lf, int32_t *lb,
                                            int32_t *rf, int32_t *rb)
{
    HAL_StatusTypeDef s;
    if (lf == NULL || lb == NULL || rf == NULL || rb == NULL) return HAL_ERROR;

    s = Motor_ReadPosition(LF, lf, NULL);  if (s != HAL_OK) return s;  car_straight.lf_position = *lf;
    s = Motor_ReadPosition(LB, lb, NULL);  if (s != HAL_OK) return s;  car_straight.lb_position = *lb;
    s = Motor_ReadPosition(RF, rf, NULL);  if (s != HAL_OK) return s;  car_straight.rf_position = *rf;
    s = Motor_ReadPosition(RB, rb, NULL);  if (s != HAL_OK) return s;  car_straight.rb_position = *rb;
    return HAL_OK;
}

static HAL_StatusTypeDef ReadAverageDistance(float *dist)
{
    int32_t lf, lb, rf, rb;
    if (dist == NULL) return HAL_ERROR;
    HAL_StatusTypeDef s = ReadWheelPositions(&lf, &lb, &rf, &rb);
    if (s != HAL_OK) return s;
    *dist = (PositionToMm(lf) + PositionToMm(lb) - PositionToMm(rf) - PositionToMm(rb)) * 0.25f;
    return HAL_OK;
}

static void ChassisSetWheels(float lf, float rf, float lb, float rb, float max_spd)
{
    lf = LimitValue(lf, -max_spd, max_spd);
    rf = LimitValue(rf, -max_spd, max_spd);
    lb = LimitValue(lb, -max_spd, max_spd);
    rb = LimitValue(rb, -max_spd, max_spd);

    Motor_SpeedControl(LF, (lf >= 0) ? DIRECTION_POSITIVE : DIRECTION_NEGATIVE, chassis_motor_slope, (uint16_t)Abs(lf), SNF_ENABLE);
    delay_ms(1);
    Motor_SpeedControl(RF, (rf >= 0) ? DIRECTION_NEGATIVE : DIRECTION_POSITIVE, chassis_motor_slope, (uint16_t)Abs(rf), SNF_ENABLE);
    delay_ms(1);
    Motor_SpeedControl(LB, (lb >= 0) ? DIRECTION_POSITIVE : DIRECTION_NEGATIVE, chassis_motor_slope, (uint16_t)Abs(lb), SNF_ENABLE);
    delay_ms(1);
    Motor_SpeedControl(RB, (rb >= 0) ? DIRECTION_NEGATIVE : DIRECTION_POSITIVE, chassis_motor_slope, (uint16_t)Abs(rb), SNF_ENABLE);
    delay_ms(1);
    Motor_SyncStart(); delay_ms(1);
}

void PID_Config_Init(void)
{
    if (pid_config_done) return;
    pid_config_done = 1U;

    /* ============ CarStraight ============ */
    car_straight.max_speed                 = 200.0f;
    car_straight.max_turn_speed            = 300.0f;
    car_straight.distance_deadband         = 3.0f;
    car_straight.yaw_deadband              = 1.0f;
    car_straight.position_integral_limit   = 1000.0f;
    car_straight.angle_integral_limit      = 100.0f;
    car_straight.wheel_diameter_mm         = 75.0f;
    car_straight.position_counts_per_rev   = 65536.0f;
    car_straight.max_y_speed               = 300.0f;
    car_straight.vy_slope                  = 10;
    car_straight.last_status               = HAL_OK;
    chassis_motor_slope = 200U;
    
    mm_per_count = 3.1415926f * car_straight.wheel_diameter_mm
                 / car_straight.position_counts_per_rev;

    {
        PID_Cycle *c = &car_straight.position_cycle;
        c->p = 0.9f;  c->i = 0.0f;  c->d = 0.3f;
        c->out_max = car_straight.max_speed;
        c->i_max   = car_straight.position_integral_limit;
        c->deadband = car_straight.distance_deadband;

        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }
    {
        PID_Cycle *c = &car_straight.angle_cycle;
        c->p = 0.4f;  c->i = 0.0f;  c->d = 0.0f;
        c->out_max = car_straight.max_turn_speed;
        c->i_max   = car_straight.angle_integral_limit;
        c->deadband = car_straight.yaw_deadband;

        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }

    /* ============ InPlaceTurn ============ */
    in_place_turn.max_turn_speed    = 200.0f;
    in_place_turn.max_wheel_speed   = 200.0f;
    in_place_turn.yaw_deadband      = 0.5f;
    in_place_turn.z_integral_limit  = 100.0f;
    in_place_turn.stable_count_limit = 10U;

    {
        PID_Cycle *c = &in_place_turn.z_cycle;
        c->p = 7.0f;  c->i = 0.1f;  c->d = 0.0f;
        c->out_max = in_place_turn.max_turn_speed;
        c->i_max   = in_place_turn.z_integral_limit;
        c->deadband = in_place_turn.yaw_deadband;

        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }

    /* ============ InPlaceTurnMm ============ */
    in_place_turn_mm.max_turn_speed    = 200.0f;
    in_place_turn_mm.max_wheel_speed   = 200.0f;
    in_place_turn_mm.min_turn_speed    = 30.0f;
    in_place_turn_mm.yaw_deadband      = 0.5f;
    in_place_turn_mm.z_integral_limit  = 100.0f;
    in_place_turn_mm.stable_count_limit = 10U;

    {
        PID_Cycle *c = &in_place_turn_mm.z_cycle;
        c->p = 7.0f;  c->i = 0.1f;  c->d = 0.0f;
        c->out_max = in_place_turn_mm.max_turn_speed;
        c->i_max   = in_place_turn_mm.z_integral_limit;
        c->deadband = in_place_turn_mm.yaw_deadband;

        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }

    /* ============ VisionPrecision ============ */
    vision_precision.max_x_speed      = 300.0f;
    vision_precision.max_y_speed      = 300.0f;
    vision_precision.max_rotate_speed = 120.0f;
    vision_precision.max_wheel_speed  = 450.0f;
    vision_precision.x_deadband       = 2.0f;
    vision_precision.y_deadband       = 2.0f;
    vision_precision.yaw_deadband     = 0.5f;
    vision_precision.stable_count_limit = 10U;
    vision_precision.data_timeout_limit = 10U;

    {
        PID_Cycle *c = &vision_precision.x_cycle;
        c->p = 0.5f;  c->i = 0.01f;  c->d = 0.1f;
        c->out_max = vision_precision.max_x_speed;
        c->i_max   = vision_precision.max_x_speed;
        c->deadband = vision_precision.x_deadband;

        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }
    {
        PID_Cycle *c = &vision_precision.y_cycle;
        c->p = -0.8f;  c->i = -0.01f;  c->d = 0.0f;
        c->out_max = vision_precision.max_y_speed;
        c->i_max   = vision_precision.max_y_speed;
        c->deadband = vision_precision.y_deadband;

        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }
    {
        PID_Cycle *c = &vision_precision.z_cycle;
        c->p = 8.0f;  c->i = 0.01f;  c->d = 0.0f;
        c->out_max = vision_precision.max_rotate_speed;
        c->i_max   = vision_precision.max_rotate_speed;
        c->deadband = vision_precision.yaw_deadband;

        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }

    /* ============ FieldPos ============ */
    field_pos.max_x_speed        = 180.0f;
    field_pos.max_y_speed        = 180.0f;
    field_pos.max_rotate_speed   = 200.0f;
    field_pos.max_wheel_speed    = 350.0f;
    field_pos.x_deadband         = 3.0f;
    field_pos.y_deadband         = 2.0f;
    field_pos.yaw_deadband       = 0.5f;
    field_pos.stable_count_limit     = 10U;
    field_pos.y_encoder_mm_per_count = 0.0445f;  /* π×58mm / (1024线×4倍频) */

    {
        PID_Cycle *c = &field_pos.x_cycle;
        c->p = 1.0f;  c->i = 0.0f;  c->d = 3.0f;
        c->out_max = field_pos.max_x_speed;
        c->i_max   = field_pos.max_x_speed;
        c->deadband = field_pos.x_deadband;
        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }
    {
        PID_Cycle *c = &field_pos.y_cycle;
        c->p = 1.5f;  c->i = 0.02f;  c->d = 1.0f;
        c->out_max = field_pos.max_y_speed;
        c->i_max   = field_pos.max_y_speed;
        c->deadband = field_pos.y_deadband;
        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }
    {
        PID_Cycle *c = &field_pos.z_cycle;
        c->p = 2.0f;  c->i = 0.07f;  c->d = 1.0f;
        c->out_max = field_pos.max_rotate_speed;
        c->i_max   = field_pos.max_rotate_speed;
        c->deadband = field_pos.yaw_deadband;
        c->mode = PID_POSITION;
        PID_Cycle_Reset(c);
    }
}

static void DisableCarStraight(CarStraight_Cascade *cs)
{
    cs->enable = 0U;
    PID_Cycle_SetEnable(&cs->position_cycle, 0U);
    PID_Cycle_SetEnable(&cs->angle_cycle, 0U);
    cs->v_ref = 0.0f;
    cs->vy_ref = 0.0f;
    cs->vy_target = 0.0f;
    cs->w_ref = 0.0f;
}

static void DisableInPlaceTurn(InPlaceTurn_Cascade *it)
{
    it->enable = 0U;
    it->stop_zero_pending = 0U;
    PID_Cycle_SetEnable(&it->z_cycle, 0U);
}

static void DisableInPlaceTurnMm(InPlaceTurnMm_Cascade *it)
{
    it->enable = 0U;
    it->stop_zero_pending = 0U;
    PID_Cycle_SetEnable(&it->z_cycle, 0U);
}

static void DisableVisionPrecision(VisionPrecision_Cascade *vp)
{
    vp->enable = 0U;
    vp->stop_zero_pending = 0U;
    PID_Cycle_SetEnable(&vp->x_cycle, 0U);
    PID_Cycle_SetEnable(&vp->y_cycle, 0U);
    PID_Cycle_SetEnable(&vp->z_cycle, 0U);
    Vision_UART_StopStream();
}

static void DisableFieldPos(FieldPos_Cascade *fp)
{
    fp->enable = 0U;
    fp->stop_zero_pending = 0U;
    PID_Cycle_SetEnable(&fp->x_cycle, 0U);
    PID_Cycle_SetEnable(&fp->y_cycle, 0U);
    PID_Cycle_SetEnable(&fp->z_cycle, 0U);
}

/* ── 全场坐标: X 来自麦轮编码器, Y 来自 TIM4 外部编码器, Z 来自 IMU ── */
static HAL_StatusTypeDef ReadFieldPositions(FieldPos_Cascade *fp)
{
    if (fp == NULL) return HAL_ERROR;

    /* X 轴 — 麦轮电机编码器合成 */
    int32_t lf, lb, rf, rb;
    HAL_StatusTypeDef s = ReadWheelPositions(&lf, &lb, &rf, &rb);
    if (s != HAL_OK) return s;

    float dx = (PositionToMm(lf - fp->start_lf)
              - PositionToMm(rf - fp->start_rf)
              + PositionToMm(lb - fp->start_lb)
              - PositionToMm(rb - fp->start_rb)) * 0.25f;

    fp->moved_x = dx * FIELD_POS_X_MM_CORRECTION;
    fp->current_x = fp->start_x + fp->moved_x;

    /* Y 轴 — TIM4 外部编码器 */
    int32_t encoder_now = get_encoder_point()->total_count;
    fp->moved_y = -(float)(encoder_now - fp->y_encoder_start)
                * fp->y_encoder_mm_per_count
                * FIELD_POS_Y_MM_CORRECTION;
    fp->current_y = fp->start_y + fp->moved_y;

    return HAL_OK;
}

/* ============ CarStraight ============ */

HAL_StatusTypeDef CarStraight_Start(CarStraight_Cascade *cs, float distance, float vy, float yaw)
{
    PID_Config_Init();
    DisableInPlaceTurn(&in_place_turn);
    DisableInPlaceTurnMm(&in_place_turn_mm);
    DisableVisionPrecision(&vision_precision);
    DisableFieldPos(&field_pos);

    RefreshAllPositions();
    HAL_StatusTypeDef s = ReadAverageDistance(&cs->start_distance);

    if (s != HAL_OK && s != HAL_BUSY) {
        cs->enable = 1U;  cs->error = 1U;
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, car_straight.max_speed);
        return s;
    }

    cs->target_distance = distance;
    cs->vy_target        = LimitValue(vy, -cs->max_y_speed, cs->max_y_speed);
    cs->target_yaw      = NormalizeAngle(yaw);
    cs->current_distance = cs->start_distance;
    cs->current_yaw      = (float)global_angle;
    cs->moved_distance   = 0.0f;
    cs->distance_error   = distance;
    cs->yaw_error = NormalizeAngle(cs->target_yaw - cs->current_yaw);
    cs->v_ref = 0.0f;  cs->vy_ref = 0.0f;  cs->vy_target = 0.0f;  cs->w_ref = 0.0f;
    cs->finished = 0U;  cs->error = 0U;

    PID_Cycle_Reset(&cs->position_cycle);
    PID_Cycle_Reset(&cs->angle_cycle);
    PID_Cycle_SetEnable(&cs->position_cycle, 1U);
    PID_Cycle_SetEnable(&cs->angle_cycle, 1U);
    cs->enable = 1U;
    return HAL_OK;
}

void CarStraight_Control(CarStraight_Cascade *cs)
{
    if (cs->enable == 0U) return;
    if (Motor_IsComBusy() != 0U) { cs->last_status = HAL_BUSY; return; }

    HAL_StatusTypeDef s = RefreshAllPositions();
    if (s != HAL_OK) { cs->last_status = s; return; }

    s = ReadAverageDistance(&cs->current_distance);
    if (s != HAL_OK) {
        cs->enable = 0U;  cs->error = 1U;  cs->last_status = s;
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, car_straight.max_speed);
        return;
    }

    cs->current_yaw    = (float)global_angle;
    cs->moved_distance = cs->current_distance - cs->start_distance;
    cs->distance_error = cs->target_distance - cs->moved_distance;
    cs->yaw_error = NormalizeAngle(cs->target_yaw - cs->current_yaw);

    if (Abs(cs->distance_error) <= cs->distance_deadband
        && Abs(cs->yaw_error) <= cs->yaw_deadband) {
        CarStraight_Stop(cs);
        cs->finished = 1U;
        cs->last_status = HAL_OK;
        return;
    }

    cs->v_ref = PID_Control(&cs->position_cycle, cs->target_distance, cs->moved_distance);
    cs->w_ref = PID_Control(&cs->angle_cycle, cs->yaw_error, 0.0f);
    if (cs->vy_ref < cs->vy_target) {
        cs->vy_ref += (float)cs->vy_slope;
        if (cs->vy_ref > cs->vy_target) cs->vy_ref = cs->vy_target;
    } else if (cs->vy_ref > cs->vy_target) {
        cs->vy_ref -= (float)cs->vy_slope;
        if (cs->vy_ref < cs->vy_target) cs->vy_ref = cs->vy_target;
    }
    ChassisSetWheels(cs->v_ref - cs->vy_ref - cs->w_ref,
                     cs->v_ref + cs->vy_ref + cs->w_ref,
                     cs->v_ref + cs->vy_ref - cs->w_ref,
                     cs->v_ref - cs->vy_ref + cs->w_ref,
                     car_straight.max_speed);
    cs->last_status = HAL_OK;
}

void CarStraight_Stop(CarStraight_Cascade *cs)
{
    DisableCarStraight(cs);
    ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, car_straight.max_speed);
}


/* ============ InPlaceTurn ============ */

void InPlaceTurn_Start(InPlaceTurn_Cascade *it, float angle)
{
    PID_Config_Init();
    DisableCarStraight(&car_straight);
    DisableInPlaceTurnMm(&in_place_turn_mm);
    DisableVisionPrecision(&vision_precision);
    DisableFieldPos(&field_pos);

    it->turn_angle  = angle;
    it->start_yaw   = (float)global_angle;
    it->target_yaw  = NormalizeAngle(it->start_yaw + angle);
    it->current_yaw = it->start_yaw;
    it->yaw_error   = NormalizeAngle(it->target_yaw - it->current_yaw);
    it->wz_ref      = 0.0f;
    it->stable_count = 0U;
    it->finished    = 0U;
    it->stop_zero_pending = 0U;

    PID_Cycle_Reset(&it->z_cycle);
    PID_Cycle_SetEnable(&it->z_cycle, 1U);
    it->enable = 1U;
}

void InPlaceTurn_Control(InPlaceTurn_Cascade *it)
{
    if (it->enable == 0U) {
        if (it->stop_zero_pending && Motor_IsComBusy() == 0U) {
            ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, it->max_turn_speed);
            it->stop_zero_pending = 0U;
        }
        return;
    }
    if (Motor_IsComBusy() != 0U) return;

    it->current_yaw = (float)global_angle;
    it->yaw_error = NormalizeAngle(it->target_yaw - it->current_yaw);

    it->wz_ref = PID_Control(&it->z_cycle, it->yaw_error, 0.0f);
    it->wz_ref = LimitValue(it->wz_ref, -it->max_turn_speed, it->max_turn_speed);

    if (Abs(it->yaw_error) < it->yaw_deadband) it->stable_count++;
    else                                       it->stable_count = 0U;

    if (it->stable_count >= it->stable_count_limit) {
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, it->max_turn_speed);
        it->stop_zero_pending = 0U;
        it->wz_ref   = 0.0f;
        it->finished = 1U;
        it->enable   = 0U;
        PID_Cycle_SetEnable(&it->z_cycle, 0U);
        return;
    }

    ChassisSetWheels(-it->wz_ref, it->wz_ref, -it->wz_ref, it->wz_ref, it->max_turn_speed);
}

void InPlaceTurn_Stop(InPlaceTurn_Cascade *it)
{
    DisableInPlaceTurn(it);
    it->stop_zero_pending = 1U;
    if (Motor_IsComBusy() == 0U) {
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, it->max_turn_speed);
        it->stop_zero_pending = 0U;
    }
}


/* ============ VisionPrecision ============ */

void VisionPrecision_Start(VisionPrecision_Cascade *vp, float target_yaw)
{
    PID_Config_Init();
    DisableCarStraight(&car_straight);
    DisableInPlaceTurn(&in_place_turn);
    DisableInPlaceTurnMm(&in_place_turn_mm);
    DisableFieldPos(&field_pos);
    if (Motor_IsComBusy() == 0U) ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, car_straight.max_speed);

    vp->target_yaw    = NormalizeAngle(target_yaw);
    vp->current_yaw   = (float)global_angle;
    vp->yaw_error     = NormalizeAngle(vp->target_yaw - vp->current_yaw);
    vp->vx_ref = 0.0f;  vp->vy_ref = 0.0f;  vp->wz_ref = 0.0f;
    vp->x_error = 0.0f;
    vp->y_error = 0.0f;
    vp->data_valid = 0U;
    vp->data_frame_counter = 0U;
    vp->stable_count      = 0U;
    vp->data_timeout_count = 0U;
    vp->finished   = 0U;
    vp->stop_zero_pending = 0U;

    PID_Cycle_Reset(&vp->x_cycle);
    PID_Cycle_Reset(&vp->y_cycle);
    PID_Cycle_Reset(&vp->z_cycle);
    PID_Cycle_SetEnable(&vp->x_cycle, 1U);
#if ENABLE_VISION_Y_AXIS_PID
    PID_Cycle_SetEnable(&vp->y_cycle, 1U);
#else
    PID_Cycle_SetEnable(&vp->y_cycle, 0U);
#endif
    PID_Cycle_SetEnable(&vp->z_cycle, 1U);

    Vision_UART_StopStream();
    delay_ms(VISION_PRECISION_START_DELAY_MS);
    Vision_UART_StartCircle();
    vp->enable = 1U;
}

void VisionPrecision_Control(VisionPrecision_Cascade *vp)
{
    if (vp->enable == 0U) {
        if (vp->stop_zero_pending && Motor_IsComBusy() == 0U) {
            ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, vp->max_wheel_speed);
            vp->stop_zero_pending = 0U;
        }
        return;
    }
    if (Motor_IsComBusy() != 0U) return;

    if (vp->data_timeout_count < 65535U) vp->data_timeout_count++;

    if (vp->data_valid == 0U || vp->data_timeout_count > vp->data_timeout_limit) {
        vp->vx_ref = 0.0f;  vp->vy_ref = 0.0f;  vp->wz_ref = 0.0f;
        vp->stable_count = 0U;
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, vp->max_wheel_speed);
        return;
    }

    vp->current_yaw = (float)global_angle;
    vp->yaw_error = NormalizeAngle(vp->target_yaw - vp->current_yaw);

    vp->vx_ref = PID_Control(&vp->x_cycle, vp->x_error, 0.0f);
#if ENABLE_VISION_Y_AXIS_PID
    vp->vy_ref = PID_Control(&vp->y_cycle, vp->y_error, 0.0f);
#else
    vp->vy_ref = 0.0f;
#endif
    vp->wz_ref = PID_Control(&vp->z_cycle, vp->yaw_error, 0.0f);
    vp->vx_ref = LimitValue(vp->vx_ref, -vp->max_x_speed, vp->max_x_speed);
#if ENABLE_VISION_Y_AXIS_PID
    vp->vy_ref = LimitValue(vp->vy_ref, -vp->max_y_speed, vp->max_y_speed);
#endif
    vp->wz_ref = LimitValue(vp->wz_ref, -vp->max_rotate_speed, vp->max_rotate_speed);

    float lf = vp->vx_ref - vp->vy_ref - vp->wz_ref;
    float rf = vp->vx_ref + vp->vy_ref + vp->wz_ref;
    float lb = vp->vx_ref + vp->vy_ref - vp->wz_ref;
    float rb = vp->vx_ref - vp->vy_ref + vp->wz_ref;

#if ENABLE_VISION_Y_AXIS_PID
    if (Abs(vp->x_error) < vp->x_deadband && Abs(vp->y_error) < vp->y_deadband
        && Abs(vp->yaw_error) < vp->yaw_deadband)
#else
    if (Abs(vp->x_error) < vp->x_deadband
        && Abs(vp->yaw_error) < vp->yaw_deadband)
#endif
        vp->stable_count++;
    else
        vp->stable_count = 0U;

    if (vp->stable_count >= vp->stable_count_limit) {
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, vp->max_wheel_speed);
        vp->stop_zero_pending = 0U;
        vp->finished = 1U;
        DisableVisionPrecision(vp);
        return;
    }
    ChassisSetWheels(lf, rf, lb, rb, vp->max_wheel_speed);
}

void VisionPrecision_Stop(VisionPrecision_Cascade *vp)
{
    vp->stop_zero_pending = 1U;
    DisableVisionPrecision(vp);
    if (Motor_IsComBusy() == 0U) {
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, vp->max_wheel_speed);
        vp->stop_zero_pending = 0U;
    }
}

void VisionPrecision_Calibrate(void)
{
    uint32_t start_tick;

		zhuafang7();
    VisionPrecision_Start(&vision_precision, (float)global_angle);
    start_tick = HAL_GetTick();
    while (!vision_precision.finished) {
        pit_control();
        if ((uint32_t)(HAL_GetTick() - start_tick) >= VISION_PRECISION_CALIBRATE_TIMEOUT_MS) {
            VisionPrecision_Stop(&vision_precision);
            vision_precision.finished = 1U;
            break;
        }
    }
}

void VisionPrecision_Calibrate2(void)
{
    uint32_t start_tick;

		zhuafang7_2();
    VisionPrecision_Start(&vision_precision, (float)global_angle);
    start_tick = HAL_GetTick();
    while (!vision_precision.finished) {
        pit_control();
        if ((uint32_t)(HAL_GetTick() - start_tick) >= VISION_PRECISION_CALIBRATE_TIMEOUT_MS) {
            VisionPrecision_Stop(&vision_precision);
            vision_precision.finished = 1U;
            break;
        }
    }
}
void VisionPrecision_Update(VisionPrecision_Cascade *vp, float x, float y, uint8_t valid)
{
    vp->x_error = x;
    vp->y_error = y;
    vp->data_valid = valid ? 1U : 0U;
    if (vp->data_valid) vp->data_timeout_count = 0U;
    vp->data_frame_counter++;
}

/* ============ PIT Control Loop ============ */

void pit_control(void)
{
    static uint8_t pid_divider = 0;
    static uint32_t last_bt_ms = 0;

    while (pit_1ms_pending > 0U) {
        __disable_irq();
        if (pit_1ms_pending == 0U) {
            __enable_irq();
            break;
        }
        pit_1ms_pending--;
        __enable_irq();

        system_count++;
        pid_divider++;
        Encoder_task();
				
        if ((uint32_t)(system_count - last_bt_ms) >= 500U) {
            last_bt_ms = system_count;
            Bluetooth_SendAngle((float)global_angle);
            //(void)Screen_SendGyroAngle((float)global_angle);
        }
				
				
        if (pid_divider >= 20U) {
            pid_divider = 0U;
            InPlaceTurn_Control(&in_place_turn);
            InPlaceTurnMm_Control(&in_place_turn_mm);
            CarStraight_Control(&car_straight);
            VisionPrecision_Control(&vision_precision);
            FieldPos_Control(&field_pos);
            ServoMotorVision_Control(&servo_motor_vision);
        }
    }
																	
}

void Mission_PitDelay(uint32_t delay_ms)
{
    uint32_t start_count;

    if (delay_ms == 0U) {
        return;
    }

    start_count = system_count;
    while ((uint32_t)(system_count - start_count) < delay_ms) {
        pit_control();
    }
}

void InPlaceTurn_Block(InPlaceTurn_Cascade *it, float angle)
{
    InPlaceTurn_Start(it, angle);
    while (!it->finished) {
        pit_control();
    }
}

void CarStraight_Block(CarStraight_Cascade *cs, float distance, float vy, float yaw)
{
    CarStraight_Start(cs, distance, vy, yaw);
    while (!cs->finished) {
        pit_control();
    }
}

/* ============ InPlaceTurnMm ============ */

void InPlaceTurnMm_Start(InPlaceTurnMm_Cascade *it, float angle)
{
    PID_Config_Init();
    DisableCarStraight(&car_straight);
    DisableInPlaceTurn(&in_place_turn);
    DisableVisionPrecision(&vision_precision);
    DisableFieldPos(&field_pos);

    it->turn_angle  = angle;
    it->start_yaw   = (float)global_angle;
    it->target_yaw  = NormalizeAngle(it->start_yaw + angle);
    it->current_yaw = it->start_yaw;
    it->yaw_error   = NormalizeAngle(it->target_yaw - it->current_yaw);
    it->wz_ref      = 0.0f;
    it->stable_count = 0U;
    it->finished    = 0U;
    it->stop_zero_pending = 0U;

    PID_Cycle_Reset(&it->z_cycle);
    PID_Cycle_SetEnable(&it->z_cycle, 1U);
    it->enable = 1U;
}

void InPlaceTurnMm_Control(InPlaceTurnMm_Cascade *it)
{
    if (it->enable == 0U) {
        if (it->stop_zero_pending && Motor_IsComBusy() == 0U) {
            ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, it->max_turn_speed);
            it->stop_zero_pending = 0U;
        }
        return;
    }
    if (Motor_IsComBusy() != 0U) return;

    it->current_yaw = (float)global_angle;
    it->yaw_error = NormalizeAngle(it->target_yaw - it->current_yaw);

    it->wz_ref = PID_Control(&it->z_cycle, it->yaw_error, 0.0f);
    it->wz_ref = LimitValue(it->wz_ref, -it->max_turn_speed, it->max_turn_speed);

    if (Abs(it->wz_ref) < it->min_turn_speed && Abs(it->yaw_error) > it->yaw_deadband) {
        it->wz_ref = (it->wz_ref >= 0.0f) ? it->min_turn_speed : -it->min_turn_speed;
    }

    if (Abs(it->yaw_error) < it->yaw_deadband) it->stable_count++;
    else                                       it->stable_count = 0U;

    if (it->stable_count >= it->stable_count_limit) {
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, it->max_turn_speed);
        it->stop_zero_pending = 0U;
        it->wz_ref   = 0.0f;
        it->finished = 1U;
        it->enable   = 0U;
        PID_Cycle_SetEnable(&it->z_cycle, 0U);
        return;
    }

    ChassisSetWheels(-it->wz_ref, it->wz_ref, -it->wz_ref, it->wz_ref, it->max_turn_speed);
}

void InPlaceTurnMm_Stop(InPlaceTurnMm_Cascade *it)
{
    DisableInPlaceTurnMm(it);
    it->stop_zero_pending = 1U;
    if (Motor_IsComBusy() == 0U) {
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, it->max_turn_speed);
        it->stop_zero_pending = 0U;
    }
}

void InPlaceTurnMm_Block(InPlaceTurnMm_Cascade *it, float angle)
{
    InPlaceTurnMm_Start(it, angle);
    while (!it->finished) {
        pit_control();
    }
}

/* ============ FieldPos ============ */

void FieldPos_Start(FieldPos_Cascade *fp, float dx, float dy, float dyaw)
{
    PID_Config_Init();
    DisableCarStraight(&car_straight);
    DisableInPlaceTurn(&in_place_turn);
    DisableInPlaceTurnMm(&in_place_turn_mm);
    DisableVisionPrecision(&vision_precision);

    Encoder_Reset();
    RefreshAllPositions();

    /* 记录起始轮位置 */
    int32_t lf, lb, rf, rb;
    ReadWheelPositions(&lf, &lb, &rf, &rb);
    fp->start_lf = lf;
    fp->start_lb = lb;
    fp->start_rf = rf;
    fp->start_rb = rb;

    fp->start_x    = 0.0f;
    fp->start_y    = 0.0f;
    fp->y_encoder_start = 0;
    fp->current_x  = 0.0f;
    fp->current_y  = 0.0f;
    fp->moved_x    = 0.0f;
    fp->moved_y    = 0.0f;

    fp->target_x   = dx;
    fp->target_y   = dy;

    fp->start_yaw  = (float)global_angle;
    fp->target_yaw = NormalizeAngle(fp->start_yaw + dyaw);
    fp->current_yaw = fp->start_yaw;
    fp->yaw_error  = NormalizeAngle(fp->target_yaw - fp->current_yaw);

    fp->vx_ref = 0.0f;
    fp->vy_ref = 0.0f;
    fp->wz_ref = 0.0f;
    fp->stable_count = 0U;
    fp->finished = 0U;
    fp->stop_zero_pending = 0U;

    PID_Cycle_Reset(&fp->x_cycle);
    PID_Cycle_Reset(&fp->y_cycle);
    PID_Cycle_Reset(&fp->z_cycle);
    PID_Cycle_SetEnable(&fp->x_cycle, 1U);
    PID_Cycle_SetEnable(&fp->y_cycle, 1U);
    PID_Cycle_SetEnable(&fp->z_cycle, 1U);
    fp->enable = 1U;
}

void FieldPos_StartAbs(FieldPos_Cascade *fp, float dx, float dy, float yaw)
{
    PID_Config_Init();
    DisableCarStraight(&car_straight);
    DisableInPlaceTurn(&in_place_turn);
    DisableInPlaceTurnMm(&in_place_turn_mm);
    DisableVisionPrecision(&vision_precision);

    Encoder_Reset();
    RefreshAllPositions();

    /* 记录起始轮位置 */
    int32_t lf, lb, rf, rb;
    ReadWheelPositions(&lf, &lb, &rf, &rb);
    fp->start_lf = lf;
    fp->start_lb = lb;
    fp->start_rf = rf;
    fp->start_rb = rb;

    fp->start_x    = 0.0f;
    fp->start_y    = 0.0f;
    fp->y_encoder_start = 0;
    fp->current_x  = 0.0f;
    fp->current_y  = 0.0f;
    fp->moved_x    = 0.0f;
    fp->moved_y    = 0.0f;

    fp->target_x   = dx;
    fp->target_y   = dy;

    fp->start_yaw  = (float)global_angle;
    fp->target_yaw = NormalizeAngle(yaw);
    fp->current_yaw = fp->start_yaw;
    fp->yaw_error  = NormalizeAngle(fp->target_yaw - fp->current_yaw);

    fp->vx_ref = 0.0f;
    fp->vy_ref = 0.0f;
    fp->wz_ref = 0.0f;
    fp->stable_count = 0U;
    fp->finished = 0U;
    fp->stop_zero_pending = 0U;

    PID_Cycle_Reset(&fp->x_cycle);
    PID_Cycle_Reset(&fp->y_cycle);
    PID_Cycle_Reset(&fp->z_cycle);
    PID_Cycle_SetEnable(&fp->x_cycle, 1U);
    PID_Cycle_SetEnable(&fp->y_cycle, 1U);
    PID_Cycle_SetEnable(&fp->z_cycle, 1U);
    fp->enable = 1U;
}

void FieldPos_Control(FieldPos_Cascade *fp)
{
    if (fp->enable == 0U) {
        if (fp->stop_zero_pending && Motor_IsComBusy() == 0U) {
            ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, fp->max_wheel_speed);
            fp->stop_zero_pending = 0U;
        }
        return;
    }
    if (Motor_IsComBusy() != 0U) return;

    HAL_StatusTypeDef s = RefreshAllPositions();
    if (s != HAL_OK) return;

    s = ReadFieldPositions(fp);
    if (s != HAL_OK) {
        DisableFieldPos(fp);
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, fp->max_wheel_speed);
        return;
    }

    fp->current_yaw = (float)global_angle;
    fp->yaw_error   = NormalizeAngle(fp->target_yaw - fp->current_yaw);

    /* 3 路独立 PID */
    fp->vx_ref = PID_Control(&fp->x_cycle, fp->target_x, fp->moved_x);
    fp->vy_ref = PID_Control(&fp->y_cycle, fp->target_y, fp->moved_y);
    fp->wz_ref = PID_Control(&fp->z_cycle, fp->yaw_error, 0.0f);

    fp->vx_ref = LimitValue(fp->vx_ref, -fp->max_x_speed, fp->max_x_speed);
    fp->vy_ref = LimitValue(fp->vy_ref, -fp->max_y_speed, fp->max_y_speed);
    fp->wz_ref = LimitValue(fp->wz_ref, -fp->max_rotate_speed, fp->max_rotate_speed);

    /* 稳定判定 */
    if (Abs(fp->target_x - fp->moved_x) < fp->x_deadband &&
        Abs(fp->target_y - fp->moved_y) < fp->y_deadband &&
        Abs(fp->yaw_error)               < fp->yaw_deadband) {
        fp->stable_count++;
    } else {
        fp->stable_count = 0U;
    }

    if (fp->stable_count >= fp->stable_count_limit) {
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, fp->max_wheel_speed);
        fp->stop_zero_pending = 0U;
        fp->finished = 1U;
        DisableFieldPos(fp);
        return;
    }

    /* Mecanum 轮速度分配 */
    ChassisSetWheels(fp->vx_ref - fp->vy_ref - fp->wz_ref,
                     fp->vx_ref + fp->vy_ref + fp->wz_ref,
                     fp->vx_ref + fp->vy_ref - fp->wz_ref,
                     fp->vx_ref - fp->vy_ref + fp->wz_ref,
                     fp->max_wheel_speed);
}

void FieldPos_Stop(FieldPos_Cascade *fp)
{
    fp->stop_zero_pending = 1U;
    DisableFieldPos(fp);
    if (Motor_IsComBusy() == 0U) {
        ChassisSetWheels(0.0f, 0.0f, 0.0f, 0.0f, fp->max_wheel_speed);
        fp->stop_zero_pending = 0U;
    }
}

void FieldPos_Block(FieldPos_Cascade *fp, float dx, float dy, float dyaw)
{
    FieldPos_Start(fp, dx, dy, dyaw);
    while (!fp->finished) {
        pit_control();
    }
}

void FieldPos_BlockAbs(FieldPos_Cascade *fp, float dx, float dy, float yaw)
{
    FieldPos_StartAbs(fp, dx, dy, yaw);
    while (!fp->finished) {
        pit_control();
    }
}
