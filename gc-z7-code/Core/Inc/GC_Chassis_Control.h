#ifndef GC_CHASSIS_CONTROL_H
#define GC_CHASSIS_CONTROL_H

#include "stm32f4xx_hal.h"
#include "PID.h"

typedef struct {
    PID_Cycle position_cycle;
    PID_Cycle angle_cycle;

    float max_speed;
    float max_turn_speed;
    float distance_deadband;
    float yaw_deadband;
    float position_integral_limit;
    float angle_integral_limit;
    float wheel_diameter_mm;
    float position_counts_per_rev;
    uint16_t vy_slope;

    float target_distance;
    float start_distance;
    float current_distance;
    float moved_distance;
    float target_yaw;
    float current_yaw;
    float distance_error;
    float yaw_error;
    float max_y_speed;
    float v_ref;
    float vy_target;
    float vy_ref;
    float w_ref;
    int32_t lf_position;
    int32_t lb_position;
    int32_t rf_position;
    int32_t rb_position;
    uint8_t enable;
    uint8_t finished;
    uint8_t error;
    HAL_StatusTypeDef last_status;
} CarStraight_Cascade;

typedef struct {
    PID_Cycle z_cycle;

    float max_turn_speed;
    float max_wheel_speed;
    float yaw_deadband;
    float z_integral_limit;
    uint8_t stable_count_limit;

    float turn_angle;
    float start_yaw;
    float target_yaw;
    float current_yaw;
    float yaw_error;
    float wz_ref;
    uint16_t stable_count;
    uint8_t enable;
    uint8_t finished;

    uint8_t stop_zero_pending;
} InPlaceTurn_Cascade;

typedef struct {
    PID_Cycle z_cycle;

    float max_turn_speed;
    float max_wheel_speed;
    float min_turn_speed;
    float yaw_deadband;
    float z_integral_limit;
    uint8_t stable_count_limit;

    float turn_angle;
    float start_yaw;
    float target_yaw;
    float current_yaw;
    float yaw_error;
    float wz_ref;
    uint16_t stable_count;
    uint8_t enable;
    uint8_t finished;
    uint8_t stop_zero_pending;
} InPlaceTurnMm_Cascade;

typedef struct {
    PID_Cycle x_cycle;
    PID_Cycle y_cycle;
    PID_Cycle z_cycle;

    float max_x_speed;
    float max_y_speed;
    float max_rotate_speed;
    float max_wheel_speed;
    float x_deadband;
    float y_deadband;
    float yaw_deadband;
    uint16_t stable_count_limit;
    uint16_t data_timeout_limit;

    volatile float x_error;
    volatile float y_error;
    volatile uint8_t data_valid;

    volatile uint32_t data_frame_counter;

    volatile uint16_t data_timeout_count;
    uint16_t stable_count;
    float target_yaw;
    float current_yaw;
    float yaw_error;
    float vx_ref;
    float vy_ref;
    float wz_ref;
    float lf_speed;
    float rf_speed;
    float lb_speed;
    float rb_speed;
    uint8_t enable;
    uint8_t finished;
    uint8_t stop_zero_pending;
} VisionPrecision_Cascade;

typedef struct {
    PID_Cycle x_cycle;
    PID_Cycle y_cycle;
    PID_Cycle z_cycle;

    float max_x_speed;
    float max_y_speed;
    float max_rotate_speed;
    float max_wheel_speed;
    float x_deadband;
    float y_deadband;
    float yaw_deadband;
    uint16_t stable_count_limit;

    float target_x;
    float target_y;
    float target_yaw;

    float start_x;
    float start_y;
    float current_x;
    float current_y;
    float moved_x;
    float moved_y;

    int32_t start_lf;
    int32_t start_lb;
    int32_t start_rf;
    int32_t start_rb;

    float y_encoder_mm_per_count;   /* TIM4 编码器 → mm 转换系数 */
    int32_t y_encoder_start;        /* 启动时编码器基准值 */

    float start_yaw;
    float current_yaw;
    float yaw_error;

    float vx_ref;
    float vy_ref;
    float wz_ref;

    uint16_t stable_count;
    uint8_t enable;
    uint8_t finished;
    uint8_t stop_zero_pending;
} FieldPos_Cascade;

extern CarStraight_Cascade car_straight;
extern InPlaceTurn_Cascade in_place_turn;
extern InPlaceTurnMm_Cascade in_place_turn_mm;
extern VisionPrecision_Cascade vision_precision;
extern FieldPos_Cascade field_pos;

extern uint16_t chassis_motor_slope;

void PID_Config_Init(void);

HAL_StatusTypeDef CarStraight_Start(CarStraight_Cascade *cs, float distance, float vy, float yaw);
void CarStraight_Control(CarStraight_Cascade *cs);
void CarStraight_Stop(CarStraight_Cascade *cs);

void InPlaceTurn_Start(InPlaceTurn_Cascade *it, float angle);
void InPlaceTurn_Control(InPlaceTurn_Cascade *it);
void InPlaceTurn_Stop(InPlaceTurn_Cascade *it);

void InPlaceTurnMm_Start(InPlaceTurnMm_Cascade *it, float angle);
void InPlaceTurnMm_Control(InPlaceTurnMm_Cascade *it);
void InPlaceTurnMm_Stop(InPlaceTurnMm_Cascade *it);
void InPlaceTurnMm_Block(InPlaceTurnMm_Cascade *it, float angle);

void VisionPrecision_Start(VisionPrecision_Cascade *vp, float target_yaw);
void VisionPrecision_Control(VisionPrecision_Cascade *vp);
void VisionPrecision_Stop(VisionPrecision_Cascade *vp);
void VisionPrecision_Update(VisionPrecision_Cascade *vp, float x, float y, unsigned char valid);
void VisionPrecision_Calibrate(void);

void FieldPos_Start(FieldPos_Cascade *fp, float dx, float dy, float dyaw);
void FieldPos_StartAbs(FieldPos_Cascade *fp, float dx, float dy, float yaw);
void FieldPos_Control(FieldPos_Cascade *fp);
void FieldPos_Stop(FieldPos_Cascade *fp);
void FieldPos_Block(FieldPos_Cascade *fp, float dx, float dy, float dyaw);
void FieldPos_BlockAbs(FieldPos_Cascade *fp, float dx, float dy, float yaw);

extern volatile uint32_t pit_1ms_pending;
extern volatile uint32_t system_count;

void pit_control(void);
void Mission_PitDelay(uint32_t delay_ms);
void InPlaceTurn_Block(InPlaceTurn_Cascade *it, float angle);
void CarStraight_Block(CarStraight_Cascade *cs, float distance, float vy, float yaw);

void VisionPrecision_Calibrate2(void);
#endif
