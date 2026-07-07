#ifndef SERVOMOTORCONTROL_H
#define SERVOMOTORCONTROL_H

#include "stm32f4xx_hal.h"
#include "PID.h"

/**
 * @brief 视觉闭环 — 舵机0(X轴) + 舵机13(Y轴) 双路独立PID控制
 * 
 * X轴:  0号Lobot 180°位置舵机，PWM=500~2500控角 (moveServo)
 * Y轴:  13号Lobot 360°连续旋转舵机，PWM=500~2500控速 (servo13_SetSpeed)
 */
typedef struct {
    PID_Cycle x_cycle;              /* X轴PID (舵机0) */
    PID_Cycle y_cycle;              /* Y轴PID (舵机13) */

    float max_x_step;               /* 舵机0单次PWM修正最大步长 */
    float max_y_speed;              /* 舵机13最大速度偏移 (±max 对应PWM 1500±speed) */
    float x_deadband;               /* X轴像素死区 */
    float y_deadband;               /* Y轴像素死区 */
    float y_speed_deadband;         /* Y轴速度死区 (|vy_ref|<此值算到位, PWM=1500±speed) */
    uint16_t stable_count_limit;    /* 稳定判定次数 */
    uint16_t data_timeout_limit;    /* 数据超时判定次数 */

    volatile float x_error;         /* 视觉X误差 (外部写入，原始值) */
    volatile float y_error;         /* 视觉Y误差 (外部写入，原始值) */
    float x_pred;                   /* Kalman预测X误差 (PID使用) */
    float y_pred;                   /* Kalman预测Y误差 (PID使用) */
    volatile uint8_t data_valid;    /* 视觉数据有效标志 */
    volatile uint32_t data_frame_counter;
    volatile uint16_t data_timeout_count;

    uint16_t stable_count;
    float vx_ref;                   /* X轴PID输出 → 舵机0 PWM修正步长 */
    float vy_ref;                   /* Y轴PID输出 → 舵机13速度偏移 */

    uint16_t servo0_target_pwm;     /* 舵机0当前目标PWM */
    uint32_t last_move_tick;        /* 上次发送moveServo命令的时刻 (限流) */

    /* Kalman 滤波器状态 (1D位置+速度) */
    float kf_x, kf_y;              /* 滤波后位置 */
    float kf_vx, kf_vy;            /* 估计速度 (像素/秒) */
    float kf_px, kf_py;            /* 估计协方差 */
    float kf_q;                    /* 过程噪声 (tunable, 推荐0.5~5.0) */
    float kf_r;                    /* 测量噪声 (tunable, 推荐5.0~50.0) */
    float kf_predict_ms;           /* 预测超前时间 (ms, 推荐80~200) */
    uint8_t kf_initialized;
    uint32_t kf_last_tick;

    uint8_t enable;
    uint8_t finished;
} ServoMotorVision_Cascade;

extern ServoMotorVision_Cascade servo_motor_vision;

/* ──── 参数初始化 ──── */
void ServoMotorVision_ConfigInit(void);

/* ──── 生命周期 ──── */
void ServoMotorVision_Start(ServoMotorVision_Cascade *smv);
void ServoMotorVision_Control(ServoMotorVision_Cascade *smv);
void ServoMotorVision_Stop(ServoMotorVision_Cascade *smv);
void ServoMotorVision_MoveServo0(uint16_t pwm, uint16_t time_ms);

/* ──── 视觉数据注入 (外部中断/串口回调中调用) ──── */
void ServoMotorVision_Update(ServoMotorVision_Cascade *smv,
                             float x, float y, uint8_t valid);

/* ──── 阻塞接口 ──── */
void ServoMotorVision_Block(ServoMotorVision_Cascade *smv);

/* ──── 一键视觉校准 (阻塞，全自动) ──── */
void ServoMotorVision_Calibrate(uint8_t color_task);
void ServoMotorVision_Calibrate_Pos(uint8_t color_task);

#endif /* SERVOMOTORCONTROL_H */
