#include "ServoMotorControl.h"
#include "LobotServoController.h"
#include "Motor_Move.h"
#include "delay.h"
#include "serial.h"
#include "huaner_servo.h"
ServoMotorVision_Cascade servo_motor_vision;

static uint8_t config_inited = 0U;

/* ─── 限流参数 ─── */
#define ENABLE_X_AXIS_PID       1U      /* 1: tune servo 0 X axis */
#define ENABLE_Y_AXIS_PID       1U      /* 0: tune X axis only, keep servo 13 stopped */
#define SERVO0_CMD_INTERVAL_MS   50U    /* moveServo 最小发送间隔 (ms) */
#define SERVO0_MIN_PWM           500U
#define SERVO0_MAX_PWM           2500U
#define SERVO0_SEARCH_MIN_PWM    1860U
#define SERVO0_SEARCH_MAX_PWM    2200U
#define SERVO0_SEARCH_STEP_PWM   20U
#define SERVO0_SEARCH_MOVE_MS    20U
#define SERVO0_SEARCH_DWELL_MS   120U
#define SERVO0_SEARCH_VALID_FRAMES 1U
#define SERVO0_SEARCH_START_SETTLE_MS 300U

/* ============ 参数初始化 ============ */

void ServoMotorVision_ConfigInit(void)
{
    if (config_inited) return;
    config_inited = 1U;

    servo_motor_vision.max_x_step          = 100.0f;
    servo_motor_vision.max_y_speed          = 1000.0f;
    servo_motor_vision.x_deadband           = 40.0f;
    servo_motor_vision.y_deadband           = 10.0f;
    servo_motor_vision.y_speed_deadband     = 200.0f;   /* |vy_ref|<100 即PWM在1500±100 */
    servo_motor_vision.stable_count_limit   = 10U;
    servo_motor_vision.data_timeout_limit   = 50U;
    servo_motor_vision.last_move_tick       = 0U;

    /* X轴PID — 舵机0 (180°位置控制) */
    {
        PID_Cycle *c = &servo_motor_vision.x_cycle;
        c->p       = 0.2f;
        c->i       = 0.0f;
        c->d       = 0.4f;
        c->out_max = servo_motor_vision.max_x_step;
        c->i_max   = servo_motor_vision.max_x_step;
        c->deadband = servo_motor_vision.x_deadband;
        c->mode    = PID_POSITION;
        PID_Cycle_Reset(c);
    }

    /* Y轴PID — 舵机13 (360°速度控制) */
    {
        PID_Cycle *c = &servo_motor_vision.y_cycle;
        c->p       = 10.0f;
        c->i       = 0.3f;
        c->d       = 61.0f;
        c->out_max = servo_motor_vision.max_y_speed;
        c->i_max   = servo_motor_vision.max_y_speed;
        c->deadband = servo_motor_vision.y_deadband;
        c->mode    = PID_POSITION;
        PID_Cycle_Reset(c);
    }
}

/* ============ 内部工具 ============ */

static uint16_t Servo0_ClampPwm(uint16_t pwm)
{
    if (pwm < SERVO0_MIN_PWM) return SERVO0_MIN_PWM;
    if (pwm > SERVO0_MAX_PWM) return SERVO0_MAX_PWM;
    return pwm;
}

void ServoMotorVision_MoveServo0(uint16_t pwm, uint16_t time_ms)
{
    uint16_t target = Servo0_ClampPwm(pwm);

    servo_motor_vision.servo0_target_pwm = target;
    moveServo(0, target, time_ms);
}

static void StopActuators(void)
{
    /* X轴: 舵机0 — 保持当前位置不动，不发新指令 */
    /* Y轴: 停舵机13 */
    servo13_SetSpeed(0);
}

static uint8_t ServoMotorVision_WaitForTarget(uint32_t wait_ms,
                                             uint32_t *last_frame,
                                             uint8_t *valid_count)
{
    extern void pit_control(void);
    uint32_t start = HAL_GetTick();

    if (last_frame == NULL || valid_count == NULL) {
        return 0U;
    }

    while ((uint32_t)(HAL_GetTick() - start) < wait_ms) {
        pit_control();
        if (servo_motor_vision.data_frame_counter != *last_frame) {
            *last_frame = servo_motor_vision.data_frame_counter;
            if (servo_motor_vision.data_valid) {
                if (*valid_count < SERVO0_SEARCH_VALID_FRAMES) {
                    (*valid_count)++;
                }
                if (*valid_count >= SERVO0_SEARCH_VALID_FRAMES) {
                    return 1U;
                }
            } else {
                *valid_count = 0U;
            }
        }
    }

    return 0U;
}

static void ServoMotorVision_SearchTarget(uint8_t color_task)
{
    uint16_t pwm = servo_motor_vision.servo0_target_pwm;
    int8_t direction = 1;
    uint32_t last_frame = 0U;
    uint8_t valid_count = 0U;

    if (pwm < SERVO0_SEARCH_MIN_PWM) {
        pwm = SERVO0_SEARCH_MIN_PWM;
    } else if (pwm > SERVO0_SEARCH_MAX_PWM) {
        pwm = SERVO0_SEARCH_MAX_PWM;
    }
    if (pwm >= (uint16_t)(SERVO0_SEARCH_MAX_PWM - SERVO0_SEARCH_STEP_PWM)) {
        direction = -1;
    }

    servo_motor_vision.enable = 0U;
    servo_motor_vision.finished = 1U;
    servo_motor_vision.vx_ref = 0.0f;
    servo_motor_vision.vy_ref = 0.0f;
    servo_motor_vision.data_valid = 0U;
    servo_motor_vision.data_frame_counter = 0U;
    servo_motor_vision.data_timeout_count = 0U;
    PID_Cycle_SetEnable(&servo_motor_vision.x_cycle, 0U);
    PID_Cycle_SetEnable(&servo_motor_vision.y_cycle, 0U);
    Vision_UART_StopStream();
    delay_ms(10);

    ServoMotorVision_MoveServo0(pwm, SERVO0_SEARCH_MOVE_MS);
    delay_ms(SERVO0_SEARCH_START_SETTLE_MS);
    servo_motor_vision.data_valid = 0U;
    servo_motor_vision.data_frame_counter = 0U;
    servo_motor_vision.data_timeout_count = 0U;
    last_frame = servo_motor_vision.data_frame_counter;
    valid_count = 0U;
    Vision_UART_StartTask(color_task);
    delay_ms(10);

    while (1) {
        ServoMotorVision_MoveServo0(pwm, SERVO0_SEARCH_MOVE_MS);

        if (ServoMotorVision_WaitForTarget(SERVO0_SEARCH_DWELL_MS,
                                            &last_frame,
                                            &valid_count)) {
            return;
        }

        if (direction > 0) {
            if ((uint16_t)(pwm + SERVO0_SEARCH_STEP_PWM) >= SERVO0_SEARCH_MAX_PWM) {
                pwm = SERVO0_SEARCH_MAX_PWM;
                direction = -1;
            } else {
                pwm = (uint16_t)(pwm + SERVO0_SEARCH_STEP_PWM);
            }
        } else {
            if (pwm <= (uint16_t)(SERVO0_SEARCH_MIN_PWM + SERVO0_SEARCH_STEP_PWM)) {
                pwm = SERVO0_SEARCH_MIN_PWM;
                direction = 1;
            } else {
                pwm = (uint16_t)(pwm - SERVO0_SEARCH_STEP_PWM);
            }
        }
    }
}

static void ApplyOutputs(ServoMotorVision_Cascade *smv)
{
#if ENABLE_X_AXIS_PID
    /* ── X轴 → 舵机0: 当前目标PWM按PID步长逐步修正 ── */
    {
        uint32_t now = HAL_GetTick();
        if (now - smv->last_move_tick >= SERVO0_CMD_INTERVAL_MS) {
            int32_t tmp = (int32_t)smv->servo0_target_pwm + (int32_t)smv->vx_ref;

            if (tmp < SERVO0_MIN_PWM)      tmp = SERVO0_MIN_PWM;
            else if (tmp > SERVO0_MAX_PWM) tmp = SERVO0_MAX_PWM;

            ServoMotorVision_MoveServo0((uint16_t)tmp, 80);
            smv->last_move_tick = now;
        }
    }
#endif

#if ENABLE_Y_AXIS_PID
    /* ── Y轴 → 舵机13 (360°连续舵机, 方向取反) ── */
    {
        int16_t speed = -(int16_t)smv->vy_ref;
        if (speed > 1000)  speed = 1000;
        if (speed < -1000) speed = -1000;
        servo13_SetSpeed(speed);
    }
#else
    servo13_SetSpeed(0);
#endif
}

/* ============ 生命周期 ============ */

void ServoMotorVision_Start(ServoMotorVision_Cascade *smv)
{
    ServoMotorVision_ConfigInit();

    smv->vx_ref = 0.0f;
    smv->vy_ref = 0.0f;
    smv->stable_count      = 0U;
    smv->data_timeout_count = 0U;
    smv->finished = 0U;

    PID_Cycle_Reset(&smv->x_cycle);
    PID_Cycle_Reset(&smv->y_cycle);
#if ENABLE_X_AXIS_PID
    PID_Cycle_SetEnable(&smv->x_cycle, 1U);
#else
    PID_Cycle_SetEnable(&smv->x_cycle, 0U);
#endif
#if ENABLE_Y_AXIS_PID
    PID_Cycle_SetEnable(&smv->y_cycle, 1U);
#else
    PID_Cycle_SetEnable(&smv->y_cycle, 0U);
#endif

    smv->enable = 1U;

    /* 初始停稳 */
    StopActuators();
}

void ServoMotorVision_Control(ServoMotorVision_Cascade *smv)
{
    if (smv->enable == 0U) {
        if (smv->finished == 0U) {
            StopActuators();
            smv->finished = 1U;
        }
        return;
    }

    /* ── 数据超时检测 ── */
    if (smv->data_timeout_count < 65535U) smv->data_timeout_count++;

    if (smv->data_valid == 0U || smv->data_timeout_count > smv->data_timeout_limit) {
        /* 无有效视觉数据 → 收手等待 */
        smv->vx_ref = 0.0f;
        smv->vy_ref = 0.0f;
        smv->stable_count = 0U;
        StopActuators();
        return;
    }

    /* ── PID 计算 ── */
#if ENABLE_X_AXIS_PID
    smv->vx_ref = PID_Control(&smv->x_cycle, smv->x_error, 0.0f);
    smv->vx_ref = LimitValue(smv->vx_ref, -smv->max_x_step, smv->max_x_step);
#else
    smv->vx_ref = 0.0f;
#endif
#if ENABLE_Y_AXIS_PID
    smv->vy_ref = PID_Control(&smv->y_cycle, smv->y_error, 0.0f);
    smv->vy_ref = LimitValue(smv->vy_ref, -(float)smv->max_y_speed, (float)smv->max_y_speed);
#else
    smv->vy_ref = 0.0f;
#endif

    /* ── 稳定判定: 像素死区 + Y轴速度在1500±100 ── */
    {
#if ENABLE_Y_AXIS_PID
        uint8_t y_speed_ok = (Abs(smv->vy_ref) < smv->y_speed_deadband);
#endif

#if ENABLE_X_AXIS_PID && ENABLE_Y_AXIS_PID
        if (Abs(smv->x_error) < smv->x_deadband &&
            Abs(smv->y_error) < smv->y_deadband && y_speed_ok) {
#elif ENABLE_X_AXIS_PID
        if (Abs(smv->x_error) < smv->x_deadband) {
#elif ENABLE_Y_AXIS_PID
        if (Abs(smv->y_error) < smv->y_deadband && y_speed_ok) {
#else
        if (1) {
#endif
            smv->stable_count++;
        } else {
            smv->stable_count = 0U;
        }
    }

    if (smv->stable_count >= smv->stable_count_limit) {
        /* 收敛完成 */
        StopActuators();
        Vision_UART_StopStream();
        smv->finished = 1U;
        smv->enable   = 0U;
        PID_Cycle_SetEnable(&smv->x_cycle, 0U);
        PID_Cycle_SetEnable(&smv->y_cycle, 0U);
        return;
    }

    /* ── 执行输出 ── */
    ApplyOutputs(smv);
}

void ServoMotorVision_Stop(ServoMotorVision_Cascade *smv)
{
    smv->enable = 0U;
    PID_Cycle_SetEnable(&smv->x_cycle, 0U);
    PID_Cycle_SetEnable(&smv->y_cycle, 0U);
    StopActuators();
}

/* ============ 视觉数据注入 ============ */

void ServoMotorVision_Update(ServoMotorVision_Cascade *smv,
                             float x, float y, uint8_t valid)
{
    smv->x_error = x;
    smv->y_error = y;
    smv->data_valid = valid ? 1U : 0U;
    if (smv->data_valid) {
        smv->data_timeout_count = 0U;
    }
    smv->data_frame_counter++;
}

/* ============ 阻塞接口 ============ */

void ServoMotorVision_Block(ServoMotorVision_Cascade *smv)
{
    extern void pit_control(void);
    ServoMotorVision_Start(smv);
    while (!smv->finished) {
        pit_control();
    }
}

static void ServoMotorVision_BlockWithResearch(ServoMotorVision_Cascade *smv, uint8_t color_task)
{
    extern void pit_control(void);

    ServoMotorVision_Start(smv);
    while (!smv->finished) {
        pit_control();

        if (smv->enable != 0U &&
            smv->data_valid == 0U &&
            smv->data_timeout_count > smv->data_timeout_limit) {
            ServoMotorVision_Stop(smv);
//            ServoMotorVision_SearchTarget(color_task);
            ServoMotorVision_Start(smv);
        }
    }
}

/* ============ 一键视觉校准 ============ */

/**
 * @brief 视觉校准入口 — 调用一次，全自动完成
 *
 * 流程:
 *   1. 通知 OpenMV 开始发送数据，并在 1860~2200 PWM 扫描寻找目标
 *   2. 找到目标后启动 ServoMotorVision PID 闭环
 *   3. 阻塞等待收敛 (pit_control 周期调度)
 *   4. 停止闭环 + 通知 OpenMV 停止 (发 'ok')
 *
 * 调用示例:
 *   ServoMotorVision_Calibrate(3U);
 */
void ServoMotorVision_Calibrate(uint8_t color_task)
{
    /* Step 1: 主动扫描，直到 OpenMV 看到目标 */
	  servo13_Home(1100);
//    ServoMotorVision_SearchTarget(color_task);
//delay_ms(1500);
    /* Step 2~3: 启动闭环；若丢目标超时，则重新搜索 */
    ServoMotorVision_BlockWithResearch(&servo_motor_vision, color_task);

    /* Step 4: 停止闭环 & 通知 OpenMV 停止 */
    Vision_UART_StopStream();
}
void ServoMotorVision_Calibrate_Pos(uint8_t color_task)
{
    /* Step 1: 定点扫描，直到 OpenMV 看到目标 */
	Vision_UART_StopStream();
	delay_ms(10);
	Vision_UART_StartTask(color_task);
		delay_ms(10);
	  servo13_Home(100);
		
		kandi();
		
//    ServoMotorVision_SearchTarget(color_task);
//delay_ms(1500);
    /* Step 2~3: 启动闭环；若丢目标超时，则重新搜索 */
    ServoMotorVision_BlockWithResearch(&servo_motor_vision, color_task);
		
    /* Step 4: 停止闭环 & 通知 OpenMV 停止 */
    Vision_UART_StopStream();
}
