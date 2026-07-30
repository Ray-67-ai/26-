#include "motor_encoder.h"
#include "vehicle_config.h"
#include "ti_msp_dl_config.h"

/*
 * 电机与编码器全局状态。
 * 右/左编码器计数使用 volatile，因为在 ISR 中更新。
 */
static volatile int32_t g_right_count;
static volatile int32_t g_left_count;
static int32_t g_last_right_count;
static int32_t g_last_left_count;
static float g_right_integral;
static float g_left_integral;
static motor_state_t g_motor;

/* 限制浮点值在最小与最大之间。 */
static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/* 设置右电机前进方向的方向引脚。 */
static void set_right_direction_forward(void)
{
#if CFG_RIGHT_FORWARD_IN1_HIGH
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN2_PIN);
#else
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN2_PIN);
#endif
}

/* 设置左电机前进方向的方向引脚。 */
static void set_left_direction_forward(void)
{
#if CFG_LEFT_FORWARD_IN1_HIGH
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN2_PIN);
#else
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN2_PIN);
#endif
}

/*
 * 根据占空比设置左右电机 PWM 输出。
 */
static void set_pwm(float right_duty, float left_duty)
{
    float right = clampf(right_duty, 0.0f, 1.0f);
    float left = clampf(left_duty, 0.0f, 1.0f);
    uint32_t right_ticks = (uint32_t)
        (right * CFG_PWM_PERIOD_TICKS);
    uint32_t left_ticks = (uint32_t)
        (left * CFG_PWM_PERIOD_TICKS);

    DL_TimerA_setCaptureCompareValue(
        PWM_RIGHT_INST, right_ticks, GPIO_PWM_RIGHT_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        PWM_LEFT_INST, left_ticks, GPIO_PWM_LEFT_C0_IDX);
}

/* 初始化电机控制状态并保持电机自由滑行。 */
void motor_init(void)
{
    g_right_count = 0;
    g_left_count = 0;
    g_last_right_count = 0;
    g_last_left_count = 0;
    g_right_integral = 0.0f;
    g_left_integral = 0.0f;
    g_motor = (motor_state_t) {0};
    motor_coast();
}

/* 运行前准备：重置积分项并保存当前编码器计数为基准。 */
void motor_prepare_run(void)
{
    int32_t right_now = g_right_count;
    int32_t left_now = g_left_count;

    g_last_right_count = right_now;
    g_last_left_count = left_now;
    g_right_integral = 0.0f;
    g_left_integral = 0.0f;
    g_motor.count_right = right_now;
    g_motor.count_left = left_now;
    g_motor.measured_right_rpm = 0.0f;
    g_motor.measured_left_rpm = 0.0f;
    g_motor.duty_right = 0.0f;
    g_motor.duty_left = 0.0f;
}

/* 设置目标车轮转速并限制在最大允许速度内。 */
void motor_set_targets(float right_rpm, float left_rpm)
{
    g_motor.target_right_rpm =
        clampf(right_rpm, 0.0f, CFG_SPEED_MAX_RPM);
    g_motor.target_left_rpm =
        clampf(left_rpm, 0.0f, CFG_SPEED_MAX_RPM);
}

/*
 * 简单的 PI 速度控制器，返回 PWM 占空比。
 * 当目标速度接近 0 时重置积分项并输出 0。
 */
static float speed_pi(float target, float measured, float *integral)
{
    float error;
    float output;

    if (target < 1.0f) {
        *integral = 0.0f;
        return 0.0f;
    }

    error = target - measured;
    *integral += error * CFG_SPEED_KI * CFG_SPEED_CONTROL_DT_S;
    *integral = clampf(*integral, -0.20f, 0.70f);
    output = CFG_PWM_START_DUTY + CFG_SPEED_KP * error + *integral;
    return clampf(output, 0.0f, CFG_PWM_MAX_DUTY);
}

/*
 * 每 10ms 调用一次的速度环控制，计算实际 RPM 并更新 PWM 输出。
 */
void motor_speed_control_10ms(void)
{
    int32_t right_now = g_right_count;
    int32_t left_now = g_left_count;
    int32_t right_delta = right_now - g_last_right_count;
    int32_t left_delta = left_now - g_last_left_count;
    const float rpm_per_count =
        60.0f / (CFG_COUNTS_PER_WHEEL_REV * CFG_SPEED_CONTROL_DT_S);

    g_last_right_count = right_now;
    g_last_left_count = left_now;
    g_motor.count_right = right_now;
    g_motor.count_left = left_now;
    g_motor.measured_right_rpm = (float) right_delta * rpm_per_count;
    g_motor.measured_left_rpm = (float) left_delta * rpm_per_count;

    set_right_direction_forward();
    set_left_direction_forward();
    g_motor.duty_right = speed_pi(g_motor.target_right_rpm,
                                  g_motor.measured_right_rpm,
                                  &g_right_integral);
    g_motor.duty_left = speed_pi(g_motor.target_left_rpm,
                                 g_motor.measured_left_rpm,
                                 &g_left_integral);
    set_pwm(g_motor.duty_right, g_motor.duty_left);
}

/*
 * 让电机自由滑行：停止 PWM，并关闭方向引脚。
 */
void motor_coast(void)
{
    motor_set_targets(0.0f, 0.0f);
    set_pwm(0.0f, 0.0f);
    DL_GPIO_clearPins(MOTOR_DIR_PORT,
        MOTOR_DIR_AIN1_PIN | MOTOR_DIR_AIN2_PIN |
        MOTOR_DIR_BIN1_PIN | MOTOR_DIR_BIN2_PIN);
    g_right_integral = 0.0f;
    g_left_integral = 0.0f;
}

/*
 * 主动刹车：将 TB6612 设置为短制动状态并输出最大 PWM。
 */
void motor_active_brake(void)
{
    motor_set_targets(0.0f, 0.0f);
    /* TB6612：PWM为高且IN1=IN2时为短制动。 */
    DL_GPIO_setPins(MOTOR_DIR_PORT,
        MOTOR_DIR_AIN1_PIN | MOTOR_DIR_AIN2_PIN |
        MOTOR_DIR_BIN1_PIN | MOTOR_DIR_BIN2_PIN);
    set_pwm(1.0f, 1.0f);
    g_right_integral = 0.0f;
    g_left_integral = 0.0f;
}

/*
 * 右编码器边沿中断：根据 A/B 相状态增减计数。
 */
void motor_encoder_right_edge_isr(void)
{
    bool a_high =
        (DL_GPIO_readPins(ENCODER_PORT, ENCODER_RIGHT_A_PIN) != 0U);
    bool b_high =
        (DL_GPIO_readPins(ENCODER_PORT, ENCODER_RIGHT_B_PIN) != 0U);
#if CFG_RIGHT_FORWARD_WHEN_A_EQUALS_B
    g_right_count += (a_high == b_high) ? 1 : -1;
#else
    g_right_count += (a_high == b_high) ? -1 : 1;
#endif
}

/*
 * 左编码器边沿中断：根据 A/B 相状态增减计数。
 */
void motor_encoder_left_edge_isr(void)
{
    bool a_high =
        (DL_GPIO_readPins(ENCODER_PORT, ENCODER_LEFT_A_PIN) != 0U);
    bool b_high =
        (DL_GPIO_readPins(ENCODER_PORT, ENCODER_LEFT_B_PIN) != 0U);
#if CFG_LEFT_FORWARD_WHEN_A_EQUALS_B
    g_left_count += (a_high == b_high) ? 1 : -1;
#else
    g_left_count += (a_high == b_high) ? -1 : 1;
#endif
}

const motor_state_t *motor_get(void)
{
    return &g_motor;
}
