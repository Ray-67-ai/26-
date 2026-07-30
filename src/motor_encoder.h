#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <stdint.h>

/*
 * 电机和编码器控制状态结构。
 */
typedef struct {
    float target_right_rpm;
    float target_left_rpm;
    float measured_right_rpm;
    float measured_left_rpm;
    float duty_right;
    float duty_left;
    int32_t count_right;
    int32_t count_left;
} motor_state_t;

/* 初始化电机控制模块并保持滑行状态。 */
void motor_init(void);

/* 运行前准备：重置积分和编码器基准。 */
void motor_prepare_run(void);

/* 设置目标右/左车轮转速。 */
void motor_set_targets(float right_rpm, float left_rpm);

/* 10ms 速度环控制，用于更新 PWM 占空比。 */
void motor_speed_control_10ms(void);

/* 断开驱动，让电机自由滑行。 */
void motor_coast(void);

/* 主动刹车，TB6612 短制动模式。 */
void motor_active_brake(void);

/* 右编码器 A 相上升沿/下降沿中断处理。 */
void motor_encoder_right_edge_isr(void);

/* 左编码器 A 相上升沿/下降沿中断处理。 */
void motor_encoder_left_edge_isr(void);

/* 获取当前电机状态。 */
const motor_state_t *motor_get(void);

#endif
