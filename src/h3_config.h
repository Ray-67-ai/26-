#ifndef H3_CONFIG_H
#define H3_CONFIG_H

/* Formal competition build; RTT remains diagnostic-only. */
#define H3_AUTOTUNE_BUILD                    (0)
#define H3_STANDALONE_BUILD                  (0)
#define H3_AUTO_START_ENABLE                 (0)
#define H3_AUTO_START_VISION_STABLE_MS      (300U)

/* 第三问题面目标，单位为 mm。正方向为从 O 点指向 +5 cm。 */
#define H3_TARGET_POSITIVE_MM              (50.0f)
#define H3_TARGET_NEGATIVE_MM             (-55.0f)
/* Keep positive travel pulling past +50 mm so velocity feedback does not
 * brake the ball before it reaches the required +5 cm mark. */
#define H3_POSITIVE_DRIVE_TARGET_MM         (80.0f)
#define H3_POSITIVE_BRAKE_LOOKAHEAD_S        (0.36f)
#define H3_POSITIVE_HARD_TRIGGER_MM          (50.0f)
#define H3_FINAL_POSITION_TOLERANCE_MM      (6.0f)
#define H3_FINAL_SPEED_TOLERANCE_MM_S       (25.0f)
#define H3_FINAL_STABLE_TIME_MS             (250U)
/* Five seconds is the preferred score target, not an abandon point.  The
 * long timeout is only an abnormal-run safety watchdog; normal closed-loop
 * holding continues after the score target has been reached. */
#define H3_PREFERRED_FINISH_TIME_MS         (5000U)
#define H3_TOTAL_TIME_LIMIT_MS             (60000U)

/* 启动与失联保护。 */
#define H3_START_CENTER_TOLERANCE_MM        (15.0f)
#define H3_VISION_STALE_TIMEOUT_MS          (200U)
#define H3_KEY_DEBOUNCE_MS                  (150U)

/*
 * 球位置外环初值：
 *   motor = -Kp * (target - position) - Ki * integral + Kd * velocity
 * motor > 0 表示摆杆右端升高，球受到向左的加速度。
 * 这些参数只属于第三问，实车必须按调试顺序逐项调整。
 */
#define H3_BALL_KP_DEG_PER_MM               (0.1600f)
#define H3_BALL_KI_DEG_PER_MM_S             (0.0030f)
#define H3_BALL_KD_DEG_PER_MM_S             (0.1000f)
#define H3_INTEGRAL_LIMIT_MM_S               (120.0f)
#define H3_MAX_MOTOR_ANGLE_DEG               (9.0f)
#define H3_MAX_MOTOR_SLEW_DEG_S              (200.0f)
#define H3_NORMAL_PID_MAX_ANGLE_DEG           (7.0f)
#define H3_NEGATIVE_BRAKE_MAX_ANGLE_DEG       (7.0f)
#define H3_NEGATIVE_KD_SCALE                   (1.3f)
#define H3_STICTION_ERROR_MM                  (8.0f)
#define H3_STICTION_PROGRESS_MM               (3.0f)
#define H3_STICTION_STAGE1_DELAY_MS          (300U)
#define H3_STICTION_STAGE2_DELAY_MS          (900U)
#define H3_STICTION_STAGE3_DELAY_MS         (1800U)
#define H3_STICTION_STAGE1_ANGLE_DEG          (9.0f)
#define H3_STICTION_STAGE2_ANGLE_DEG          (9.0f)
#define H3_STICTION_STAGE3_ANGLE_DEG          (9.0f)
#define H3_POSITIVE_KICK_ANGLE_DEG            (7.0f)
#define H3_NEGATIVE_KICK_ANGLE_DEG            (9.0f)
#define H3_POSITIVE_KICK_MAX_MS               (200U)
#define H3_POSITIVE_KICK_EXIT_POSITION_MM     (12.0f)
#define H3_POSITIVE_KICK_EXIT_SPEED_MM_S      (45.0f)
#define H3_NEGATIVE_KICK_MAX_MS               (250U)
#define H3_NEGATIVE_KICK_EXIT_SPEED_MM_S      (0.0f)
#define H3_MOTOR_COMMAND_MIN_CHANGE_DEG      (0.03f)
#define H3_MOTOR_COMMAND_MAX_INTERVAL_MS     (60U)

/* 视觉坐标和速度低通滤波系数，范围 0~1，越大响应越快。 */
#define H3_POSITION_FILTER_ALPHA             (0.7f)
#define H3_VELOCITY_FILTER_ALPHA             (0.35f)

/* ZDT X42S Emm TTL普通绝对位置命令参数。 */
#define H3_ZDT_ADDRESS                       (1U)
#define H3_ZDT_SPEED_RPM                     (120U)
#define H3_ZDT_ACCELERATION                  (80U)
#define H3_ZDT_PULSES_PER_REV                (3200.0f)
#define H3_ZDT_CW_RAISES_RIGHT               (1)

/* OLED刷新周期。 */
#define H3_UI_UPDATE_MS                      (100U)
#define H3_OLED_PAGE_UPDATE_MS               (12U)

#endif
