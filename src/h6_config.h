#ifndef H6_CONFIG_H
#define H6_CONFIG_H

#include "h5_config.h"

#ifndef H6_TUNING_BUILD
#define H6_TUNING_BUILD                         (0)
#endif

/* H6 uses the ball position present when Q3 is pressed as its fixed target.
 * The camera keeps reporting its normal absolute coordinate; only the MCU
 * subtracts the captured target for feedback control. */
#define H6_CAPTURE_STABLE_MS                    (300U)
#define H6_CAPTURE_MIN_SAMPLES                  (5U)
#define H6_CAPTURE_MAX_SPEED_MM_S               (8.0f)
#define H6_CAPTURE_MAX_SPREAD_MM                (3.0f)
#define H6_PRESTART_WINDOW_MM                   (3.0f)
#define H6_PRESTART_MAX_SPEED_MM_S              (8.0f)
#define H6_PRESTART_STABLE_MS                   (300U)

/* The full-lap route and the verified H5 controller are intentionally kept
 * unchanged.  H6 differs only in target acquisition and relative error. */
#define H6_VISION_STALE_TIMEOUT_MS              H5_VISION_STALE_TIMEOUT_MS
#define H6_KEY_DEBOUNCE_MS                      H5_KEY_DEBOUNCE_MS
#define H6_LARGE_ERROR_BOOST_MM                 H5_LARGE_ERROR_BOOST_MM
#define H6_BALL_KP_DEG_PER_MM                   H5_BALL_KP_DEG_PER_MM
#define H6_BALL_KI_DEG_PER_MM_S                 H5_BALL_KI_DEG_PER_MM_S
#define H6_BALL_KD_DEG_PER_MM_S                 H5_BALL_KD_DEG_PER_MM_S
#define H6_PREDICTION_TIME_S                    H5_PREDICTION_TIME_S
#define H6_ACCEL_FF_DEG_PER_M_S2                H5_ACCEL_FF_DEG_PER_M_S2
#define H6_NORMAL_MAX_ANGLE_DEG                 H5_NORMAL_MAX_ANGLE_DEG
#define H6_KICK_ANGLE_DEG                       H5_KICK_ANGLE_DEG
#define H6_MOTOR_MAX_SLEW_DEG_S                 H5_MOTOR_MAX_SLEW_DEG_S
#define H6_CRUISE_RPM                           H5_CRUISE_RPM
#define H6_ACCEL_TIME_MS                        H5_ACCEL_TIME_MS
#define H6_FINISH_ARM_DISTANCE_MM               H5_FINISH_ARM_DISTANCE_MM
#define H6_FINISH_MIN_TIME_MS                   H5_FINISH_MIN_TIME_MS
#define H6_FINISH_WIDE_CHANNELS                 H5_FINISH_WIDE_CHANNELS
#define H6_FINISH_CONFIRM_MS                    H5_FINISH_CONFIRM_MS
#define H6_START_LINE_CLEAR_MS                  H5_START_LINE_CLEAR_MS
#define H6_POST_A_DECEL_TIME_MS                 H5_POST_A_DECEL_TIME_MS
#define H6_POST_A_HARD_STOP_MM                  H5_POST_A_HARD_STOP_MM
#define H6_FINISH_FALLBACK_DISTANCE_MM          H5_FINISH_FALLBACK_DISTANCE_MM
#define H6_RUN_FAILSAFE_MS                      H5_RUN_FAILSAFE_MS
#define H6_ACCEL_KD_MULTIPLIER                  H5_ACCEL_KD_MULTIPLIER
#define H6_ACCEL_START_BIAS_DEG                 H5_ACCEL_START_BIAS_DEG
#define H6_ACCEL_START_BIAS_HOLD_MS             H5_ACCEL_START_BIAS_HOLD_MS
#define H6_ACCEL_START_BIAS_FADE_MS             H5_ACCEL_START_BIAS_FADE_MS
#define H6_INTEGRAL_LIMIT_MM_S                  H5_INTEGRAL_LIMIT_MM_S
#define H6_INTEGRAL_ACTIVE_ERROR_MM             H5_INTEGRAL_ACTIVE_ERROR_MM
#define H6_MOTOR_COMMAND_MIN_CHANGE_DEG         H5_MOTOR_COMMAND_MIN_CHANGE_DEG
#define H6_MOTOR_COMMAND_MAX_INTERVAL_MS        H5_MOTOR_COMMAND_MAX_INTERVAL_MS
#define H6_STUCK_POSITION_MM                    H5_STUCK_POSITION_MM
#define H6_STUCK_SPEED_MM_S                     H5_STUCK_SPEED_MM_S
#define H6_STUCK_CONFIRM_MS                     H5_STUCK_CONFIRM_MS
#define H6_CRUISE_KICK_ANGLE_DEG                H5_CRUISE_KICK_ANGLE_DEG
#define H6_KICK_MAX_MS                          H5_KICK_MAX_MS
#define H6_KICK_EXIT_CENTER_SPEED_MM_S          H5_KICK_EXIT_CENTER_SPEED_MM_S
#define H6_KICK_COOLDOWN_MS                     H5_KICK_COOLDOWN_MS
#define H6_EXIT_C_WINDOW_START_MM               H5_EXIT_C_WINDOW_START_MM
#define H6_EXIT_C_WINDOW_END_MM                 H5_EXIT_C_WINDOW_END_MM
#define H6_EXIT_A_WINDOW_START_MM               H5_EXIT_A_WINDOW_START_MM
#define H6_EXIT_A_WINDOW_END_MM                 H5_EXIT_A_WINDOW_END_MM
#define H6_EXIT_C_KD_MULTIPLIER                 H5_EXIT_C_KD_MULTIPLIER
#define H6_EXIT_A_KD_MULTIPLIER                 H5_EXIT_A_KD_MULTIPLIER
#define H6_LINE_CONTROL_PERIOD_MS               H5_LINE_CONTROL_PERIOD_MS
#define H6_SPEED_CONTROL_PERIOD_MS              H5_SPEED_CONTROL_PERIOD_MS
#define H6_UI_UPDATE_MS                         H5_UI_UPDATE_MS
#define H6_OLED_PAGE_UPDATE_MS                  H5_OLED_PAGE_UPDATE_MS

#endif
