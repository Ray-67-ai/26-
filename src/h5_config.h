#ifndef H5_CONFIG_H
#define H5_CONFIG_H

#include "h4_config.h"

#ifndef H5_TUNING_BUILD
#define H5_TUNING_BUILD                         (0)
#endif

/* Official H5 course: 1.5 m + pi*0.5 m + 1.5 m + pi*0.5 m. */
#define H5_TRACK_LENGTH_MM                      (6141.6f)

/* H5 keeps the proven H4 launch structure, but freezes the runtime values
 * verified during the full-lap real-car trials.  Do not alias these gains to
 * H4: H4 must retain its independently tuned 020741 parameter set. */
#define H5_TARGET_MM                            H4_TARGET_MM
#define H5_VISION_STALE_TIMEOUT_MS              H4_VISION_STALE_TIMEOUT_MS
#define H5_KEY_DEBOUNCE_MS                      H4_KEY_DEBOUNCE_MS
#define H5_PRECENTER_WINDOW_MM                   H4_PRECENTER_WINDOW_MM
#define H5_PRECENTER_MAX_SPEED_MM_S              H4_PRECENTER_MAX_SPEED_MM_S
#define H5_PRECENTER_STABLE_MS                   H4_PRECENTER_STABLE_MS
#define H5_LARGE_ERROR_BOOST_MM                  H4_LARGE_ERROR_BOOST_MM

#define H5_BALL_KP_DEG_PER_MM                   (0.0750f)
#define H5_BALL_KI_DEG_PER_MM_S                 (0.0000f)
#define H5_BALL_KD_DEG_PER_MM_S                 (0.0450f)
#define H5_PREDICTION_TIME_S                    (0.020f)
#define H5_ACCEL_FF_DEG_PER_M_S2                H4_ACCEL_FF_DEG_PER_M_S2
#define H5_NORMAL_MAX_ANGLE_DEG                  H4_NORMAL_MAX_ANGLE_DEG
#define H5_KICK_ANGLE_DEG                       H4_KICK_ANGLE_DEG
#define H5_MOTOR_MAX_SLEW_DEG_S                 H4_MOTOR_MAX_SLEW_DEG_S

#define H5_CRUISE_RPM                           (140.0f)
#define H5_ACCEL_TIME_MS                        (2700U)

/* Ignore all wide-line events until the last 441.6 mm of the lap.  The
 * perpendicular A marker must then be seen by at least four channels for
 * 30 ms.  This prevents the two semicircles from being mistaken for A. */
#define H5_FINISH_ARM_DISTANCE_MM               (5200.0f)
#define H5_FINISH_MIN_TIME_MS                   (19000U)
#define H5_FINISH_WIDE_CHANNELS                 (4U)
#define H5_FINISH_CONFIRM_MS                    (5U)
#define H5_START_LINE_CLEAR_MS                  (100U)

/* The scoring timer is latched at A.  The car then performs a smooth stop
 * after about 0.50 m so the complete chassis clears the A marker. */
#define H5_POST_A_DECEL_TIME_MS                 (3800U)
#define H5_POST_A_HARD_STOP_MM                  (520.0f)
#define H5_FINISH_FALLBACK_DISTANCE_MM          (6100.0f)
#define H5_RUN_FAILSAFE_MS                      (60000U)

/* Exact H4 020741 ball-control values. */
#define H5_ACCEL_KD_MULTIPLIER                  H4_ACCEL_KD_MULTIPLIER
#define H5_ACCEL_START_BIAS_DEG                 H4_ACCEL_START_BIAS_DEG
#define H5_ACCEL_START_BIAS_HOLD_MS             H4_ACCEL_START_BIAS_HOLD_MS
#define H5_ACCEL_START_BIAS_FADE_MS             H4_ACCEL_START_BIAS_FADE_MS
#define H5_INTEGRAL_LIMIT_MM_S                  H4_INTEGRAL_LIMIT_MM_S
#define H5_INTEGRAL_ACTIVE_ERROR_MM             H4_INTEGRAL_ACTIVE_ERROR_MM
#define H5_MOTOR_COMMAND_MIN_CHANGE_DEG          H4_MOTOR_COMMAND_MIN_CHANGE_DEG
#define H5_MOTOR_COMMAND_MAX_INTERVAL_MS         H4_MOTOR_COMMAND_MAX_INTERVAL_MS
#define H5_STUCK_POSITION_MM                    H4_STUCK_POSITION_MM
#define H5_STUCK_SPEED_MM_S                     H4_STUCK_SPEED_MM_S
#define H5_STUCK_CONFIRM_MS                     H4_STUCK_CONFIRM_MS
#define H5_CRUISE_KICK_ANGLE_DEG                (5.0f)
#define H5_KICK_MAX_MS                          H4_KICK_MAX_MS
#define H5_KICK_EXIT_CENTER_SPEED_MM_S          H4_KICK_EXIT_CENTER_SPEED_MM_S
#define H5_KICK_COOLDOWN_MS                     H4_KICK_COOLDOWN_MS

/* The two curve exits create repeatable longitudinal disturbances.  Apply
 * extra damping only while the ball is moving away from O in these windows. */
#define H5_EXIT_C_WINDOW_START_MM                (1700.0f)
#define H5_EXIT_C_WINDOW_END_MM                  (3200.0f)
#define H5_EXIT_A_WINDOW_START_MM                (4700.0f)
#define H5_EXIT_A_WINDOW_END_MM                  (5700.0f)
#define H5_EXIT_C_KD_MULTIPLIER                  (1.50f)
#define H5_EXIT_A_KD_MULTIPLIER                  (1.50f)

#define H5_LINE_CONTROL_PERIOD_MS               H4_LINE_CONTROL_PERIOD_MS
#define H5_SPEED_CONTROL_PERIOD_MS              H4_SPEED_CONTROL_PERIOD_MS
#define H5_UI_UPDATE_MS                         H4_UI_UPDATE_MS
#define H5_OLED_PAGE_UPDATE_MS                  H4_OLED_PAGE_UPDATE_MS

#endif
