#ifndef H4_CONFIG_H
#define H4_CONFIG_H

/* The build script overrides this to 1 for J-Link/RTT-only test firmware. */
#ifndef H4_TUNING_BUILD
#define H4_TUNING_BUILD                      (0)
#endif

/* H4: keep the ball at O while the car follows AB and passes B. */
#define H4_TARGET_MM                         (0.0f)
/* The scoring boundary is +/-10 mm.  Use +/-8 mm as the internal
 * engineering guard band to retain margin for vision/mechanical scatter. */
#define H4_ALLOWED_ERROR_MM                   (8.0f)
#define H4_VISION_STALE_TIMEOUT_MS           (200U)
#define H4_KEY_DEBOUNCE_MS                   (150U)

/* A START request first holds the car still and closes the ball loop at O.
 * Vehicle motion begins only after position and speed have both remained
 * inside this window long enough to reject a one-frame false zero. */
#define H4_PRECENTER_WINDOW_MM                (3.0f)
#define H4_PRECENTER_MAX_SPEED_MM_S           (8.0f)
#define H4_PRECENTER_STABLE_MS                (300U)
#define H4_LARGE_ERROR_BOOST_MM               (20.0f)

/* Final road-speed profile verified on the physical car.  It crosses B
 * (about 1.50 m) in roughly 7.0 s, then decelerates after passing B. */
#define H4_CRUISE_RPM                        (160.0f)
#define H4_ACCEL_TIME_MS                     (2700U)
#define H4_DECEL_START_DISTANCE_MM           (1500.0f)
#define H4_DECEL_TIME_MS                     (1800U)
#define H4_HARD_STOP_DISTANCE_MM             (1800.0f)
#define H4_RUN_FAILSAFE_MS                   (10000U)

/* Ball controller.  For target O:
 * motor_deg = Kp*x_pred + Ki*integral(x) + Kd*v + Kff*a_car.
 * The feed-forward sign is deliberately runtime-adjustable because it
 * depends on the camera coordinate direction and the physical installation. */
#define H4_BALL_KP_DEG_PER_MM                (0.0600f)
#define H4_BALL_KI_DEG_PER_MM_S              (0.0000f)
#define H4_BALL_KD_DEG_PER_MM_S              (0.0350f)
#define H4_ACCEL_KD_MULTIPLIER                (1.30f)
#define H4_PREDICTION_TIME_S                  (0.020f)
#define H4_ACCEL_FF_DEG_PER_M_S2             (-13.5f)
/* The chassis briefly outruns its low-speed target during launch.  Apply a
 * small tube-only bias before that measured overshoot, then remove it before
 * cruise so the already-stable middle section is unchanged. */
#define H4_ACCEL_START_BIAS_DEG               (-1.0f)
#define H4_ACCEL_START_BIAS_HOLD_MS           (800U)
#define H4_ACCEL_START_BIAS_FADE_MS           (400U)
#define H4_INTEGRAL_LIMIT_MM_S                (80.0f)
#define H4_INTEGRAL_ACTIVE_ERROR_MM           (10.0f)
#define H4_NORMAL_MAX_ANGLE_DEG               (4.0f)
#define H4_MOTOR_MAX_SLEW_DEG_S               (140.0f)
#define H4_MOTOR_COMMAND_MIN_CHANGE_DEG       (0.03f)
#define H4_MOTOR_COMMAND_MAX_INTERVAL_MS      (60U)

/* Static-friction release.  It is allowed only after the ball has already
 * moved away from O and remained almost stationary for a short time. */
#define H4_STUCK_POSITION_MM                  (5.0f)
#define H4_STUCK_SPEED_MM_S                   (5.0f)
#define H4_STUCK_CONFIRM_MS                   (80U)
#define H4_KICK_ANGLE_DEG                     (7.0f)
#define H4_CRUISE_KICK_ANGLE_DEG              (3.5f)
#define H4_KICK_MAX_MS                        (80U)
#define H4_KICK_EXIT_CENTER_SPEED_MM_S        (12.0f)
#define H4_KICK_COOLDOWN_MS                   (180U)

/* Line following and user interface. */
#define H4_LINE_CONTROL_PERIOD_MS             (5U)
#define H4_SPEED_CONTROL_PERIOD_MS            (10U)
#define H4_UI_UPDATE_MS                       (100U)
#define H4_OLED_PAGE_UPDATE_MS                (12U)
#define H4_ACTIVE_BRAKE_MS                    (0U)

#endif
