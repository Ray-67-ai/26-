#include "h5_balance_loop.h"

#include "h3_vision.h"
#include "h4_tuning_link.h"
#include "h5_config.h"
#include "line_sensor.h"
#include "motor_encoder.h"
#include "ssd1306.h"
#include "ti_msp_dl_config.h"
#include "vehicle_config.h"
#include "zdt_stepper.h"

#include <stdbool.h>
#include <stdio.h>

typedef enum {
    H5_WAIT_VISION = 0,
    H5_READY,
    H5_ACCEL,
    H5_CRUISE,
    H5_DECEL,
    H5_STOPPED,
    H5_FAULT_VISION,
    H5_FAULT_LINE,
    H5_FAULT_TIMEOUT
} h5_state_t;

static volatile uint32_t g_millis;
static volatile bool g_start_key_event;
static h5_state_t g_state;

#if !H5_TUNING_BUILD
/* The H5 tuning image receives temporary values over RTT.  The formal
 * combined image instead uses this immutable, real-car-verified set so a
 * reset or power cycle cannot silently fall back to the H4 defaults. */
static const h4_tuning_runtime_t g_h5_competition_runtime = {
    .kp_deg_per_mm = H5_BALL_KP_DEG_PER_MM,
    .ki_deg_per_mm_s = H5_BALL_KI_DEG_PER_MM_S,
    .kd_deg_per_mm_s = H5_BALL_KD_DEG_PER_MM_S,
    .prediction_time_s = H5_PREDICTION_TIME_S,
    .accel_ff_deg_per_m_s2 = H5_ACCEL_FF_DEG_PER_M_S2,
    .normal_max_angle_deg = H5_NORMAL_MAX_ANGLE_DEG,
    .kick_angle_deg = H5_KICK_ANGLE_DEG,
    .max_motor_slew_deg_s = H5_MOTOR_MAX_SLEW_DEG_S,
    .cruise_rpm = H5_CRUISE_RPM,
    .accel_time_ms = H5_ACCEL_TIME_MS,
    .decel_start_mm = H4_DECEL_START_DISTANCE_MM,
    .decel_time_ms = H4_DECEL_TIME_MS,
    .generation = 0U,
};
#endif

static const h4_tuning_runtime_t *h5_runtime(void)
{
#if H5_TUNING_BUILD
    return h4_tuning_link_runtime();
#else
    return &g_h5_competition_runtime;
#endif
}
static bool g_motor_armed;
static bool g_oled_present;
static bool g_start_pending;
static bool g_start_line_cleared;
static bool g_finish_marker_seen;
static uint32_t g_last_key_ms;
static uint32_t g_centered_since_ms;
static uint32_t g_start_request_ms;
static uint32_t g_run_start_ms;
static uint32_t g_phase_start_ms;
static uint32_t g_lap_elapsed_ms;
static uint32_t g_finish_elapsed_ms;
static uint32_t g_last_line_control_ms;
static uint32_t g_last_speed_control_ms;
static uint32_t g_last_ui_ms;
static uint32_t g_last_oled_page_ms;
static uint32_t g_last_line_seen_ms;
static uint32_t g_last_motor_command_ms;
static uint32_t g_stuck_start_ms;
static uint32_t g_kick_start_ms;
static uint32_t g_kick_cooldown_until_ms;
static uint16_t g_start_clear_ms;
static uint16_t g_finish_wide_ms;
static int32_t g_start_right_count;
static int32_t g_start_left_count;
static int16_t g_last_line_error;
static float g_distance_mm;
static float g_finish_distance_mm;
static float g_target_rpm;
static float g_planned_accel_m_s2;
static float g_integral_mm_s;
static float g_last_motor_deg;
static float g_predicted_mm;
static float g_feedforward_deg;
static bool g_kick_active;
static line_sample_t g_line;

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

static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static float smoothstep(float u)
{
    u = clampf(u, 0.0f, 1.0f);
    return u * u * (3.0f - 2.0f * u);
}

static float smoothstep_derivative(float u)
{
    u = clampf(u, 0.0f, 1.0f);
    return 6.0f * u * (1.0f - u);
}

static uint32_t h5_millis(void)
{
    return g_millis;
}

void h5_balance_loop_tick_1ms_isr(void)
{
    ++g_millis;
}

void h5_balance_loop_start_key_isr(void)
{
    g_start_key_event = true;
}

static bool vision_is_recent(uint32_t now)
{
    const h3_vision_sample_t *vision = h3_vision_get();
    return (vision->good_frames > 0U) &&
        ((now - vision->last_valid_ms) <= H5_VISION_STALE_TIMEOUT_MS);
}

bool h5_balance_loop_vision_ready(void)
{
    return vision_is_recent(h5_millis());
}

static float travelled_distance_mm(void)
{
    const motor_state_t *motor = motor_get();
    int32_t right = abs_i32(motor->count_right - g_start_right_count);
    int32_t left = abs_i32(motor->count_left - g_start_left_count);
    float average_counts = (float) (right + left) * 0.5f;
    return average_counts *
        (CFG_PI * CFG_WHEEL_DIAMETER_MM) / CFG_COUNTS_PER_WHEEL_REV;
}

static bool send_motor_angle(uint32_t now, float desired_deg, bool force)
{
    const h4_tuning_runtime_t *runtime = h5_runtime();
    float command_deg = clampf(desired_deg,
        -runtime->kick_angle_deg, runtime->kick_angle_deg);

    if (!force && (g_last_motor_command_ms != 0U)) {
        float maximum_step = runtime->max_motor_slew_deg_s *
            (float) (now - g_last_motor_command_ms) * 0.001f;
        command_deg = clampf(command_deg,
            g_last_motor_deg - maximum_step,
            g_last_motor_deg + maximum_step);
    }
    if (!force &&
        (absf(command_deg - g_last_motor_deg) <
         H5_MOTOR_COMMAND_MIN_CHANGE_DEG) &&
        ((now - g_last_motor_command_ms) <
         H5_MOTOR_COMMAND_MAX_INTERVAL_MS)) {
        return true;
    }
    if (!zdt_stepper_move_absolute_deg(command_deg)) {
        return false;
    }
    g_last_motor_deg = command_deg;
    g_last_motor_command_ms = now;
    return true;
}

static void stop_vehicle(uint32_t now, h5_state_t final_state,
                         const char *event)
{
    motor_coast();
    g_target_rpm = 0.0f;
    g_planned_accel_m_s2 = 0.0f;
    g_integral_mm_s = 0.0f;
    g_feedforward_deg = 0.0f;
    g_finish_elapsed_ms = (g_start_request_ms == 0U) ?
        0U : (now - g_start_request_ms);
    g_state = final_state;
    g_start_pending = false;
    g_centered_since_ms = 0U;
    g_kick_active = false;
    g_stuck_start_ms = 0U;
    (void) send_motor_angle(now, 0.0f, true);
    h4_tuning_link_event(event);
}

static void reset_ready(uint32_t now)
{
    motor_coast();
    g_start_request_ms = 0U;
    g_run_start_ms = 0U;
    g_phase_start_ms = now;
    g_lap_elapsed_ms = 0U;
    g_finish_elapsed_ms = 0U;
    g_distance_mm = 0.0f;
    g_finish_distance_mm = 0.0f;
    g_target_rpm = 0.0f;
    g_planned_accel_m_s2 = 0.0f;
    g_integral_mm_s = 0.0f;
    g_predicted_mm = 0.0f;
    g_feedforward_deg = 0.0f;
    g_start_pending = false;
    g_start_line_cleared = false;
    g_finish_marker_seen = false;
    g_centered_since_ms = 0U;
    g_start_clear_ms = 0U;
    g_finish_wide_ms = 0U;
    g_kick_active = false;
    g_stuck_start_ms = 0U;
    g_kick_start_ms = 0U;
    g_kick_cooldown_until_ms = now;
    g_last_motor_command_ms = 0U;
    g_last_motor_deg = 0.0f;
    (void) send_motor_angle(now, 0.0f, true);
    g_state = vision_is_recent(now) ? H5_READY : H5_WAIT_VISION;
}

static void start_run(uint32_t now)
{
    const motor_state_t *motor;

    motor_prepare_run();
    motor = motor_get();
    g_start_right_count = motor->count_right;
    g_start_left_count = motor->count_left;
    g_run_start_ms = now;
    g_phase_start_ms = now;
    g_last_line_control_ms = now;
    g_last_speed_control_ms = now;
    g_last_line_seen_ms = now;
    g_last_line_error = 0;
    g_distance_mm = 0.0f;
    g_finish_distance_mm = 0.0f;
    g_target_rpm = 0.0f;
    g_planned_accel_m_s2 = 0.0f;
    g_integral_mm_s = 0.0f;
    g_start_line_cleared = false;
    g_finish_marker_seen = false;
    g_start_clear_ms = 0U;
    g_finish_wide_ms = 0U;
    g_stuck_start_ms = 0U;
    g_kick_active = false;
    g_kick_cooldown_until_ms = now;
    g_start_pending = false;
    g_centered_since_ms = 0U;
    g_state = H5_ACCEL;
    h4_tuning_link_event("H5_RUN_STARTED");
}

static void request_precenter(uint32_t now)
{
    motor_coast();
    g_target_rpm = 0.0f;
    g_planned_accel_m_s2 = 0.0f;
    g_start_request_ms = now;
    g_start_pending = true;
    g_centered_since_ms = 0U;
    g_state = H5_READY;
    h4_tuning_link_event("H5_PRECENTER_STARTED");
}

static void update_precenter(uint32_t now,
                             const h3_vision_sample_t *vision)
{
    bool centered;

    if (!g_start_pending || (g_state != H5_READY)) {
        return;
    }
    centered = (absf(vision->position_mm - H5_TARGET_MM) <=
                H5_PRECENTER_WINDOW_MM) &&
               (absf(vision->velocity_mm_s) <=
                H5_PRECENTER_MAX_SPEED_MM_S);
    if (!centered) {
        g_centered_since_ms = 0U;
        return;
    }
    if (g_centered_since_ms == 0U) {
        g_centered_since_ms = now;
    } else if ((now - g_centered_since_ms) >= H5_PRECENTER_STABLE_MS) {
        h4_tuning_link_event("H5_PRECENTER_COMPLETE");
        start_run(now);
    }
}

static void handle_key(uint32_t now)
{
    if (!g_start_key_event) {
        return;
    }
    g_start_key_event = false;
    if ((now - g_last_key_ms) < H5_KEY_DEBOUNCE_MS) {
        return;
    }
    g_last_key_ms = now;

    if ((g_state == H5_READY) || (g_state == H5_WAIT_VISION)) {
        if (vision_is_recent(now)) {
            request_precenter(now);
        }
    } else if ((g_state == H5_STOPPED) ||
               (g_state == H5_FAULT_VISION) ||
               (g_state == H5_FAULT_LINE) ||
               (g_state == H5_FAULT_TIMEOUT)) {
        reset_ready(now);
    }
}

static void handle_tuning_commands(uint32_t now)
{
    h4_tune_command_t command;

    while (h4_tuning_link_take_command(&command)) {
        switch (command) {
            case H4_TUNE_COMMAND_ARM:
                (void) zdt_stepper_enable(true);
                g_motor_armed = true;
                g_last_motor_command_ms = 0U;
                g_last_motor_deg = 0.0f;
                (void) send_motor_angle(now, 0.0f, true);
                h4_tuning_link_event("H5_ARMED");
                break;
            case H4_TUNE_COMMAND_START:
                if (!g_motor_armed) {
                    h4_tuning_link_event("H5_REJECT_NOT_ARMED");
                } else if (!vision_is_recent(now)) {
                    h4_tuning_link_event("H5_REJECT_NO_VISION");
                } else if ((g_state == H5_ACCEL) ||
                           (g_state == H5_CRUISE) ||
                           (g_state == H5_DECEL)) {
                    h4_tuning_link_event("H5_REJECT_ALREADY_RUNNING");
                } else {
                    if (g_state != H5_READY) {
                        reset_ready(now);
                    }
                    request_precenter(now);
                }
                break;
            case H4_TUNE_COMMAND_RESET:
                reset_ready(now);
                h4_tuning_link_event("H5_RESET_READY");
                break;
            case H4_TUNE_COMMAND_STOP:
                stop_vehicle(now, H5_STOPPED, "H5_MANUAL_STOP");
                break;
            default:
                break;
        }
    }
}

static void begin_finish_decel(uint32_t now, bool marker_seen)
{
    g_finish_marker_seen = marker_seen;
    g_lap_elapsed_ms = (g_start_request_ms == 0U) ?
        0U : (now - g_start_request_ms);
    g_finish_distance_mm = g_distance_mm;
    g_phase_start_ms = now;
    g_state = H5_DECEL;
    h4_tuning_link_event(marker_seen ?
        "H5_A_PASSED_DECEL_STARTED" : "H5_A_FALLBACK_DECEL_STARTED");
}

static void update_speed_profile(uint32_t now)
{
    const h4_tuning_runtime_t *runtime = h5_runtime();
    const float circumference_m =
        CFG_PI * CFG_WHEEL_DIAMETER_MM * 0.001f;

    if (g_state == H5_ACCEL) {
        uint32_t phase_ms = now - g_phase_start_ms;
        float u = (float) phase_ms / (float) runtime->accel_time_ms;
        float rpm_per_s;

        if (u >= 1.0f) {
            g_state = H5_CRUISE;
            g_phase_start_ms = now;
            g_target_rpm = runtime->cruise_rpm;
            g_planned_accel_m_s2 = 0.0f;
            h4_tuning_link_event("H5_CRUISE_STARTED");
        } else {
            g_target_rpm = runtime->cruise_rpm * smoothstep(u);
            rpm_per_s = runtime->cruise_rpm *
                smoothstep_derivative(u) * 1000.0f /
                (float) runtime->accel_time_ms;
            g_planned_accel_m_s2 =
                rpm_per_s * circumference_m / 60.0f;
        }
    } else if (g_state == H5_CRUISE) {
        g_target_rpm = runtime->cruise_rpm;
        g_planned_accel_m_s2 = 0.0f;
    } else if (g_state == H5_DECEL) {
        uint32_t phase_ms = now - g_phase_start_ms;
        float u = (float) phase_ms / (float) H5_POST_A_DECEL_TIME_MS;
        float rpm_per_s;

        if ((u >= 1.0f) ||
            ((g_distance_mm - g_finish_distance_mm) >=
             H5_POST_A_HARD_STOP_MM)) {
            stop_vehicle(now, H5_STOPPED, "H5_RUN_FINISHED");
            return;
        }
        g_target_rpm = runtime->cruise_rpm * (1.0f - smoothstep(u));
        rpm_per_s = -runtime->cruise_rpm *
            smoothstep_derivative(u) * 1000.0f /
            (float) H5_POST_A_DECEL_TIME_MS;
        g_planned_accel_m_s2 =
            rpm_per_s * circumference_m / 60.0f;
    }
}

static void update_finish_detection(uint32_t now)
{
    uint32_t elapsed = (g_start_request_ms == 0U) ?
        0U : (now - g_start_request_ms);
    bool finish_candidate =
        (g_state == H5_CRUISE) && g_start_line_cleared &&
        (g_distance_mm >= H5_FINISH_ARM_DISTANCE_MM) &&
        (elapsed >= H5_FINISH_MIN_TIME_MS) &&
        (g_line.black_count >= H5_FINISH_WIDE_CHANNELS);

    if (finish_candidate) {
        g_finish_wide_ms += H5_LINE_CONTROL_PERIOD_MS;
        if (g_finish_wide_ms >= H5_FINISH_CONFIRM_MS) {
            g_finish_wide_ms = 0U;
            begin_finish_decel(now, true);
        }
    } else {
        g_finish_wide_ms = 0U;
    }

    if ((g_state == H5_CRUISE) &&
        (g_distance_mm >= H5_FINISH_FALLBACK_DISTANCE_MM)) {
        begin_finish_decel(now, false);
    }
}

static void run_line_control(uint32_t now)
{
    float line_p;
    float line_d;
    float steer;
    float steer_limit;

    g_line = line_sensor_read();
    g_distance_mm = travelled_distance_mm();
    if (g_line.line_visible) {
        g_last_line_seen_ms = now;
    } else if ((now - g_last_line_seen_ms) > CFG_LINE_LOST_TIMEOUT_MS) {
        stop_vehicle(now, H5_FAULT_LINE, "H5_FAULT_LINE_LOST");
        return;
    }

    if (!g_start_line_cleared) {
        if (g_line.line_visible && !g_line.wide_line) {
            g_start_clear_ms += H5_LINE_CONTROL_PERIOD_MS;
            if (g_start_clear_ms >= H5_START_LINE_CLEAR_MS) {
                g_start_line_cleared = true;
            }
        } else {
            g_start_clear_ms = 0U;
        }
    }

    update_finish_detection(now);
    update_speed_profile(now);
    if ((g_state != H5_ACCEL) && (g_state != H5_CRUISE) &&
        (g_state != H5_DECEL)) {
        return;
    }

    line_p = CFG_LINE_KP_RPM * (float) g_line.error;
    line_d = CFG_LINE_KD_RPM *
        (float) (g_line.error - g_last_line_error);
    steer = line_p + line_d;
    steer_limit = g_target_rpm * 0.60f;
    if (steer_limit > CFG_SPEED_STEER_LIMIT_RPM) {
        steer_limit = CFG_SPEED_STEER_LIMIT_RPM;
    }
    steer = clampf(steer, -steer_limit, steer_limit);
    g_last_line_error = g_line.error;
    motor_set_targets(g_target_rpm - steer, g_target_rpm + steer);

    if (!g_finish_marker_seen &&
        (g_start_request_ms != 0U) &&
        ((now - g_start_request_ms) >= H5_RUN_FAILSAFE_MS)) {
        stop_vehicle(now, H5_FAULT_TIMEOUT, "H5_FAULT_TIMEOUT");
    }
}

static void update_stiction_logic(uint32_t now,
                                  const h3_vision_sample_t *vision)
{
    bool displaced_and_slow =
        (absf(g_predicted_mm) >= H5_STUCK_POSITION_MM) &&
        (absf(vision->velocity_mm_s) <= H5_STUCK_SPEED_MM_S);

    if (g_kick_active) {
        bool moving_to_center =
            ((vision->position_mm * vision->velocity_mm_s) < 0.0f) &&
            (absf(vision->velocity_mm_s) >=
             H5_KICK_EXIT_CENTER_SPEED_MM_S);
        if (((now - g_kick_start_ms) >= H5_KICK_MAX_MS) ||
            moving_to_center ||
            (absf(g_predicted_mm) < H5_STUCK_POSITION_MM)) {
            g_kick_active = false;
            g_kick_cooldown_until_ms = now + H5_KICK_COOLDOWN_MS;
            g_stuck_start_ms = 0U;
            h4_tuning_link_event("H5_KICK_FINISHED");
        }
        return;
    }

    if ((int32_t) (now - g_kick_cooldown_until_ms) < 0) {
        g_stuck_start_ms = 0U;
        return;
    }
    if (displaced_and_slow) {
        if (g_stuck_start_ms == 0U) {
            g_stuck_start_ms = now;
        } else if ((now - g_stuck_start_ms) >= H5_STUCK_CONFIRM_MS) {
            g_kick_active = true;
            g_kick_start_ms = now;
            g_stuck_start_ms = 0U;
            h4_tuning_link_event("H5_KICK_STARTED");
        }
    } else {
        g_stuck_start_ms = 0U;
    }
}

static void run_ball_control(uint32_t now,
                             const h3_vision_sample_t *vision)
{
    const h4_tuning_runtime_t *runtime = h5_runtime();
    float motor_deg;
    float phase_limit_deg = runtime->normal_max_angle_deg;
    float kd_deg_per_mm_s = runtime->kd_deg_per_mm_s;
    float dt_s = vision->sample_dt_s;
    bool running = (g_state == H5_ACCEL) || (g_state == H5_CRUISE) ||
                   (g_state == H5_DECEL);

    g_predicted_mm = vision->position_mm +
        runtime->prediction_time_s * vision->velocity_mm_s;
    if (g_start_pending || (g_state == H5_ACCEL) ||
        (g_state == H5_DECEL) ||
        (absf(g_predicted_mm) >= H5_LARGE_ERROR_BOOST_MM)) {
        phase_limit_deg = runtime->kick_angle_deg;
    }

    if (absf(g_predicted_mm) <= H5_INTEGRAL_ACTIVE_ERROR_MM &&
        (dt_s > 0.0f) && (dt_s <= 0.2f)) {
        g_integral_mm_s += g_predicted_mm * dt_s;
        g_integral_mm_s = clampf(g_integral_mm_s,
            -H5_INTEGRAL_LIMIT_MM_S, H5_INTEGRAL_LIMIT_MM_S);
    } else {
        g_integral_mm_s *= 0.95f;
    }

    g_feedforward_deg = running ?
        runtime->accel_ff_deg_per_m_s2 * g_planned_accel_m_s2 : 0.0f;
    if (g_state == H5_ACCEL) {
        uint32_t launch_ms = now - g_run_start_ms;
        kd_deg_per_mm_s *= H5_ACCEL_KD_MULTIPLIER;
        if (launch_ms < H5_ACCEL_START_BIAS_HOLD_MS) {
            g_feedforward_deg += H5_ACCEL_START_BIAS_DEG;
        } else if (launch_ms < (H5_ACCEL_START_BIAS_HOLD_MS +
                                H5_ACCEL_START_BIAS_FADE_MS)) {
            float fade_u = (float) (launch_ms -
                H5_ACCEL_START_BIAS_HOLD_MS) /
                (float) H5_ACCEL_START_BIAS_FADE_MS;
            g_feedforward_deg += H5_ACCEL_START_BIAS_DEG *
                (1.0f - smoothstep(fade_u));
        }
    } else if ((g_state == H5_CRUISE) &&
               ((g_predicted_mm * vision->velocity_mm_s) > 0.0f)) {
        if ((g_distance_mm >= H5_EXIT_C_WINDOW_START_MM) &&
            (g_distance_mm <= H5_EXIT_C_WINDOW_END_MM)) {
            kd_deg_per_mm_s *= H5_EXIT_C_KD_MULTIPLIER;
        } else if ((g_distance_mm >= H5_EXIT_A_WINDOW_START_MM) &&
                   (g_distance_mm <= H5_EXIT_A_WINDOW_END_MM)) {
            kd_deg_per_mm_s *= H5_EXIT_A_KD_MULTIPLIER;
        }
    }
    motor_deg = runtime->kp_deg_per_mm * g_predicted_mm +
        runtime->ki_deg_per_mm_s * g_integral_mm_s +
        kd_deg_per_mm_s * vision->velocity_mm_s +
        g_feedforward_deg;

    update_stiction_logic(now, vision);
    if (g_kick_active) {
        float kick_angle_deg = (g_state == H5_CRUISE) ?
            H5_CRUISE_KICK_ANGLE_DEG : runtime->kick_angle_deg;
        motor_deg = (g_predicted_mm >= 0.0f) ?
            kick_angle_deg : -kick_angle_deg;
    } else {
        motor_deg = clampf(motor_deg,
            -phase_limit_deg, phase_limit_deg);
    }
    (void) send_motor_angle(now, motor_deg, false);
}

static const char *state_text(void)
{
    switch (g_state) {
        case H5_WAIT_VISION:  return "WAIT CAM";
        case H5_READY:        return "READY";
        case H5_ACCEL:        return "ACCEL";
        case H5_CRUISE:       return "LAP";
        case H5_DECEL:        return "A DECEL";
        case H5_STOPPED:      return "STOP";
        case H5_FAULT_VISION: return "CAM LOST";
        case H5_FAULT_LINE:   return "LINE LOST";
        case H5_FAULT_TIMEOUT:return "TIMEOUT";
        default:              return "ERROR";
    }
}

static void update_ui(uint32_t now)
{
    const h3_vision_sample_t *vision = h3_vision_get();
    uint32_t elapsed = (g_start_request_ms == 0U) ? 0U :
        ((g_lap_elapsed_ms != 0U) ? g_lap_elapsed_ms :
         (now - g_start_request_ms));
    char text[24];

    ssd1306_clear();
    ssd1306_draw_text(0U, 0U, "H5 FULL LAP");
    (void) snprintf(text, sizeof(text), "%s CAM%s", state_text(),
        vision_is_recent(now) ? "OK" : "--");
    ssd1306_draw_text(0U, 1U, text);
    (void) snprintf(text, sizeof(text), "X%+4d V%+4d",
        (int) vision->position_mm, (int) vision->velocity_mm_s);
    ssd1306_draw_text(0U, 2U, text);
    (void) snprintf(text, sizeof(text), "D%04lu R%03d",
        (unsigned long) g_distance_mm, (int) g_target_rpm);
    ssd1306_draw_text(0U, 3U, text);
    (void) snprintf(text, sizeof(text), "ANG%+4d FF%+3d",
        (int) g_last_motor_deg, (int) g_feedforward_deg);
    ssd1306_draw_text(0U, 4U, text);
    (void) snprintf(text, sizeof(text), "L%02X N%u A%s",
        g_line.black_mask, g_line.black_count,
        (g_distance_mm >= H5_FINISH_ARM_DISTANCE_MM) ? "ON" : "--");
    ssd1306_draw_text(0U, 5U, text);
    (void) snprintf(text, sizeof(text), "TIME %lu.%02lu",
        (unsigned long) (elapsed / 1000U),
        (unsigned long) ((elapsed % 1000U) / 10U));
    ssd1306_draw_text(0U, 6U, text);
    ssd1306_draw_text(0U, 7U, g_start_pending ? "CENTERING O" :
        (g_finish_marker_seen ? "A PASSED" :
         (g_kick_active ? "STATIC KICK" : "Q3 START / RTT")));
}

void h5_balance_loop_init(void)
{
    g_millis = 0U;
    g_start_key_event = false;
    g_state = H5_WAIT_VISION;
    g_motor_armed = true;
    g_oled_present = ssd1306_is_present();
    g_start_pending = false;
    g_start_line_cleared = false;
    g_finish_marker_seen = false;
    g_last_key_ms = 0U;
    g_centered_since_ms = 0U;
    g_start_request_ms = 0U;
    g_run_start_ms = 0U;
    g_phase_start_ms = 0U;
    g_lap_elapsed_ms = 0U;
    g_finish_elapsed_ms = 0U;
    g_last_line_control_ms = 0U;
    g_last_speed_control_ms = 0U;
    g_last_ui_ms = 0U;
    g_last_oled_page_ms = 0U;
    g_last_line_seen_ms = 0U;
    g_last_motor_command_ms = 0U;
    g_stuck_start_ms = 0U;
    g_kick_start_ms = 0U;
    g_kick_cooldown_until_ms = 0U;
    g_start_clear_ms = 0U;
    g_finish_wide_ms = 0U;
    g_start_right_count = 0;
    g_start_left_count = 0;
    g_last_line_error = 0;
    g_distance_mm = 0.0f;
    g_finish_distance_mm = 0.0f;
    g_target_rpm = 0.0f;
    g_planned_accel_m_s2 = 0.0f;
    g_integral_mm_s = 0.0f;
    g_last_motor_deg = 0.0f;
    g_predicted_mm = 0.0f;
    g_feedforward_deg = 0.0f;
    g_kick_active = false;
    g_line = (line_sample_t) {0};

    h4_tuning_link_init();
}

void h5_balance_loop_process(void)
{
    uint32_t now = h5_millis();
    const h3_vision_sample_t *vision;
    const motor_state_t *motor;
    bool new_sample;
    bool running;

    h3_vision_process(now);
    h4_tuning_link_process();
    new_sample = h3_vision_take_new_sample();
    vision = h3_vision_get();
    handle_tuning_commands(now);

    if ((g_state == H5_WAIT_VISION) && vision_is_recent(now)) {
        g_state = H5_READY;
        h4_tuning_link_event("H5_VISION_READY");
    }
    handle_key(now);
    if (g_start_pending && !vision_is_recent(now)) {
        g_start_pending = false;
        g_centered_since_ms = 0U;
        g_state = H5_WAIT_VISION;
        (void) send_motor_angle(now, 0.0f, true);
        h4_tuning_link_event("H5_PRECENTER_VISION_LOST");
    }
    running = (g_state == H5_ACCEL) || (g_state == H5_CRUISE) ||
              (g_state == H5_DECEL);

    if (running && !vision_is_recent(now)) {
        stop_vehicle(now, H5_FAULT_VISION, "H5_FAULT_VISION_LOST");
    } else if (new_sample && vision->valid &&
               ((g_state == H5_READY) ||
                (g_state == H5_ACCEL) ||
                (g_state == H5_CRUISE) ||
                (g_state == H5_DECEL))) {
        run_ball_control(now, vision);
        update_precenter(now, vision);
    }

    running = (g_state == H5_ACCEL) || (g_state == H5_CRUISE) ||
              (g_state == H5_DECEL);
    if (running &&
        ((now - g_last_line_control_ms) >= H5_LINE_CONTROL_PERIOD_MS)) {
        g_last_line_control_ms = now;
        run_line_control(now);
    }
    if (running &&
        ((now - g_last_speed_control_ms) >= H5_SPEED_CONTROL_PERIOD_MS)) {
        g_last_speed_control_ms = now;
        motor_speed_control_10ms();
    }

    motor = motor_get();
    h4_tuning_link_telemetry(now, (uint8_t) g_state, g_motor_armed,
        vision, zdt_stepper_get_status(), &g_line, g_predicted_mm,
        g_last_motor_deg, g_feedforward_deg, g_distance_mm, g_target_rpm,
        0.5f * (motor->measured_right_rpm + motor->measured_left_rpm),
        g_kick_active,
        (g_start_request_ms == 0U) ? 0U :
            ((g_lap_elapsed_ms != 0U) ? g_lap_elapsed_ms :
             (now - g_start_request_ms)));

    if (g_oled_present && ((now - g_last_ui_ms) >= H5_UI_UPDATE_MS)) {
        g_last_ui_ms = now;
        update_ui(now);
    }
    if (g_oled_present &&
        ((now - g_last_oled_page_ms) >= H5_OLED_PAGE_UPDATE_MS)) {
        g_last_oled_page_ms = now;
        (void) ssd1306_refresh_next_page();
    }
}
