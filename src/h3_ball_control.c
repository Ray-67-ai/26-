#include "h3_ball_control.h"

#include "h3_config.h"
#include "h3_tuning_link.h"
#include "h3_vision.h"
#include "motor_encoder.h"
#include "ssd1306.h"
#include "ti_msp_dl_config.h"
#include "zdt_stepper.h"

#include <stdbool.h>
#include <stdio.h>

typedef enum {
    H3_WAIT_VISION = 0,
    H3_READY,
    H3_GO_POSITIVE,
    H3_GO_NEGATIVE,
    H3_HOLD_NEGATIVE,
    H3_RETURN_CENTER,
    H3_DONE,
    H3_TIMEOUT,
    H3_VISION_FAULT
} h3_state_t;

typedef enum {
    H3_PHASE_IDLE = 0,
    H3_PHASE_POSITIVE_KICK,
    H3_PHASE_POSITIVE_PID,
    H3_PHASE_NEGATIVE_KICK,
    H3_PHASE_NEGATIVE_PID
} h3_motion_phase_t;

static volatile uint32_t g_millis;
static volatile bool g_start_key_event;
static h3_state_t g_state;
static uint32_t g_last_key_ms;
static uint32_t g_run_start_ms;
static uint32_t g_positive_reached_ms;
static uint32_t g_finish_elapsed_ms;
static uint32_t g_stable_start_ms;
static uint32_t g_last_ui_ms;
static uint32_t g_last_oled_page_ms;
static uint32_t g_last_command_ms;
static float g_target_mm;
static float g_integral_mm_s;
static float g_last_command_angle_deg;
static bool g_oled_present;
static uint32_t g_camera_ack_tx_frames;
static bool g_motor_armed;
static h3_motion_phase_t g_motion_phase;
static uint32_t g_phase_start_ms;
static uint32_t g_stall_start_ms;
static float g_stall_anchor_position_mm;
static uint32_t g_last_motor_diag_ms;
static bool g_motor_diag_read_position;
static uint32_t g_auto_start_vision_since_ms;
static bool g_auto_started;

static void send_camera_ack(const h3_vision_sample_t *vision)
{
    char frame[56];
    int length;
    int i;

    length = snprintf(frame, sizeof(frame), "A,%lu,%u,%lu,%lu\r\n",
        (unsigned long) vision->sequence,
        vision->valid ? 1U : 0U,
        (unsigned long) vision->good_frames,
        (unsigned long) vision->bad_frames);
    if ((length <= 0) || (length >= (int) sizeof(frame))) {
        return;
    }
    for (i = 0; i < length; ++i) {
        DL_UART_Main_transmitDataBlocking(
            VISION_UART_INST, (uint8_t) frame[i]);
    }
    ++g_camera_ack_tx_frames;
}

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

static uint32_t h3_millis(void)
{
    return g_millis;
}

void h3_ball_control_tick_1ms_isr(void)
{
    ++g_millis;
}

void h3_ball_control_start_key_isr(void)
{
    g_start_key_event = true;
}

void h3_ball_control_zdt_rx_isr(uint8_t byte)
{
    zdt_stepper_rx_byte_isr(byte);
}

void h3_ball_control_vision_rx_isr(uint8_t byte)
{
    h3_vision_rx_byte_isr(byte);
}

static const char *state_text(void)
{
    switch (g_state) {
        case H3_WAIT_VISION:   return "WAIT CAM";
        case H3_READY:         return "READY";
        case H3_GO_POSITIVE:   return "GO +5CM";
        case H3_GO_NEGATIVE:   return "GO -5CM";
        case H3_HOLD_NEGATIVE: return "HOLD -5";
        case H3_RETURN_CENTER: return "TO CENTER";
        case H3_DONE:          return "DONE";
        case H3_TIMEOUT:       return "TIMEOUT";
        case H3_VISION_FAULT:  return "CAM LOST";
        default:               return "ERROR";
    }
}

static bool vision_is_recent(uint32_t now)
{
    const h3_vision_sample_t *vision = h3_vision_get();
    return (vision->good_frames > 0U) &&
        ((now - vision->last_valid_ms) <= H3_VISION_STALE_TIMEOUT_MS);
}

bool h3_ball_control_vision_ready(void)
{
    return vision_is_recent(h3_millis());
}

static bool send_motor_angle(uint32_t now, float desired_deg, bool force)
{
    const h3_tuning_runtime_t *runtime = h3_tuning_link_runtime();
    float command_deg;
    float maximum_step;

    desired_deg = clampf(desired_deg,
                         -runtime->max_motor_angle_deg,
                         runtime->max_motor_angle_deg);

    command_deg = desired_deg;
    if (g_last_command_ms != 0U) {
        maximum_step = runtime->max_motor_slew_deg_s *
            (float) (now - g_last_command_ms) * 0.001f;
        command_deg = clampf(command_deg,
            g_last_command_angle_deg - maximum_step,
            g_last_command_angle_deg + maximum_step);
    }

    if (!force &&
        (absf(command_deg - g_last_command_angle_deg) <
         H3_MOTOR_COMMAND_MIN_CHANGE_DEG) &&
        ((now - g_last_command_ms) < H3_MOTOR_COMMAND_MAX_INTERVAL_MS)) {
        return true;
    }

    if (!zdt_stepper_move_absolute_deg(command_deg)) {
        return false;
    }
    g_last_command_angle_deg = command_deg;
    g_last_command_ms = now;
    return true;
}

static void start_run(uint32_t now)
{
    g_run_start_ms = now;
    g_positive_reached_ms = 0U;
    g_finish_elapsed_ms = 0U;
    g_stable_start_ms = 0U;
    g_integral_mm_s = 0.0f;
    g_target_mm = H3_POSITIVE_DRIVE_TARGET_MM;
    g_state = H3_GO_POSITIVE;
    g_motion_phase = H3_PHASE_POSITIVE_KICK;
    g_phase_start_ms = now;
    g_stall_start_ms = now;
    g_stall_anchor_position_mm = h3_vision_get()->position_mm;
    h3_tuning_link_event("RUN_STARTED");
    h3_tuning_link_event("POS_KICK_STARTED");
}

static void start_return_center(uint32_t now)
{
    g_target_mm = 0.0f;
    g_integral_mm_s = 0.0f;
    g_stable_start_ms = 0U;
    g_state = H3_RETURN_CENTER;
    g_motion_phase = H3_PHASE_IDLE;
    g_phase_start_ms = now;
    g_stall_start_ms = now;
    g_stall_anchor_position_mm = h3_vision_get()->position_mm;
    (void) send_motor_angle(now, 0.0f, true);
    h3_tuning_link_event("RETURN_CENTER");
}

static void handle_tuning_commands(uint32_t now)
{
    h3_tune_command_t command;

    while (h3_tuning_link_take_command(&command)) {
        switch (command) {
            case H3_TUNE_COMMAND_ARM:
                (void) zdt_stepper_enable(true);
                g_motor_armed = true;
                g_last_command_ms = 0U;
                g_last_command_angle_deg = 0.0f;
                (void) send_motor_angle(now, 0.0f, true);
                h3_tuning_link_event("ARMED");
                break;

            case H3_TUNE_COMMAND_START:
                if (!g_motor_armed) {
                    h3_tuning_link_event("REJECT_NOT_ARMED");
                } else if (!vision_is_recent(now)) {
                    h3_tuning_link_event("REJECT_NO_VISION");
                } else {
                    start_run(now);
                }
                break;

            case H3_TUNE_COMMAND_RESET:
                if (g_motor_armed && vision_is_recent(now)) {
                    start_return_center(now);
                } else {
                    h3_tuning_link_event("REJECT_RESET_NOT_READY");
                }
                break;

            case H3_TUNE_COMMAND_STOP:
                (void) zdt_stepper_stop_now();
                /* During bench tuning, park at the calibrated neutral angle
                 * and keep holding torque. Disabling immediately used to
                 * leave the beam tilted, so the ball continued rolling after
                 * the host had already declared the trial stopped. */
                (void) zdt_stepper_enable(true);
                g_last_command_ms = 0U;
                g_last_command_angle_deg = 0.0f;
                (void) send_motor_angle(now, 0.0f, true);
                g_motor_armed = true;
                g_target_mm = 0.0f;
                g_integral_mm_s = 0.0f;
                g_state = vision_is_recent(now) ? H3_READY : H3_WAIT_VISION;
                g_motion_phase = H3_PHASE_IDLE;
                g_phase_start_ms = now;
                h3_tuning_link_event("STOPPED_PARKED");
                break;

            case H3_TUNE_COMMAND_RAWTEST:
                if (!g_motor_armed) {
                    h3_tuning_link_event("REJECT_RAWTEST_NOT_ARMED");
                } else if (zdt_stepper_send_reference_test()) {
                    h3_tuning_link_event("RAWTEST_SENT");
                } else {
                    h3_tuning_link_event("RAWTEST_TX_ERROR");
                }
                break;

            case H3_TUNE_COMMAND_RAWBACK:
                if (!g_motor_armed) {
                    h3_tuning_link_event("REJECT_RAWBACK_NOT_ARMED");
                } else if (zdt_stepper_send_reference_back()) {
                    h3_tuning_link_event("RAWBACK_SENT");
                } else {
                    h3_tuning_link_event("RAWBACK_TX_ERROR");
                }
                break;

            default:
                break;
        }
    }
}

static void handle_key(uint32_t now)
{
#if H3_AUTOTUNE_BUILD
    /* Physical Q3 is deliberately ignored in the bench firmware. */
    g_start_key_event = false;
    (void) now;
    return;
#endif
    if (!g_start_key_event) {
        return;
    }

    g_start_key_event = false;

    if ((now - g_last_key_ms) < H3_KEY_DEBOUNCE_MS) {
        return;
    }

    g_last_key_ms = now;

    /*
     * 调试版本：
     * 1. 不要求视觉已经收到有效帧；
     * 2. 不要求钢球处于中心附近；
     * 3. READY或WAIT状态按键都能进入第三问。
     */
    if ((g_state == H3_READY) ||
        (g_state == H3_WAIT_VISION)) {
        start_run(now);
    } else if ((g_state == H3_DONE) ||
               (g_state == H3_TIMEOUT) ||
               (g_state == H3_VISION_FAULT)) {
        g_integral_mm_s = 0.0f;
        g_target_mm = 0.0f;
        (void) send_motor_angle(now, 0.0f, true);
        g_motion_phase = H3_PHASE_IDLE;
        g_phase_start_ms = now;

        /* 复位后直接回READY，不再回WAIT CAM */
        g_state = H3_READY;
    }
}
static void update_motion_state(uint32_t now,
                                const h3_vision_sample_t *vision)
{
    if (g_state == H3_GO_POSITIVE) {
        float positive_speed_mm_s =
            (vision->velocity_mm_s > 0.0f) ? vision->velocity_mm_s : 0.0f;
        float projected_position_mm = vision->position_mm +
            H3_POSITIVE_BRAKE_LOOKAHEAD_S * positive_speed_mm_s;
        if ((projected_position_mm >= H3_TARGET_POSITIVE_MM) ||
            (vision->position_mm >= H3_POSITIVE_HARD_TRIGGER_MM)) {
            g_positive_reached_ms = now - g_run_start_ms;
            g_target_mm = H3_TARGET_NEGATIVE_MM;
            g_integral_mm_s = 0.0f;
            g_stable_start_ms = 0U;
            g_state = H3_GO_NEGATIVE;
            g_motion_phase = H3_PHASE_NEGATIVE_KICK;
            g_phase_start_ms = now;
            h3_tuning_link_event("NEG_KICK_STARTED");
        }
    } else if ((g_state == H3_GO_NEGATIVE) ||
               (g_state == H3_HOLD_NEGATIVE)) {
        float error = H3_TARGET_NEGATIVE_MM - vision->position_mm;
        bool stable = (absf(error) <= H3_FINAL_POSITION_TOLERANCE_MM) &&
            (absf(vision->velocity_mm_s) <=
             H3_FINAL_SPEED_TOLERANCE_MM_S);

        if (stable) {
            if (g_stable_start_ms == 0U) {
                g_stable_start_ms = now;
            }
            g_state = H3_HOLD_NEGATIVE;
            if ((now - g_stable_start_ms) >= H3_FINAL_STABLE_TIME_MS) {
                g_finish_elapsed_ms = now - g_run_start_ms;
                g_state = H3_DONE;
            }
        } else {
            g_stable_start_ms = 0U;
            g_state = H3_GO_NEGATIVE;
        }
    } else if (g_state == H3_RETURN_CENTER) {
        bool centered = (absf(vision->position_mm) <=
                         H3_FINAL_POSITION_TOLERANCE_MM) &&
            (absf(vision->velocity_mm_s) <=
             H3_FINAL_SPEED_TOLERANCE_MM_S);

        if (centered) {
            if (g_stable_start_ms == 0U) {
                g_stable_start_ms = now;
            }
            if ((now - g_stable_start_ms) >= H3_FINAL_STABLE_TIME_MS) {
                g_state = H3_READY;
                g_integral_mm_s = 0.0f;
                g_stable_start_ms = 0U;
                h3_tuning_link_event("CENTERED_READY");
            }
        } else {
            g_stable_start_ms = 0U;
        }
    }

    if (((g_state == H3_GO_POSITIVE) ||
         (g_state == H3_GO_NEGATIVE) ||
         (g_state == H3_HOLD_NEGATIVE)) &&
        ((now - g_run_start_ms) >= H3_TOTAL_TIME_LIMIT_MS)) {
        g_state = H3_TIMEOUT;
        g_finish_elapsed_ms = H3_TOTAL_TIME_LIMIT_MS;
        g_target_mm = H3_TARGET_NEGATIVE_MM;
        g_motion_phase = H3_PHASE_IDLE;
        g_phase_start_ms = now;
    }
}

static void run_outer_loop(uint32_t now,
                           const h3_vision_sample_t *vision)
{
    float error;
    float motor_deg;
    bool final_target;
    float dt_s = vision->sample_dt_s;

    update_motion_state(now, vision);

    /* A safety-timeout run returns the beam to neutral.  A completed run
     * keeps evaluating the negative target so later disturbances are
     * corrected instead of abandoning the ball after the score timestamp. */
    if (g_state == H3_TIMEOUT) {
        g_target_mm = 0.0f;
        g_integral_mm_s = 0.0f;
        (void) send_motor_angle(now, 0.0f, true);
        return;
    }

    if ((g_state == H3_GO_POSITIVE) &&
        (g_motion_phase == H3_PHASE_POSITIVE_KICK)) {
        bool kick_finished =
            ((now - g_phase_start_ms) >= H3_POSITIVE_KICK_MAX_MS) ||
            (vision->position_mm >= H3_POSITIVE_KICK_EXIT_POSITION_MM) ||
            (vision->velocity_mm_s >= H3_POSITIVE_KICK_EXIT_SPEED_MM_S);

        if (!kick_finished) {
            (void) send_motor_angle(now, -H3_POSITIVE_KICK_ANGLE_DEG, false);
            return;
        }
        g_motion_phase = H3_PHASE_POSITIVE_PID;
        g_phase_start_ms = now;
        h3_tuning_link_event("POS_KICK_FINISHED");
    }

    if ((g_state == H3_GO_NEGATIVE) &&
        (g_motion_phase == H3_PHASE_NEGATIVE_KICK)) {
        bool kick_finished =
            ((now - g_phase_start_ms) >= H3_NEGATIVE_KICK_MAX_MS) ||
            (vision->velocity_mm_s <= H3_NEGATIVE_KICK_EXIT_SPEED_MM_S);

        if (!kick_finished) {
            (void) send_motor_angle(now, H3_NEGATIVE_KICK_ANGLE_DEG, false);
            return;
        }
        g_motion_phase = H3_PHASE_NEGATIVE_PID;
        g_phase_start_ms = now;
        h3_tuning_link_event("NEG_KICK_FINISHED");
    }

    final_target = (g_state == H3_GO_NEGATIVE) ||
                   (g_state == H3_HOLD_NEGATIVE) ||
                   (g_state == H3_DONE) ||
                   (g_state == H3_RETURN_CENTER);

    error = g_target_mm - vision->position_mm;
    if (final_target && (dt_s > 0.0f) && (dt_s <= 0.2f)) {
        g_integral_mm_s += error * dt_s;
        g_integral_mm_s = clampf(g_integral_mm_s,
            -H3_INTEGRAL_LIMIT_MM_S, H3_INTEGRAL_LIMIT_MM_S);
    } else if (!final_target) {
        g_integral_mm_s = 0.0f;
    }

    {
        const h3_tuning_runtime_t *runtime = h3_tuning_link_runtime();
        float kd = runtime->kd_deg_per_mm_s;
        if ((g_state == H3_GO_NEGATIVE) ||
            (g_state == H3_HOLD_NEGATIVE) ||
            (g_state == H3_DONE)) {
            kd *= H3_NEGATIVE_KD_SCALE;
        }
        motor_deg = -runtime->kp_deg_per_mm * error
                    -runtime->ki_deg_per_mm_s * g_integral_mm_s
                    +kd * vision->velocity_mm_s;
    }

    if (g_state == H3_GO_POSITIVE) {
        motor_deg = clampf(motor_deg,
            -H3_NORMAL_PID_MAX_ANGLE_DEG,
            H3_NORMAL_PID_MAX_ANGLE_DEG);
    } else if ((g_state == H3_GO_NEGATIVE) ||
               (g_state == H3_HOLD_NEGATIVE) ||
               (g_state == H3_DONE)) {
        /* During the negative leg, a negative motor command is braking.
         * Permit extra authority only in that direction; keep
         * negative travel drive at the normal 5.5-degree limit. */
        motor_deg = clampf(motor_deg,
            -H3_NEGATIVE_BRAKE_MAX_ANGLE_DEG,
            H3_NORMAL_PID_MAX_ANGLE_DEG);
    }

    /* Use measured displacement, rather than noisy instantaneous velocity,
     * to detect a ball that is genuinely stuck.  Normal motion keeps the
     * seven-degree PID authority.  If less than 3 mm progress is made, raise
     * breakaway authority in bounded stages; every 3 mm of real travel resets
     * the timer and immediately returns control to the normal PID. */
    if (absf(error) > H3_STICTION_ERROR_MM) {
        uint32_t stall_age_ms;
        float breakaway_angle_deg = 0.0f;

        if ((g_stall_start_ms == 0U) ||
            (absf(vision->position_mm - g_stall_anchor_position_mm) >=
             H3_STICTION_PROGRESS_MM)) {
            g_stall_start_ms = now;
            g_stall_anchor_position_mm = vision->position_mm;
        }
        stall_age_ms = now - g_stall_start_ms;
        if (stall_age_ms >= H3_STICTION_STAGE3_DELAY_MS) {
            breakaway_angle_deg = H3_STICTION_STAGE3_ANGLE_DEG;
        } else if (stall_age_ms >= H3_STICTION_STAGE2_DELAY_MS) {
            breakaway_angle_deg = H3_STICTION_STAGE2_ANGLE_DEG;
        } else if (stall_age_ms >= H3_STICTION_STAGE1_DELAY_MS) {
            breakaway_angle_deg = H3_STICTION_STAGE1_ANGLE_DEG;
        }
        if (breakaway_angle_deg > 0.0f) {
            motor_deg = (error > 0.0f) ?
                -breakaway_angle_deg : breakaway_angle_deg;
        }
    } else {
        g_stall_start_ms = 0U;
        g_stall_anchor_position_mm = vision->position_mm;
    }
    (void) send_motor_angle(now, motor_deg, false);
}

static void update_ui(uint32_t now)
{
    const h3_vision_sample_t *vision = h3_vision_get();
    const zdt_stepper_status_t *zdt = zdt_stepper_get_status();
    uint32_t age_ms = vision->has_frame ?
        (now - vision->received_ms) : 9999U;
    char text[24];

    ssd1306_clear();
    ssd1306_draw_text(0U, 0U, "H3 BALL CTRL");

    (void) snprintf(text, sizeof(text), "%s %s",
        state_text(), vision_is_recent(now) ? "CAM OK" : "CAM --");
    ssd1306_draw_text(0U, 1U, text);

    (void) snprintf(text, sizeof(text), "POS%+4d V%+4d",
        (int) vision->position_mm, (int) vision->velocity_mm_s);
    ssd1306_draw_text(0U, 2U, text);

    (void) snprintf(text, sizeof(text), "TGT%+4d ANG%+3d",
        (int) g_target_mm, (int) g_last_command_angle_deg);
    ssd1306_draw_text(0U, 3U, text);

    (void) snprintf(text, sizeof(text), "CAM RX%lu V%u",
        (unsigned long) (vision->rx_bytes % 100000UL),
        vision->valid ? 1U : 0U);
    ssd1306_draw_text(0U, 4U, text);

    /* G/I/E = valid / no-ball / malformed frame counters. */
    (void) snprintf(text, sizeof(text), "FRAME %lu/%lu/%lu",
        (unsigned long) (vision->good_frames % 1000UL),
        (unsigned long) (vision->invalid_frames % 1000UL),
        (unsigned long) (vision->bad_frames % 1000UL));
    ssd1306_draw_text(0U, 5U, text);

    (void) snprintf(text, sizeof(text), "ACK TX%lu A%lu",
        (unsigned long) (g_camera_ack_tx_frames % 100000UL),
        (unsigned long) (age_ms % 10000UL));
    ssd1306_draw_text(0U, 6U, text);

    (void) snprintf(text, sizeof(text), "ZDT T%lu R%lu E%lu",
        (unsigned long) (zdt->tx_frames % 1000UL),
        (unsigned long) (zdt->rx_frames % 1000UL),
        (unsigned long) (zdt->tx_errors % 1000UL));
    ssd1306_draw_text(0U, 7U, text);
}

void h3_ball_control_init(void)
{
    g_millis = 0U;
    g_start_key_event = false;
 /*
 * 调试阶段上电直接READY，
 * 不再通过WAIT CAM阻止按键启动。
 */
g_state = H3_READY;
    g_last_key_ms = 0U;
    g_run_start_ms = 0U;
    g_positive_reached_ms = 0U;
    g_finish_elapsed_ms = 0U;
    g_stable_start_ms = 0U;
    g_last_ui_ms = 0U;
    g_last_oled_page_ms = 0U;
    g_last_command_ms = 0U;
    g_target_mm = 0.0f;
    g_integral_mm_s = 0.0f;
    g_last_command_angle_deg = 0.0f;
    g_camera_ack_tx_frames = 0U;
    g_motor_armed = false;
    g_motion_phase = H3_PHASE_IDLE;
    g_phase_start_ms = 0U;
    g_stall_start_ms = 0U;
    g_stall_anchor_position_mm = 0.0f;
    g_last_motor_diag_ms = 0U;
    g_motor_diag_read_position = false;
    g_auto_start_vision_since_ms = 0U;
    g_auto_started = false;

    motor_init();
    h3_vision_init();
    zdt_stepper_init();
    h3_tuning_link_init();
    g_oled_present = ssd1306_init();

#if !H3_AUTOTUNE_BUILD
    (void) zdt_stepper_enable(true);
    g_motor_armed = true;
    (void) zdt_stepper_move_absolute_deg(0.0f);
#endif
    if (g_oled_present) {
        update_ui(0U);
    }
}

void h3_ball_control_process(void)
{
    uint32_t now = h3_millis();
    const h3_vision_sample_t *vision;
    bool new_sample;

    h3_vision_process(now);
    h3_tuning_link_process();
    new_sample = h3_vision_take_new_sample();
    vision = h3_vision_get();
    handle_tuning_commands(now);

    if (new_sample) {
        send_camera_ack(vision);
    }

    if ((g_state == H3_WAIT_VISION) && vision_is_recent(now)) {
        g_state = H3_READY;
    }
    handle_key(now);

#if H3_AUTO_START_ENABLE
    if ((g_state == H3_READY) && !g_auto_started) {
        if (vision_is_recent(now)) {
            if (g_auto_start_vision_since_ms == 0U) {
                g_auto_start_vision_since_ms = now;
            } else if ((now - g_auto_start_vision_since_ms) >=
                       H3_AUTO_START_VISION_STABLE_MS) {
                g_auto_started = true;
                start_run(now);
            }
        } else {
            g_auto_start_vision_since_ms = 0U;
        }
    }
#endif

    if (((g_state == H3_GO_POSITIVE) ||
         (g_state == H3_GO_NEGATIVE) ||
         (g_state == H3_HOLD_NEGATIVE) ||
         (g_state == H3_RETURN_CENTER) ||
         (g_state == H3_DONE) ||
         (g_state == H3_TIMEOUT)) &&
        !vision_is_recent(now)) {
        g_state = H3_VISION_FAULT;
        g_target_mm = 0.0f;
        g_integral_mm_s = 0.0f;
        (void) send_motor_angle(now, 0.0f, true);
    } else if (new_sample && vision->valid &&
               ((g_state == H3_GO_POSITIVE) ||
                (g_state == H3_GO_NEGATIVE) ||
                (g_state == H3_HOLD_NEGATIVE) ||
                (g_state == H3_RETURN_CENTER) ||
                (g_state == H3_DONE) ||
                (g_state == H3_TIMEOUT))) {
        run_outer_loop(now, vision);
    }

    h3_tuning_link_telemetry(now, (uint8_t) g_state, g_motor_armed,
        vision, zdt_stepper_get_status(), g_target_mm,
        g_last_command_angle_deg,
        (g_run_start_ms == 0U) ? 0U : (now - g_run_start_ms));

    if ((now - g_last_motor_diag_ms) >= 200U) {
        g_last_motor_diag_ms = now;
        if (g_motor_diag_read_position) {
            (void) zdt_stepper_read_real_position();
        } else {
            (void) zdt_stepper_read_status_flags();
        }
        g_motor_diag_read_position = !g_motor_diag_read_position;
    }

    if (g_oled_present && ((now - g_last_ui_ms) >= H3_UI_UPDATE_MS)) {
        g_last_ui_ms = now;
        update_ui(now);
    }
    if (g_oled_present &&
        ((now - g_last_oled_page_ms) >= H3_OLED_PAGE_UPDATE_MS)) {
        g_last_oled_page_ms = now;
        (void) ssd1306_refresh_next_page();
    }
}
