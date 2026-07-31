#include "h3_ball_control.h"

#include "h3_config.h"
#include "h3_vision.h"
#include "motor_encoder.h"
#include "ssd1306.h"
#include "zdt_stepper.h"

#include <stdbool.h>
#include <stdio.h>

typedef enum {
    H3_WAIT_VISION = 0,
    H3_READY,
    H3_GO_POSITIVE,
    H3_GO_NEGATIVE,
    H3_HOLD_NEGATIVE,
    H3_DONE,
    H3_TIMEOUT,
    H3_VISION_FAULT
} h3_state_t;

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

static bool send_motor_angle(uint32_t now, float desired_deg, bool force)
{
    float command_deg;
    float maximum_step;

    desired_deg = clampf(desired_deg,
                         -H3_MAX_MOTOR_ANGLE_DEG,
                         H3_MAX_MOTOR_ANGLE_DEG);

    command_deg = desired_deg;
    if (g_last_command_ms != 0U) {
        maximum_step = H3_MAX_MOTOR_SLEW_DEG_S *
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
    g_target_mm = H3_TARGET_POSITIVE_MM;
    g_state = H3_GO_POSITIVE;
}

static void handle_key(uint32_t now)
{
    const h3_vision_sample_t *vision;

    if (!g_start_key_event) {
        return;
    }
    g_start_key_event = false;
    if ((now - g_last_key_ms) < H3_KEY_DEBOUNCE_MS) {
        return;
    }
    g_last_key_ms = now;
    vision = h3_vision_get();

    if ((g_state == H3_READY) && vision_is_recent(now) &&
        (absf(vision->position_mm) <= H3_START_CENTER_TOLERANCE_MM)) {
        start_run(now);
    } else if ((g_state == H3_DONE) || (g_state == H3_TIMEOUT) ||
               (g_state == H3_VISION_FAULT)) {
        g_integral_mm_s = 0.0f;
        g_target_mm = 0.0f;
        (void) send_motor_angle(now, 0.0f, true);
        g_state = vision_is_recent(now) ? H3_READY : H3_WAIT_VISION;
    }
}

static void update_motion_state(uint32_t now,
                                const h3_vision_sample_t *vision)
{
    if (g_state == H3_GO_POSITIVE) {
        if (vision->position_mm >= H3_POSITIVE_REACHED_MM) {
            g_positive_reached_ms = now - g_run_start_ms;
            g_target_mm = H3_TARGET_NEGATIVE_MM;
            g_integral_mm_s = 0.0f;
            g_stable_start_ms = 0U;
            g_state = H3_GO_NEGATIVE;
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
    }

    if (((g_state == H3_GO_POSITIVE) ||
         (g_state == H3_GO_NEGATIVE) ||
         (g_state == H3_HOLD_NEGATIVE)) &&
        ((now - g_run_start_ms) >= H3_TOTAL_TIME_LIMIT_MS)) {
        g_state = H3_TIMEOUT;
        g_finish_elapsed_ms = H3_TOTAL_TIME_LIMIT_MS;
        g_target_mm = H3_TARGET_NEGATIVE_MM;
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
    final_target = (g_state == H3_GO_NEGATIVE) ||
                   (g_state == H3_HOLD_NEGATIVE) ||
                   (g_state == H3_DONE) ||
                   (g_state == H3_TIMEOUT);

    error = g_target_mm - vision->position_mm;
    if (final_target && (dt_s > 0.0f) && (dt_s <= 0.2f)) {
        g_integral_mm_s += error * dt_s;
        g_integral_mm_s = clampf(g_integral_mm_s,
            -H3_INTEGRAL_LIMIT_MM_S, H3_INTEGRAL_LIMIT_MM_S);
    } else if (!final_target) {
        g_integral_mm_s = 0.0f;
    }

    motor_deg = -H3_BALL_KP_DEG_PER_MM * error
                -H3_BALL_KI_DEG_PER_MM_S * g_integral_mm_s
                +H3_BALL_KD_DEG_PER_MM_S * vision->velocity_mm_s;
    (void) send_motor_angle(now, motor_deg, false);
}

static void update_ui(uint32_t now)
{
    const h3_vision_sample_t *vision = h3_vision_get();
    const zdt_stepper_status_t *zdt = zdt_stepper_get_status();
    uint32_t elapsed = 0U;
    char text[24];
    int position = (int) vision->position_mm;
    int velocity = (int) vision->velocity_mm_s;
    int angle_tenths = (int) (g_last_command_angle_deg * 10.0f);

    if ((g_state == H3_GO_POSITIVE) || (g_state == H3_GO_NEGATIVE) ||
        (g_state == H3_HOLD_NEGATIVE)) {
        elapsed = now - g_run_start_ms;
    } else if ((g_state == H3_DONE) || (g_state == H3_TIMEOUT)) {
        elapsed = g_finish_elapsed_ms;
    }

    ssd1306_clear();
    ssd1306_draw_text(0U, 0U, "H3 BALL CTRL");
    ssd1306_draw_text(0U, 1U, state_text());

    (void) snprintf(text, sizeof(text), "X%+4d V%+4d", position, velocity);
    ssd1306_draw_text(0U, 3U, text);
    (void) snprintf(text, sizeof(text), "T%+3d A%+3d.%1d",
                    (int) g_target_mm,
                    angle_tenths / 10,
                    (angle_tenths < 0 ? -angle_tenths : angle_tenths) % 10);
    ssd1306_draw_text(0U, 4U, text);
    (void) snprintf(text, sizeof(text), "%lu.%02luS P%lu",
                    (unsigned long) (elapsed / 1000U),
                    (unsigned long) ((elapsed % 1000U) / 10U),
                    (unsigned long) g_positive_reached_ms);
    ssd1306_draw_text(0U, 6U, text);
    (void) snprintf(text, sizeof(text), "CAM%lu ACK%lu/%02X",
                    (unsigned long) vision->good_frames,
                    (unsigned long) zdt->rx_frames,
                    zdt->last_status);
    ssd1306_draw_text(0U, 7U, text);
}

void h3_ball_control_init(void)
{
    g_millis = 0U;
    g_start_key_event = false;
    g_state = H3_WAIT_VISION;
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

    motor_init();
    h3_vision_init();
    zdt_stepper_init();
    g_oled_present = ssd1306_init();

    (void) zdt_stepper_enable(true);
    (void) zdt_stepper_move_absolute_deg(0.0f);
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
    new_sample = h3_vision_take_new_sample();
    vision = h3_vision_get();

    if ((g_state == H3_WAIT_VISION) && vision_is_recent(now)) {
        g_state = H3_READY;
    }
    handle_key(now);

    if (((g_state == H3_GO_POSITIVE) ||
         (g_state == H3_GO_NEGATIVE) ||
         (g_state == H3_HOLD_NEGATIVE) ||
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
                (g_state == H3_DONE) ||
                (g_state == H3_TIMEOUT))) {
        run_outer_loop(now, vision);
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
