#include "app.h"
#include "line_sensor.h"
#include "motor_encoder.h"
#include "ssd1306.h"
#include "vehicle_config.h"

#include <stdbool.h>
#include <stdio.h>

/*
 * 应用级状态机：处理预热、就绪、运行、制动、停止和故障状态。
 * 该模块负责读取传感器、控制电机、更新 OLED 显示，并管理时间片调度。
 */
typedef enum {
    APP_WARMUP = 0,
    APP_READY,
    APP_RUNNING,
    APP_BRAKING,
    APP_STOPPED,
    APP_FAULT_ENCODER_LEVEL,
    APP_FAULT_OLED,
    APP_FAULT_LINE_LOST,
    APP_FAULT_TIMEOUT
} app_state_t;

static volatile uint32_t g_millis;
static volatile bool g_start_key_event;
static app_state_t g_app_state;
static uint32_t g_boot_ms;
static uint32_t g_run_start_ms;
static uint32_t g_stop_elapsed_ms;
static uint32_t g_brake_start_ms;
static uint32_t g_last_line_seen_ms;
static uint32_t g_last_key_ms;
static uint32_t g_last_line_control_ms;
static uint32_t g_last_speed_ms;
static uint32_t g_last_ui_ms;
static uint32_t g_last_oled_page_ms;
static uint16_t g_start_clear_ms;
static uint16_t g_finish_wide_ms;
static bool g_start_line_cleared;
static bool g_result_valid;
static line_sample_t g_line;
static int16_t g_last_line_error;
static int32_t g_start_right_count;
static int32_t g_start_left_count;
static float g_distance_mm;

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

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
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

/* 返回系统运行时间，单位为毫秒。 */
uint32_t app_millis(void)
{
    return g_millis;
}

/* 1ms 定时器中断调用，维护全局时间戳。 */
void app_tick_1ms_isr(void)
{
    ++g_millis;
}

/* 启动按键中断标记，主循环后续处理去抖和状态切换。 */
void app_start_key_isr(void)
{
    g_start_key_event = true;
}

/*
 * 进入指定停止/故障状态，并对电机进行短制动。
 * 记录运行时长用于后续显示或停止判定。
 */
static void stop_with_state(app_state_t state)
{
    if (g_app_state == APP_RUNNING) {
        g_stop_elapsed_ms = app_millis() - g_run_start_ms;
    }
    motor_active_brake();
    g_brake_start_ms = app_millis();
    g_app_state = state;
}

/*
 * 开始运行：重置计数、控制变量和状态，并设置初始目标转速。
 */
static void start_run(uint32_t now)
{
    const motor_state_t *motor;

    motor_prepare_run();
    motor = motor_get();
    g_run_start_ms = now;
    g_stop_elapsed_ms = 0U;
    g_last_line_seen_ms = now;
    g_start_clear_ms = 0U;
    g_finish_wide_ms = 0U;
    g_start_line_cleared = false;
    g_result_valid = false;
    g_last_line_error = 0;
    g_start_right_count = motor->count_right;
    g_start_left_count = motor->count_left;
    g_distance_mm = 0.0f;
    g_last_line_control_ms = now;
    g_last_speed_ms = now;
    g_app_state = APP_RUNNING;
    motor_set_targets(CFG_SPEED_NORMAL_RPM, CFG_SPEED_NORMAL_RPM);
}

/*
 * 处理按键事件：防抖后在 READY 状态启动运行，或在 STOPPED 状态重置为 READY。
 */
static void handle_key(uint32_t now)
{
    if (!g_start_key_event) {
        return;
    }
    /* Preserve an early Q1 press until the 500 ms sensor warmup completes. */
    if (g_app_state == APP_WARMUP) {
        return;
    }
    g_start_key_event = false;
    if ((now - g_last_key_ms) < 150U) {
        return;
    }
    g_last_key_ms = now;

    if (g_app_state == APP_READY) {
        start_run(now);
    } else if (g_app_state == APP_STOPPED) {
        motor_coast();
        g_result_valid = false;
        g_app_state = APP_READY;
    }
}

/*
 * 5ms 控制循环：读取线路传感器、计算偏差、执行 PID 转向并判定终点/故障。
 */
static void run_control_5ms(uint32_t now)
{
    uint32_t elapsed = now - g_run_start_ms;
    float base_rpm;
    float line_p;
    float line_d;
    float steer_rpm;
    float right_rpm;
    float left_rpm;
    float steer_limit;//新增
    g_line = line_sensor_read();
    g_distance_mm = travelled_distance_mm();

    if (g_line.line_visible) {
        g_last_line_seen_ms = now;
    } else if ((now - g_last_line_seen_ms) > CFG_LINE_LOST_TIMEOUT_MS) {
        stop_with_state(APP_FAULT_LINE_LOST);
        return;
    }

    /* 离开起点横线并连续看见普通窄线100 ms后，才允许识别终点。 */
    if (!g_start_line_cleared) {
        if (!g_line.wide_line && g_line.line_visible) {
            g_start_clear_ms += 5U;
            if (g_start_clear_ms >= CFG_START_LINE_CLEAR_MS) {
                g_start_line_cleared = true;
            }
        } else {
            g_start_clear_ms = 0U;
        }
    }

    base_rpm = (g_distance_mm >= CFG_FINISH_SLOWDOWN_DISTANCE_MM) ?
        CFG_SPEED_FINISH_RPM : CFG_SPEED_NORMAL_RPM;

    /*
     * 起步时先保持左右轮等速。只有车辆离开起点横线，并连续识别到
     * 普通窄线后才启用巡线转向，避免首次未读到线时的搜索误差让车
     * 固定向右急转。
     */
    if (!g_start_line_cleared) {
        line_p = 0.0f;
        line_d = 0.0f;
        steer_rpm = 0.0f;
        g_last_line_error = g_line.line_visible ? g_line.error : 0;
    } else {
        line_p = CFG_LINE_KP_RPM * (float) g_line.error;
        line_d = CFG_LINE_KD_RPM *
            (float) (g_line.error - g_last_line_error);
        steer_rpm = line_p + line_d;
       /*
 * 转向修正最多为当前基础速度的60%，
 * 防止低速时一侧车轮目标速度接近0。
 */
steer_limit = base_rpm * 0.60f;

if (steer_limit > CFG_SPEED_STEER_LIMIT_RPM) {
    steer_limit = CFG_SPEED_STEER_LIMIT_RPM;
}

steer_rpm = clampf(steer_rpm,
                   -steer_limit,
                   steer_limit);
        g_last_line_error = g_line.error;
    }

    right_rpm = base_rpm - steer_rpm;
    left_rpm = base_rpm + steer_rpm;
    motor_set_targets(right_rpm, left_rpm);

    if (g_start_line_cleared &&
        (elapsed >= CFG_FINISH_MIN_TIME_MS) &&
        g_line.wide_line) {
        g_finish_wide_ms += 5U;
        if (g_finish_wide_ms >= CFG_FINISH_DEBOUNCE_MS) {
            g_stop_elapsed_ms = elapsed;
            g_result_valid = true;
            motor_active_brake();
            g_brake_start_ms = now;
            g_app_state = APP_BRAKING;
            return;
        }
    } else {
        g_finish_wide_ms = 0U;
    }

    if (elapsed >= CFG_RUN_FAILSAFE_MS) {
        stop_with_state(APP_FAULT_TIMEOUT);
    }
}

/*
 * 根据当前状态返回 OLED 显示的状态字符串。
 */
static const char *state_text(void)
{
    if (g_result_valid &&
        ((g_app_state == APP_BRAKING) ||
         (g_app_state == APP_STOPPED))) {
        return "FINISH";
    }

    switch (g_app_state) {
        case APP_WARMUP:              return "WARMUP";
        case APP_READY:               return "READY";
        case APP_RUNNING:             return "RUN";
        case APP_BRAKING:             return "BRAKE";
        case APP_STOPPED:             return "STOP";
        case APP_FAULT_ENCODER_LEVEL: return "ENC 5V UNSAFE";
        case APP_FAULT_OLED:          return "OLED ERROR";
        case APP_FAULT_LINE_LOST:     return "LINE LOST";
        case APP_FAULT_TIMEOUT:       return "TIMEOUT";
        default:                      return "ERROR";
    }
}

/*
 * 更新 OLED 显示内容，显示状态、速度、计时和传感器信息。
 */
static void update_ui(uint32_t now)
{
    const motor_state_t *motor = motor_get();
    uint32_t shown_ms = 0U;
    uint32_t seconds;
    uint32_t centiseconds;
    uint32_t warmup_left = 0U;
    char text[24];

    if (g_app_state == APP_RUNNING) {
        shown_ms = now - g_run_start_ms;
    } else if ((g_stop_elapsed_ms > 0U) &&
               (g_app_state != APP_WARMUP) &&
               (g_app_state != APP_READY)) {
        shown_ms = g_stop_elapsed_ms;
    }
    seconds = shown_ms / 1000U;
    centiseconds = (shown_ms % 1000U) / 10U;

    ssd1306_clear();
    ssd1306_draw_text(0U, 0U, "H2 LINE CAR");
    ssd1306_draw_text(0U, 2U, state_text());

    (void) snprintf(text, sizeof(text), "R%4d L%4d",
                    (int) motor->measured_right_rpm,
                    (int) motor->measured_left_rpm);
    ssd1306_draw_text(0U, 3U, text);

    (void) snprintf(text, sizeof(text), "%02lu.%02lu S",
                    (unsigned long) seconds,
                    (unsigned long) centiseconds);
    ssd1306_draw_text_2x(0U, 4U, text);

    if (g_app_state == APP_WARMUP) {
        if ((now - g_boot_ms) < CFG_SENSOR_WARMUP_MS) {
            warmup_left = (CFG_SENSOR_WARMUP_MS -
                           (now - g_boot_ms) + 999U) / 1000U;
        }
        (void) snprintf(text, sizeof(text), "WAIT %02lu S",
                        (unsigned long) warmup_left);
    } else if (g_result_valid) {
        (void) snprintf(text, sizeof(text), "D%04lu RESULT",
                        (unsigned long) g_distance_mm);
    } else {
        (void) snprintf(text, sizeof(text), "D%04lu L%02X N%u",
                        (unsigned long) g_distance_mm,
                        g_line.black_mask, g_line.black_count);
    }
    ssd1306_draw_text(0U, 7U, text);
}

/*
 * 应用初始化：清除状态、复位子模块，并尝试初始化 OLED。
 */
void app_init(void)
{
    g_millis = 0U;
    g_start_key_event = false;
    g_app_state = APP_WARMUP;
    g_boot_ms = 0U;
    g_run_start_ms = 0U;
    g_stop_elapsed_ms = 0U;
    g_brake_start_ms = 0U;
    g_last_line_seen_ms = 0U;
    g_last_key_ms = 0U;
    g_last_line_control_ms = 0U;
    g_last_speed_ms = 0U;
    g_last_ui_ms = 0U;
    g_last_oled_page_ms = 0U;
    g_start_clear_ms = 0U;
    g_finish_wide_ms = 0U;
    g_start_line_cleared = false;
    g_result_valid = false;
    g_line = (line_sample_t) {0};
    g_last_line_error = 0;
    g_start_right_count = 0;
    g_start_left_count = 0;
    g_distance_mm = 0.0f;

    motor_init();
    (void) ssd1306_init();
    update_ui(0U);
}

/*
 * 应用主循环调用的处理函数，按时间片调度控制、显示和故障逻辑。
 */
void app_process(void)
{
    uint32_t now = app_millis();

    handle_key(now);

/*
 * 小车未运行时，也定期读取灰度传感器，
 * 方便在 READY/WARMUP/STOPPED 状态下实时检查线路。
 */
if ((g_app_state != APP_RUNNING) &&
    ((now - g_last_line_control_ms) >= 20U)) {
    g_last_line_control_ms = now;
    g_line = line_sensor_read();
}

/* 小车运行时使用5ms巡线控制周期。 */
if ((g_app_state == APP_RUNNING) &&
    ((now - g_last_line_control_ms) >= 5U)) {
    g_last_line_control_ms = now;
    run_control_5ms(now);
}

    /* 传感器预热完成后进入 READY 或编码器电平故障状态。 */
    if ((g_app_state == APP_WARMUP) &&
        ((now - g_boot_ms) >= CFG_SENSOR_WARMUP_MS)) {
#if CFG_ENCODER_INPUT_LEVEL_SAFE
        g_app_state = APP_READY;
#else
        g_app_state = APP_FAULT_ENCODER_LEVEL;
#endif
    }

    if ((g_app_state == APP_RUNNING) &&
        ((now - g_last_speed_ms) >= 10U)) {
        g_last_speed_ms = now;
        motor_speed_control_10ms();
    }

    /* 制动后短暂保持制动状态，然后进入滑行停止。 */
    if ((g_app_state == APP_BRAKING) &&
        ((now - g_brake_start_ms) >= CFG_ACTIVE_BRAKE_MS)) {
        motor_coast();
        g_app_state = APP_STOPPED;
    }

    /*
     * 故障停车也只短制动120 ms，随后释放，避免 TB6612 和电机长时间发热。
     * 故障状态本身保持不变。
     */
    if (((g_app_state == APP_FAULT_LINE_LOST) ||
         (g_app_state == APP_FAULT_TIMEOUT)) &&
        (g_brake_start_ms != 0U) &&
        ((now - g_brake_start_ms) >= CFG_ACTIVE_BRAKE_MS)) {
        motor_coast();
        g_brake_start_ms = 0U;
    }

    if ((now - g_last_ui_ms) >= 100U) {
        g_last_ui_ms = now;
        if (ssd1306_is_present()) {
            update_ui(now);
        }
    }
    /* 每次只刷新一页，避免整屏I2C传输阻塞5/10 ms控制周期。 */
    if ((now - g_last_oled_page_ms) >= 12U) {
        g_last_oled_page_ms = now;
        (void) ssd1306_refresh_next_page();
    }
}
