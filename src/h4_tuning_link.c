#include "h4_tuning_link.h"

#include "h4_config.h"
#include "zdt_stepper.h"
#include "segger_rtt/SEGGER_RTT.h"

#include <stdio.h>
#include <string.h>

#define H4_TUNE_LINE_CAPACITY       (96U)
#define H4_TUNE_TELEMETRY_PERIOD_MS (50U)
#define H4_TUNE_GAIN_SCALE          (100000.0f)

static char g_line[H4_TUNE_LINE_CAPACITY];
static uint8_t g_length;
static h4_tune_command_t g_pending;
static h4_tuning_runtime_t g_runtime;
static uint32_t g_last_telemetry_ms;
static bool g_force_telemetry;

static int32_t scaled_float(float value, float scale)
{
    float scaled = value * scale;
    return (int32_t) (scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void reply(const char *text)
{
    SEGGER_RTT_WriteString(0U, text);
    SEGGER_RTT_WriteString(0U, "\r\n");
}

static void parse_command(void)
{
    long kp;
    long ki;
    long kd;
    long prediction_ms;
    long ff_mdeg_per_m_s2;
    unsigned long normal_mdeg;
    unsigned long kick_mdeg;
    unsigned long slew_mdeg_s;
    unsigned long cruise_rpm_x10;
    unsigned long accel_ms;
    unsigned long decel_start_mm;
    unsigned long decel_ms;

    g_line[g_length] = '\0';
    if (strcmp(g_line, "ARM") == 0) {
        g_pending = H4_TUNE_COMMAND_ARM;
        reply("OK4,ARM");
    } else if (strcmp(g_line, "START") == 0) {
        g_pending = H4_TUNE_COMMAND_START;
        reply("OK4,START");
    } else if (strcmp(g_line, "RESET") == 0) {
        g_pending = H4_TUNE_COMMAND_RESET;
        reply("OK4,RESET");
    } else if (strcmp(g_line, "STOP") == 0) {
        g_pending = H4_TUNE_COMMAND_STOP;
        reply("OK4,STOP");
    } else if (strcmp(g_line, "STATUS") == 0) {
        g_force_telemetry = true;
        reply("OK4,STATUS");
    } else if (strcmp(g_line, "ZDTREAD") == 0) {
        if (zdt_stepper_read_driver_config()) {
            g_force_telemetry = true;
            reply("OK4,ZDTREAD");
        } else {
            reply("ERR4,ZDTREAD");
        }
    } else if (sscanf(g_line, "PID,%ld,%ld,%ld,%ld",
                      &kp, &ki, &kd, &prediction_ms) == 4) {
        if ((kp >= 0L) && (kp <= 50000L) &&
            (ki >= 0L) && (ki <= 20000L) &&
            (kd >= 0L) && (kd <= 50000L) &&
            (prediction_ms >= 0L) && (prediction_ms <= 200L)) {
            g_runtime.kp_deg_per_mm = (float) kp / H4_TUNE_GAIN_SCALE;
            g_runtime.ki_deg_per_mm_s = (float) ki / H4_TUNE_GAIN_SCALE;
            g_runtime.kd_deg_per_mm_s = (float) kd / H4_TUNE_GAIN_SCALE;
            g_runtime.prediction_time_s = (float) prediction_ms * 0.001f;
            ++g_runtime.generation;
            reply("OK4,PID");
        } else {
            reply("ERR4,PID_RANGE");
        }
    } else if (sscanf(g_line, "FF,%ld", &ff_mdeg_per_m_s2) == 1) {
        if ((ff_mdeg_per_m_s2 >= -20000L) &&
            (ff_mdeg_per_m_s2 <= 20000L)) {
            g_runtime.accel_ff_deg_per_m_s2 =
                (float) ff_mdeg_per_m_s2 * 0.001f;
            ++g_runtime.generation;
            reply("OK4,FF");
        } else {
            reply("ERR4,FF_RANGE");
        }
    } else if (sscanf(g_line, "LIMIT,%lu,%lu,%lu",
                      &normal_mdeg, &kick_mdeg, &slew_mdeg_s) == 3) {
        if ((normal_mdeg >= 1000UL) && (normal_mdeg <= 8000UL) &&
            (kick_mdeg >= normal_mdeg) && (kick_mdeg <= 10000UL) &&
            (slew_mdeg_s >= 20000UL) && (slew_mdeg_s <= 720000UL)) {
            g_runtime.normal_max_angle_deg = (float) normal_mdeg * 0.001f;
            g_runtime.kick_angle_deg = (float) kick_mdeg * 0.001f;
            g_runtime.max_motor_slew_deg_s = (float) slew_mdeg_s * 0.001f;
            ++g_runtime.generation;
            reply("OK4,LIMIT");
        } else {
            reply("ERR4,LIMIT_RANGE");
        }
    } else if (sscanf(g_line, "PROFILE,%lu,%lu,%lu,%lu",
                      &cruise_rpm_x10, &accel_ms,
                      &decel_start_mm, &decel_ms) == 4) {
        if ((cruise_rpm_x10 >= 400UL) && (cruise_rpm_x10 <= 2000UL) &&
            (accel_ms >= 400UL) && (accel_ms <= 2500UL) &&
            (decel_start_mm >= 1450UL) && (decel_start_mm <= 1700UL) &&
            (decel_ms >= 400UL) && (decel_ms <= 2500UL)) {
            g_runtime.cruise_rpm = (float) cruise_rpm_x10 * 0.1f;
            g_runtime.accel_time_ms = (uint32_t) accel_ms;
            g_runtime.decel_start_mm = (float) decel_start_mm;
            g_runtime.decel_time_ms = (uint32_t) decel_ms;
            ++g_runtime.generation;
            reply("OK4,PROFILE");
        } else {
            reply("ERR4,PROFILE_RANGE");
        }
    } else {
        reply("ERR4,COMMAND");
    }
}

void h4_tuning_link_init(void)
{
    SEGGER_RTT_Init();
    g_length = 0U;
    g_pending = H4_TUNE_COMMAND_NONE;
    g_runtime.kp_deg_per_mm = H4_BALL_KP_DEG_PER_MM;
    g_runtime.ki_deg_per_mm_s = H4_BALL_KI_DEG_PER_MM_S;
    g_runtime.kd_deg_per_mm_s = H4_BALL_KD_DEG_PER_MM_S;
    g_runtime.prediction_time_s = H4_PREDICTION_TIME_S;
    g_runtime.accel_ff_deg_per_m_s2 = H4_ACCEL_FF_DEG_PER_M_S2;
    g_runtime.normal_max_angle_deg = H4_NORMAL_MAX_ANGLE_DEG;
    g_runtime.kick_angle_deg = H4_KICK_ANGLE_DEG;
    g_runtime.max_motor_slew_deg_s = H4_MOTOR_MAX_SLEW_DEG_S;
    g_runtime.cruise_rpm = H4_CRUISE_RPM;
    g_runtime.accel_time_ms = H4_ACCEL_TIME_MS;
    g_runtime.decel_start_mm = H4_DECEL_START_DISTANCE_MM;
    g_runtime.decel_time_ms = H4_DECEL_TIME_MS;
    g_runtime.generation = 0U;
    g_last_telemetry_ms = 0U;
    g_force_telemetry = true;
    reply("HELLO,H4_AB_BALANCE,1");
    reply("CMD4,ARM|START|RESET|STOP|STATUS|ZDTREAD|PID,kp1e5,ki1e5,kd1e5,predict_ms|FF,mdeg_per_m_s2|LIMIT,normal_mdeg,kick_mdeg,slew_mdeg_s|PROFILE,rpm_x10,accel_ms,decel_start_mm,decel_ms");
}

void h4_tuning_link_process(void)
{
    uint8_t byte;
    while (SEGGER_RTT_Read(0U, &byte, 1U) == 1U) {
        if ((byte == '\r') || (byte == '\n')) {
            if (g_length != 0U) {
                parse_command();
                g_length = 0U;
            }
        } else if ((byte >= 0x20U) && (byte <= 0x7EU)) {
            if (g_length < (H4_TUNE_LINE_CAPACITY - 1U)) {
                g_line[g_length++] = (char) byte;
            } else {
                g_length = 0U;
                reply("ERR4,LINE_TOO_LONG");
            }
        }
    }
}

bool h4_tuning_link_take_command(h4_tune_command_t *command)
{
    if (g_pending == H4_TUNE_COMMAND_NONE) {
        return false;
    }
    *command = g_pending;
    g_pending = H4_TUNE_COMMAND_NONE;
    return true;
}

const h4_tuning_runtime_t *h4_tuning_link_runtime(void)
{
    return &g_runtime;
}

void h4_tuning_link_event(const char *text)
{
    SEGGER_RTT_WriteString(0U, "E4,");
    reply(text);
}

void h4_tuning_link_telemetry(uint32_t now_ms, uint8_t state, bool armed,
    const h3_vision_sample_t *vision, const zdt_stepper_status_t *zdt,
    const line_sample_t *line, float predicted_mm, float motor_deg,
    float ff_deg, float distance_mm, float target_rpm,
    float measured_rpm, bool kick_active, uint32_t elapsed_ms)
{
    if (!g_force_telemetry &&
        ((now_ms - g_last_telemetry_ms) < H4_TUNE_TELEMETRY_PERIOD_MS)) {
        return;
    }
    g_force_telemetry = false;
    g_last_telemetry_ms = now_ms;
    SEGGER_RTT_printf(0U,
        "T4,%lu,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
        (unsigned long) now_ms, (unsigned) state, armed ? 1U : 0U,
        (int) scaled_float(vision->position_mm, 10.0f),
        (int) scaled_float(vision->velocity_mm_s, 10.0f),
        (int) scaled_float(predicted_mm, 10.0f),
        (int) scaled_float(motor_deg, 1000.0f),
        (int) scaled_float(ff_deg, 1000.0f),
        (int) scaled_float(distance_mm, 10.0f),
        (int) scaled_float(target_rpm, 10.0f),
        (int) scaled_float(measured_rpm, 10.0f),
        (int) line->error, (unsigned) line->black_mask,
        kick_active ? 1U : 0U, vision->valid ? 1U : 0U,
        (unsigned long) elapsed_ms,
        (unsigned long) g_runtime.generation,
        (unsigned long) zdt->tx_frames,
        (unsigned long) zdt->rx_bytes,
        (unsigned long) zdt->rx_frames,
        (unsigned long) zdt->ok_responses);
}
