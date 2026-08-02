#include "h3_tuning_link.h"

#include "h3_config.h"
#include "segger_rtt/SEGGER_RTT.h"

#include <stdio.h>
#include <string.h>

#define H3_TUNE_LINE_CAPACITY       (80U)
#define H3_TUNE_TELEMETRY_PERIOD_MS (50U)
#define H3_TUNE_GAIN_SCALE          (100000.0f)

static char g_command_line[H3_TUNE_LINE_CAPACITY];
static uint8_t g_command_length;
static h3_tune_command_t g_pending_command;
static h3_tuning_runtime_t g_runtime;
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
    unsigned long kp;
    unsigned long ki;
    unsigned long kd;
    unsigned long angle_mdeg;
    unsigned long slew_mdeg_s;

    g_command_line[g_command_length] = '\0';
    if (strcmp(g_command_line, "ARM") == 0) {
        g_pending_command = H3_TUNE_COMMAND_ARM;
        reply("OK,ARM");
    } else if (strcmp(g_command_line, "START") == 0) {
        g_pending_command = H3_TUNE_COMMAND_START;
        reply("OK,START");
    } else if (strcmp(g_command_line, "RESET") == 0) {
        g_pending_command = H3_TUNE_COMMAND_RESET;
        reply("OK,RESET");
    } else if (strcmp(g_command_line, "STOP") == 0) {
        g_pending_command = H3_TUNE_COMMAND_STOP;
        reply("OK,STOP");
    } else if (strcmp(g_command_line, "RAWTEST") == 0) {
        g_pending_command = H3_TUNE_COMMAND_RAWTEST;
        reply("OK,RAWTEST");
    } else if (strcmp(g_command_line, "RAWBACK") == 0) {
        g_pending_command = H3_TUNE_COMMAND_RAWBACK;
        reply("OK,RAWBACK");
    } else if (strcmp(g_command_line, "STATUS") == 0) {
        g_force_telemetry = true;
        reply("OK,STATUS");
    } else if (sscanf(g_command_line, "SET,%lu,%lu,%lu",
                      &kp, &ki, &kd) == 3) {
        if ((kp <= 50000UL) && (ki <= 20000UL) && (kd <= 50000UL)) {
            g_runtime.kp_deg_per_mm = (float) kp / H3_TUNE_GAIN_SCALE;
            g_runtime.ki_deg_per_mm_s = (float) ki / H3_TUNE_GAIN_SCALE;
            g_runtime.kd_deg_per_mm_s = (float) kd / H3_TUNE_GAIN_SCALE;
            ++g_runtime.generation;
            reply("OK,SET");
        } else {
            reply("ERR,SET_RANGE");
        }
    } else if (sscanf(g_command_line, "LIMIT,%lu,%lu",
                      &angle_mdeg, &slew_mdeg_s) == 2) {
        if ((angle_mdeg >= 500UL) && (angle_mdeg <= 15000UL) &&
            (slew_mdeg_s >= 10000UL) && (slew_mdeg_s <= 720000UL)) {
            g_runtime.max_motor_angle_deg = (float) angle_mdeg * 0.001f;
            g_runtime.max_motor_slew_deg_s = (float) slew_mdeg_s * 0.001f;
            ++g_runtime.generation;
            reply("OK,LIMIT");
        } else {
            reply("ERR,LIMIT_RANGE");
        }
    } else {
        reply("ERR,COMMAND");
    }
}

void h3_tuning_link_init(void)
{
    SEGGER_RTT_Init();
    g_command_length = 0U;
    g_pending_command = H3_TUNE_COMMAND_NONE;
    /* Default to the values selected by repeated physical trials. */
    g_runtime.kp_deg_per_mm = H3_BALL_KP_DEG_PER_MM;
    g_runtime.ki_deg_per_mm_s = H3_BALL_KI_DEG_PER_MM_S;
    g_runtime.kd_deg_per_mm_s = H3_BALL_KD_DEG_PER_MM_S;
    g_runtime.max_motor_angle_deg = H3_MAX_MOTOR_ANGLE_DEG;
    g_runtime.max_motor_slew_deg_s = H3_MAX_MOTOR_SLEW_DEG_S;
    g_runtime.generation = 0U;
    g_last_telemetry_ms = 0U;
    g_force_telemetry = true;
    reply("HELLO,H3_AUTOTUNE,1");
    reply("CMD,ARM|START|RESET|STOP|RAWTEST|RAWBACK|STATUS|SET,kp1e5,ki1e5,kd1e5|LIMIT,mdeg,mdeg_s");
}

void h3_tuning_link_process(void)
{
    uint8_t byte;

    while (SEGGER_RTT_Read(0U, &byte, 1U) == 1U) {
        if ((byte == '\r') || (byte == '\n')) {
            if (g_command_length != 0U) {
                parse_command();
                g_command_length = 0U;
            }
        } else if ((byte >= 0x20U) && (byte <= 0x7EU)) {
            if (g_command_length < (H3_TUNE_LINE_CAPACITY - 1U)) {
                g_command_line[g_command_length++] = (char) byte;
            } else {
                g_command_length = 0U;
                reply("ERR,LINE_TOO_LONG");
            }
        }
    }
}

bool h3_tuning_link_take_command(h3_tune_command_t *command)
{
    if (g_pending_command == H3_TUNE_COMMAND_NONE) {
        return false;
    }
    *command = g_pending_command;
    g_pending_command = H3_TUNE_COMMAND_NONE;
    return true;
}

const h3_tuning_runtime_t *h3_tuning_link_runtime(void)
{
    return &g_runtime;
}

void h3_tuning_link_telemetry(uint32_t now_ms, uint8_t state,
                              bool armed,
                              const h3_vision_sample_t *vision,
                              const zdt_stepper_status_t *zdt,
                              float target_mm, float motor_angle_deg,
                              uint32_t elapsed_ms)
{
    if (!g_force_telemetry &&
        ((now_ms - g_last_telemetry_ms) < H3_TUNE_TELEMETRY_PERIOD_MS)) {
        return;
    }
    g_force_telemetry = false;
    g_last_telemetry_ms = now_ms;
    SEGGER_RTT_printf(0U,
        "T,%lu,%u,%u,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%ld,%lu,%lu\r\n",
        (unsigned long) now_ms,
        (unsigned) state,
        armed ? 1U : 0U,
        (int) scaled_float(vision->position_mm, 10.0f),
        (int) scaled_float(vision->velocity_mm_s, 10.0f),
        (int) scaled_float(target_mm, 10.0f),
        (int) scaled_float(motor_angle_deg, 1000.0f),
        (unsigned long) scaled_float(g_runtime.kp_deg_per_mm,
                                     H3_TUNE_GAIN_SCALE),
        (unsigned long) scaled_float(g_runtime.ki_deg_per_mm_s,
                                     H3_TUNE_GAIN_SCALE),
        (unsigned long) scaled_float(g_runtime.kd_deg_per_mm_s,
                                     H3_TUNE_GAIN_SCALE),
        (unsigned long) g_runtime.generation,
        vision->valid ? 1U : 0U,
        (unsigned long) elapsed_ms,
        (unsigned long) zdt->tx_frames,
        (unsigned long) zdt->rx_frames,
        (unsigned long) zdt->ok_responses,
        (unsigned long) zdt->parameter_errors,
        (unsigned long) zdt->format_errors,
        (unsigned) zdt->last_function,
        (unsigned) zdt->last_status,
        (unsigned) zdt->motor_flags,
        (long) zdt->real_position_raw,
        (unsigned long) zdt->status_read_frames,
        (unsigned long) zdt->position_read_frames);
}

void h3_tuning_link_event(const char *text)
{
    SEGGER_RTT_WriteString(0U, "E,");
    reply(text);
}
