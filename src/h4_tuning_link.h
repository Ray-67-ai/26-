#ifndef H4_TUNING_LINK_H
#define H4_TUNING_LINK_H

#include "h3_vision.h"
#include "line_sensor.h"
#include "zdt_stepper.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    H4_TUNE_COMMAND_NONE = 0,
    H4_TUNE_COMMAND_ARM,
    H4_TUNE_COMMAND_START,
    H4_TUNE_COMMAND_RESET,
    H4_TUNE_COMMAND_STOP
} h4_tune_command_t;

typedef struct {
    float kp_deg_per_mm;
    float ki_deg_per_mm_s;
    float kd_deg_per_mm_s;
    float prediction_time_s;
    float accel_ff_deg_per_m_s2;
    float normal_max_angle_deg;
    float kick_angle_deg;
    float max_motor_slew_deg_s;
    float cruise_rpm;
    uint32_t accel_time_ms;
    float decel_start_mm;
    uint32_t decel_time_ms;
    uint32_t generation;
} h4_tuning_runtime_t;

void h4_tuning_link_init(void);
void h4_tuning_link_process(void);
bool h4_tuning_link_take_command(h4_tune_command_t *command);
const h4_tuning_runtime_t *h4_tuning_link_runtime(void);
void h4_tuning_link_event(const char *text);
void h4_tuning_link_telemetry(uint32_t now_ms, uint8_t state, bool armed,
    const h3_vision_sample_t *vision, const zdt_stepper_status_t *zdt,
    const line_sample_t *line, float predicted_mm, float motor_deg,
    float ff_deg, float distance_mm, float target_rpm,
    float measured_rpm, bool kick_active, uint32_t elapsed_ms);

#endif
