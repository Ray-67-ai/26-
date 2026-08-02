#ifndef H3_TUNING_LINK_H
#define H3_TUNING_LINK_H

#include "h3_vision.h"
#include "zdt_stepper.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    H3_TUNE_COMMAND_NONE = 0,
    H3_TUNE_COMMAND_ARM,
    H3_TUNE_COMMAND_START,
    H3_TUNE_COMMAND_RESET,
    H3_TUNE_COMMAND_STOP,
    H3_TUNE_COMMAND_RAWTEST,
    H3_TUNE_COMMAND_RAWBACK
} h3_tune_command_t;

typedef struct {
    float kp_deg_per_mm;
    float ki_deg_per_mm_s;
    float kd_deg_per_mm_s;
    float max_motor_angle_deg;
    float max_motor_slew_deg_s;
    uint32_t generation;
} h3_tuning_runtime_t;

void h3_tuning_link_init(void);
void h3_tuning_link_process(void);
bool h3_tuning_link_take_command(h3_tune_command_t *command);
const h3_tuning_runtime_t *h3_tuning_link_runtime(void);
void h3_tuning_link_telemetry(uint32_t now_ms, uint8_t state,
                              bool armed,
                              const h3_vision_sample_t *vision,
                              const zdt_stepper_status_t *zdt,
                              float target_mm, float motor_angle_deg,
                              uint32_t elapsed_ms);
void h3_tuning_link_event(const char *text);

#endif
