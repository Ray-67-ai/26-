#ifndef ZDT_STEPPER_H
#define ZDT_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    volatile uint32_t tx_frames;
    volatile uint32_t tx_errors;
    volatile uint32_t rx_bytes;
    volatile uint32_t rx_frames;
    volatile uint32_t ok_responses;
    volatile uint32_t parameter_errors;
    volatile uint32_t format_errors;
    volatile uint8_t last_function;
    volatile uint8_t last_status;
    volatile uint8_t motor_flags;
    volatile int32_t real_position_raw;
    volatile uint32_t status_read_frames;
    volatile uint32_t position_read_frames;
} zdt_stepper_status_t;

void zdt_stepper_init(void);
bool zdt_stepper_enable(bool enable);
bool zdt_stepper_stop_now(void);
bool zdt_stepper_move_absolute_deg(float motor_deg);
bool zdt_stepper_read_status_flags(void);
bool zdt_stepper_read_real_position(void);
bool zdt_stepper_read_driver_config(void);
bool zdt_stepper_send_reference_test(void);
bool zdt_stepper_send_reference_back(void);
void zdt_stepper_rx_byte_isr(uint8_t byte);
const zdt_stepper_status_t *zdt_stepper_get_status(void);

#endif
