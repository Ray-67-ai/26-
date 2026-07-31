#ifndef ZDT_STEPPER_H
#define ZDT_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    volatile uint32_t tx_frames;
    volatile uint32_t tx_errors;
    volatile uint32_t rx_frames;
    volatile uint8_t last_function;
    volatile uint8_t last_status;
} zdt_stepper_status_t;

void zdt_stepper_init(void);
bool zdt_stepper_enable(bool enable);
bool zdt_stepper_stop_now(void);
bool zdt_stepper_move_absolute_deg(float motor_deg);
void zdt_stepper_rx_byte_isr(uint8_t byte);
const zdt_stepper_status_t *zdt_stepper_get_status(void);

#endif
