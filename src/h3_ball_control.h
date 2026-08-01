#ifndef H3_BALL_CONTROL_H
#define H3_BALL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

void h3_ball_control_init(void);
void h3_ball_control_process(void);
void h3_ball_control_tick_1ms_isr(void);
void h3_ball_control_start_key_isr(void);
void h3_ball_control_zdt_rx_isr(uint8_t byte);
void h3_ball_control_vision_rx_isr(uint8_t byte);
bool h3_ball_control_vision_ready(void);

#endif
