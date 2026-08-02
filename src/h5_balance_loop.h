#ifndef H5_BALANCE_LOOP_H
#define H5_BALANCE_LOOP_H

#include <stdbool.h>
#include <stdint.h>

void h5_balance_loop_init(void);
void h5_balance_loop_process(void);
void h5_balance_loop_tick_1ms_isr(void);
void h5_balance_loop_start_key_isr(void);
bool h5_balance_loop_vision_ready(void);

#endif
