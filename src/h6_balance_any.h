#ifndef H6_BALANCE_ANY_H
#define H6_BALANCE_ANY_H

#include <stdbool.h>
#include <stdint.h>

void h6_balance_any_init(void);
void h6_balance_any_process(void);
void h6_balance_any_tick_1ms_isr(void);
void h6_balance_any_start_key_isr(void);
bool h6_balance_any_vision_ready(void);

#endif
