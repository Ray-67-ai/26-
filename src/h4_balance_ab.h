    #ifndef H4_BALANCE_AB_H
#define H4_BALANCE_AB_H

#include <stdbool.h>
#include <stdint.h>

void h4_balance_ab_init(void);
void h4_balance_ab_process(void);
void h4_balance_ab_tick_1ms_isr(void);
void h4_balance_ab_start_key_isr(void);
bool h4_balance_ab_vision_ready(void);

#endif
