#include "ti_msp_dl_config.h"
#include "src/app.h"
#include "src/motor_encoder.h"

int main(void)
{
    SYSCFG_DL_init();
    app_init();

    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(USER_KEY_INT_IRQN);
    __enable_irq();

    while (1) {
        app_process();
        __WFI();
    }
}

void CONTROL_TIMER_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(CONTROL_TIMER_INST)) {
        case DL_TIMER_IIDX_LOAD:
            app_tick_1ms_isr();
            break;
        default:
            break;
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case ENCODER_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(ENCODER_PORT)) {
                case ENCODER_RIGHT_A_IIDX:
                    motor_encoder_right_edge_isr();
                    break;
                case ENCODER_LEFT_A_IIDX:
                    motor_encoder_left_edge_isr();
                    break;
                default:
                    break;
            }
            break;

        case USER_KEY_INT_IIDX:
            if (DL_GPIO_getPendingInterrupt(USER_KEY_PORT) ==
                USER_KEY_START_IIDX) {
                app_start_key_isr();
            }
            break;

        default:
            break;
    }
}
