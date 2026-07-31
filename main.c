#include "ti_msp_dl_config.h"
#include "src/competition_mode.h"

#if COMPETITION_MODE == COMPETITION_MODE_H2
#include "src/app.h"
#include "src/motor_encoder.h"
#elif COMPETITION_MODE == COMPETITION_MODE_H3
#include "src/h3_ball_control.h"
#else
#error "Unsupported COMPETITION_MODE"
#endif

int main(void)
{
    SYSCFG_DL_init();

#if COMPETITION_MODE == COMPETITION_MODE_H2
    app_init();
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(USER_KEY_INT_IRQN);
#else
    h3_ball_control_init();
    DL_UART_Main_enableInterrupt(
    VISION_UART_INST,
    DL_UART_MAIN_INTERRUPT_RX |
    DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(VISION_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(USER_KEY_INT_IRQN);
    NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
#endif
    __enable_irq();

    while (1) {
#if COMPETITION_MODE == COMPETITION_MODE_H2
        app_process();
#else
        h3_ball_control_process();
#endif
        __WFI();
    }
}

void CONTROL_TIMER_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(CONTROL_TIMER_INST)) {
        case DL_TIMER_IIDX_LOAD:
#if COMPETITION_MODE == COMPETITION_MODE_H2
            app_tick_1ms_isr();
#else
            h3_ball_control_tick_1ms_isr();
#endif
            break;
        default:
            break;
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
#if COMPETITION_MODE == COMPETITION_MODE_H2
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
#endif

        case USER_KEY_INT_IIDX:
            if (DL_GPIO_getPendingInterrupt(USER_KEY_PORT) ==
                USER_KEY_START_IIDX) {
#if COMPETITION_MODE == COMPETITION_MODE_H2
                app_start_key_isr();
#else
                h3_ball_control_start_key_isr();
#endif
            }
            break;

        default:
            break;
    }
}

#if COMPETITION_MODE == COMPETITION_MODE_H3
void DEBUG_UART_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(DEBUG_UART_INST) ==
        DL_UART_IIDX_RX) {
        h3_ball_control_zdt_rx_isr(
            (uint8_t) DL_UART_Main_receiveData(DEBUG_UART_INST));
        DL_UART_Main_clearInterruptStatus(
            DEBUG_UART_INST, DL_UART_INTERRUPT_RX);
    }
}

void VISION_UART_INST_IRQHandler(void)
{
    /*
     * 一次性读空UART接收FIFO。
     * 不要每次中断只读取一个字节，否则连续ASCII帧容易丢字节。
     */
    while (!DL_UART_Main_isRXFIFOEmpty(VISION_UART_INST)) {
        h3_ball_control_vision_rx_isr(
            (uint8_t) DL_UART_Main_receiveData(VISION_UART_INST));
    }

    DL_UART_Main_clearInterruptStatus(
        VISION_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
}
#endif
