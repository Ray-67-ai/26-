#include "ti_msp_dl_config.h"

#include "src/app.h"
#include "src/competition_mode.h"
#include "src/h3_ball_control.h"
#include "src/h3_config.h"
#include "src/h4_balance_ab.h"
#include "src/h4_config.h"
#include "src/motor_encoder.h"
#include "src/ssd1306.h"

#include <stdbool.h>
#include <stdint.h>

#define KEY_EVENT_Q1 (1U << 0)
#define KEY_EVENT_Q2 (1U << 1)
#define KEY_EVENT_Q3 (1U << 2)
#define KEY_EVENT_Q4 (1U << 3)
#define KEY_DEBOUNCE_MS (20U)

#define VISION_UART_CLEAR_MASK                                             \
    (DL_UART_MAIN_INTERRUPT_RX |                                          \
     DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |                            \
     DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                               \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                                 \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                                \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                               \
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)

#define ZDT_UART_CLEAR_MASK                                                \
    (DL_UART_MAIN_INTERRUPT_RX |                                          \
     DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |                            \
     DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                               \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                                 \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                                \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                               \
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)

static volatile uint32_t g_system_ms;
static volatile uint8_t g_key_events;
static uint8_t g_key_stable_pressed;
static uint8_t g_key_debounce_count[4];
static competition_mode_t g_selected_mode;
static competition_mode_t g_active_mode;
static bool g_h3_start_pending;
static bool g_h4_start_pending;
static bool g_dispatch_key_seen;
static uint32_t g_last_dispatch_key_ms;
static uint32_t g_last_menu_refresh_ms;

static void draw_mode_menu(void)
{
    uint8_t page;

    if (!ssd1306_is_present()) {
        return;
    }
    ssd1306_clear();
    ssd1306_draw_text(0U, 0U, "H MODE SELECT");
    if (g_selected_mode == COMPETITION_MODE_H4) {
        ssd1306_draw_text(0U, 2U, "SELECT: H4 AB");
    } else if (g_selected_mode == COMPETITION_MODE_H3) {
        ssd1306_draw_text(0U, 2U, "SELECT: H3 BALL");
    } else {
        ssd1306_draw_text(0U, 2U, "SELECT: H2 LINE");
    }
    ssd1306_draw_text(0U, 5U, "Q1: NEXT");
    ssd1306_draw_text(0U, 6U, "Q3: START");
    for (page = 0U; page < 8U; ++page) {
        (void) ssd1306_refresh_next_page();
    }
}

/*
 * MaixCAM may already be transmitting before Q2 is pressed. Discard the
 * stale/overflowed fragment and start from its next complete B,... frame.
 */
static void vision_uart_start_clean(void)
{
    NVIC_DisableIRQ(VISION_UART_INST_INT_IRQN);
    while (!DL_UART_Main_isRXFIFOEmpty(VISION_UART_INST)) {
        (void) DL_UART_Main_receiveData(VISION_UART_INST);
    }
    DL_UART_Main_clearInterruptStatus(
        VISION_UART_INST, VISION_UART_CLEAR_MASK);
    NVIC_ClearPendingIRQ(VISION_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
}

/*
 * H3 initialization can send ZDT commands before the UART0 NVIC line is
 * enabled. The motor replies immediately, so drain and parse any replies
 * already waiting in the FIFO before clearing error/pending state. This also
 * recovers from an RX FIFO overrun caused by those early replies.
 */
static void zdt_uart_start_clean(void)
{
    NVIC_DisableIRQ(DEBUG_UART_INST_INT_IRQN);
    while (!DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)) {
        h3_ball_control_zdt_rx_isr(
            (uint8_t) DL_UART_Main_receiveData(DEBUG_UART_INST));
    }
    DL_UART_Main_clearInterruptStatus(
        DEBUG_UART_INST, ZDT_UART_CLEAR_MASK);
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
}

static uint8_t take_key_events(void)
{
    uint8_t events;

    __disable_irq();
    events = g_key_events;
    g_key_events = 0U;
    __enable_irq();
    return events;
}

static uint8_t read_pressed_keys(void)
{
    uint8_t pressed = 0U;

    /* The extension-board keys are active low. */
    if (DL_GPIO_readPins(USER_KEY_Q1_H2_PORT,
                         USER_KEY_Q1_H2_PIN) == 0U) {
        pressed |= KEY_EVENT_Q1;
    }
    if (DL_GPIO_readPins(USER_KEY_Q3_START_PORT,
                         USER_KEY_Q3_START_PIN) == 0U) {
        pressed |= KEY_EVENT_Q3;
    }
    if (DL_GPIO_readPins(USER_KEY_Q4_H5_PORT,
                         USER_KEY_Q4_H5_PIN) == 0U) {
        pressed |= KEY_EVENT_Q4;
    }
    return pressed;
}

static void poll_keys_1ms_isr(void)
{
    uint8_t sample = read_pressed_keys();
    uint8_t i;

    for (i = 0U; i < 4U; ++i) {
        uint8_t mask = (uint8_t) (1U << i);
        bool sampled_pressed = (sample & mask) != 0U;
        bool stable_pressed = (g_key_stable_pressed & mask) != 0U;

        if (sampled_pressed == stable_pressed) {
            g_key_debounce_count[i] = 0U;
        } else {
            if (g_key_debounce_count[i] < KEY_DEBOUNCE_MS) {
                ++g_key_debounce_count[i];
            }
            if (g_key_debounce_count[i] >= KEY_DEBOUNCE_MS) {
                g_key_debounce_count[i] = 0U;
                if (sampled_pressed) {
                    g_key_stable_pressed |= mask;
                    g_key_events |= mask;
                } else {
                    g_key_stable_pressed &= (uint8_t) ~mask;
                }
            }
        }
    }
}

static bool dispatch_key_debounced(uint32_t now)
{
    if (g_dispatch_key_seen &&
        ((now - g_last_dispatch_key_ms) < 150U)) {
        return false;
    }
    g_dispatch_key_seen = true;
    g_last_dispatch_key_ms = now;
    return true;
}

static void start_selected_mode(void)
{
    g_active_mode = g_selected_mode;
    if (g_active_mode == COMPETITION_MODE_H2) {
        app_start_key_isr();
    } else if (g_active_mode == COMPETITION_MODE_H3) {
        g_h3_start_pending = true;
        vision_uart_start_clean();
    } else if (g_active_mode == COMPETITION_MODE_H4) {
        g_h4_start_pending = true;
        vision_uart_start_clean();
    }
}

static void select_or_trigger_mode(void)
{
    uint8_t events = take_key_events();
    uint32_t now = g_system_ms;

    if (events == 0U) {
        return;
    }

    if (g_active_mode == COMPETITION_MODE_NONE) {
        if (!dispatch_key_debounced(now)) {
            return;
        }
        if ((events & KEY_EVENT_Q1) != 0U) {
            if (g_selected_mode == COMPETITION_MODE_H2) {
                g_selected_mode = COMPETITION_MODE_H3;
            } else if (g_selected_mode == COMPETITION_MODE_H3) {
                g_selected_mode = COMPETITION_MODE_H4;
            } else {
                g_selected_mode = COMPETITION_MODE_H2;
            }
            draw_mode_menu();
        } else if ((events & KEY_EVENT_Q3) != 0U) {
            start_selected_mode();
        }
        return;
    }

    /* Lock the selected question until RESET; Q3 remains its action key. */
    if ((events & KEY_EVENT_Q3) != 0U) {
        if (g_active_mode == COMPETITION_MODE_H2) {
            app_start_key_isr();
        } else if (g_active_mode == COMPETITION_MODE_H3) {
            g_h3_start_pending = true;
        } else if (g_active_mode == COMPETITION_MODE_H4) {
            g_h4_start_pending = true;
        }
    }
}

int main(void)
{
    SYSCFG_DL_init();

    g_system_ms = 0U;
    g_key_events = 0U;
    g_key_stable_pressed = read_pressed_keys();
    g_key_debounce_count[0] = 0U;
    g_key_debounce_count[1] = 0U;
    g_key_debounce_count[2] = 0U;
    g_key_debounce_count[3] = 0U;
#if H3_AUTOTUNE_BUILD || H3_STANDALONE_BUILD
    g_selected_mode = COMPETITION_MODE_H3;
    g_active_mode = COMPETITION_MODE_H3;
#elif H4_TUNING_BUILD
    g_selected_mode = COMPETITION_MODE_H4;
    g_active_mode = COMPETITION_MODE_H4;
#else
    g_selected_mode = COMPETITION_MODE_H2;
    g_active_mode = COMPETITION_MODE_NONE;
#endif
    g_h3_start_pending = false;
    g_h4_start_pending = false;
    g_dispatch_key_seen = false;
    g_last_dispatch_key_ms = 0U;
    g_last_menu_refresh_ms = 0U;

    /* Initialize both question modules once; only the selected one runs. */
    app_init();
    h3_ball_control_init();
    h4_balance_ab_init();
#if !H3_AUTOTUNE_BUILD && !H3_STANDALONE_BUILD && !H4_TUNING_BUILD
    draw_mode_menu();
#endif

    zdt_uart_start_clean();
    NVIC_ClearPendingIRQ(VISION_UART_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);

    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
#if H3_AUTOTUNE_BUILD || H3_STANDALONE_BUILD || H4_TUNING_BUILD
    NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
#else
    /* Vision UART IRQ is enabled only after Q3 starts selected H3. */
    NVIC_DisableIRQ(VISION_UART_INST_INT_IRQN);
#endif
    __enable_irq();

    while (1) {
        select_or_trigger_mode();

        if (g_active_mode == COMPETITION_MODE_H2) {
            app_process();
        } else if (g_active_mode == COMPETITION_MODE_H3) {
            h3_ball_control_process();
            if (g_h3_start_pending && h3_ball_control_vision_ready()) {
                g_h3_start_pending = false;
                h3_ball_control_start_key_isr();
            }
        } else if (g_active_mode == COMPETITION_MODE_H4) {
            h4_balance_ab_process();
            if (g_h4_start_pending && h4_balance_ab_vision_ready()) {
                g_h4_start_pending = false;
                h4_balance_ab_start_key_isr();
            }
        } else if (ssd1306_is_present() &&
                   ((g_system_ms - g_last_menu_refresh_ms) >= 12U)) {
            g_last_menu_refresh_ms = g_system_ms;
            (void) ssd1306_refresh_next_page();
        }
        __WFI();
    }
}

void CONTROL_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(CONTROL_TIMER_INST) ==
        DL_TIMER_IIDX_LOAD) {
        ++g_system_ms;
        poll_keys_1ms_isr();
        app_tick_1ms_isr();
        h3_ball_control_tick_1ms_isr();
        h4_balance_ab_tick_1ms_isr();
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case ENCODER_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
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

        default:
            break;
    }
}

void DEBUG_UART_INST_IRQHandler(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)) {
        h3_ball_control_zdt_rx_isr(
            (uint8_t) DL_UART_Main_receiveData(DEBUG_UART_INST));
    }
    DL_UART_Main_clearInterruptStatus(
        DEBUG_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
}

void VISION_UART_INST_IRQHandler(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(VISION_UART_INST)) {
        h3_ball_control_vision_rx_isr(
            (uint8_t) DL_UART_Main_receiveData(VISION_UART_INST));
    }
    DL_UART_Main_clearInterruptStatus(
        VISION_UART_INST, VISION_UART_CLEAR_MASK);
}
