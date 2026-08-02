#include "zdt_stepper.h"

#include "h3_config.h"
#include "ti_msp_dl_config.h"

#define ZDT_FRAME_END                 (0x6BU)
#define ZDT_RX_FRAME_CAPACITY         (16U)

static volatile uint8_t g_rx_frame[ZDT_RX_FRAME_CAPACITY];
static volatile uint8_t g_rx_length;
static zdt_stepper_status_t g_status;

static bool send_frame(const uint8_t *frame, uint8_t length)
{
    uint8_t i;

    for (i = 0U; i < length; ++i) {
        /* Wait until each byte leaves the UART. Together with the frame gap
         * below this prevents adjacent ZDT commands from being merged; the
         * motor reported 01 00 EE 6B before this timing fix. */
        DL_UART_Main_transmitDataBlocking(DEBUG_UART_INST, frame[i]);
    }
    delay_cycles(CPUCLK_FREQ / 500U); /* 2 ms at 32 MHz. */
    ++g_status.tx_frames;
    return true;
}

void zdt_stepper_init(void)
{
    uint8_t i;

    g_rx_length = 0U;
    g_status = (zdt_stepper_status_t) {0};
    for (i = 0U; i < ZDT_RX_FRAME_CAPACITY; ++i) {
        g_rx_frame[i] = 0U;
    }
}

bool zdt_stepper_enable(bool enable)
{
    const uint8_t frame[6] = {
        H3_ZDT_ADDRESS, 0xF3U, 0xABU,
        enable ? 0x01U : 0x00U, 0x00U, ZDT_FRAME_END
    };
    return send_frame(frame, (uint8_t) sizeof(frame));
}

bool zdt_stepper_stop_now(void)
{
    const uint8_t frame[5] = {
        H3_ZDT_ADDRESS, 0xFEU, 0x98U, 0x00U, ZDT_FRAME_END
    };
    return send_frame(frame, (uint8_t) sizeof(frame));
}

bool zdt_stepper_move_absolute_deg(float motor_deg)
{
    uint8_t direction;
    uint32_t pulses;
    float magnitude = motor_deg;
    uint8_t frame[13];

#if !H3_ZDT_CW_RAISES_RIGHT
    magnitude = -magnitude;
#endif

    if (magnitude < 0.0f) {
        direction = 1U; /* CCW */
        magnitude = -magnitude;
    } else {
        direction = 0U; /* CW */
    }

    pulses = (uint32_t)
        (magnitude * H3_ZDT_PULSES_PER_REV / 360.0f + 0.5f);

    frame[0] = H3_ZDT_ADDRESS;
    frame[1] = 0xFDU;
    frame[2] = direction;
    frame[3] = (uint8_t) (H3_ZDT_SPEED_RPM >> 8);
    frame[4] = (uint8_t) H3_ZDT_SPEED_RPM;
    frame[5] = H3_ZDT_ACCELERATION;
    frame[6] = (uint8_t) (pulses >> 24);
    frame[7] = (uint8_t) (pulses >> 16);
    frame[8] = (uint8_t) (pulses >> 8);
    frame[9] = (uint8_t) pulses;
    frame[10] = 0x01U; /* 绝对位置模式 */
    frame[11] = 0x00U; /* 单机立即执行 */
    frame[12] = ZDT_FRAME_END;
    return send_frame(frame, (uint8_t) sizeof(frame));
}

void zdt_stepper_rx_byte_isr(uint8_t byte)
{
    uint8_t length = g_rx_length;

    if (length >= ZDT_RX_FRAME_CAPACITY) {
        length = 0U;
    }
    g_rx_frame[length++] = byte;

    if (byte == ZDT_FRAME_END) {
        if ((length >= 4U) && (g_rx_frame[0] == H3_ZDT_ADDRESS)) {
            g_status.last_function = g_rx_frame[1];
            g_status.last_status = g_rx_frame[length - 2U];
            ++g_status.rx_frames;
        }
        length = 0U;
    }
    g_rx_length = length;
}

const zdt_stepper_status_t *zdt_stepper_get_status(void)
{
    return &g_status;
}
