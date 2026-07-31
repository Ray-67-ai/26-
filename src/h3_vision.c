#include "h3_vision.h"

#include "h3_config.h"

#include <stdio.h>

#define VISION_RX_BUFFER_SIZE         (128U)
#define VISION_RX_BUFFER_MASK         (VISION_RX_BUFFER_SIZE - 1U)
#define VISION_LINE_SIZE              (64U)

static volatile uint8_t g_rx_buffer[VISION_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static char g_line[VISION_LINE_SIZE];
static uint8_t g_line_length;
static bool g_new_sample;
static h3_vision_sample_t g_sample;

static bool sequence_is_new(uint32_t sequence)
{
    return !g_sample.has_frame || (sequence != g_sample.sequence);
}

static void parse_line(uint32_t now_ms)
{
    unsigned long sequence;
    unsigned long camera_ms;
    long position_mm;
    unsigned int valid;
    int fields;
    float dt_s;
    float previous_position;
    float filtered_position;
    float raw_velocity;

    g_line[g_line_length] = '\0';
    fields = sscanf(g_line, "B,%lu,%lu,%ld,%u",
                    &sequence, &camera_ms, &position_mm, &valid);
    if ((fields != 4) || (valid > 1U) ||
        (position_mm < -200L) || (position_mm > 200L)) {
        ++g_sample.bad_frames;
        return;
    }
    if (!sequence_is_new((uint32_t) sequence)) {
        return;
    }

    dt_s = 0.0f;
    if (g_sample.has_frame) {
        uint32_t camera_dt = (uint32_t) camera_ms - g_sample.camera_ms;
        uint32_t receive_dt = now_ms - g_sample.received_ms;
        if ((camera_dt >= 5U) && (camera_dt <= 200U)) {
            dt_s = (float) camera_dt * 0.001f;
        } else if ((receive_dt >= 5U) && (receive_dt <= 200U)) {
            dt_s = (float) receive_dt * 0.001f;
        }
    }

    g_sample.sequence = (uint32_t) sequence;
    g_sample.camera_ms = (uint32_t) camera_ms;
    g_sample.received_ms = now_ms;
    g_sample.position_raw_mm = (float) position_mm;
    g_sample.valid = (valid != 0U);

    if (!g_sample.valid) {
        g_sample.has_frame = true;
        ++g_sample.bad_frames;
        g_new_sample = true;
        return;
    }

    g_sample.last_valid_ms = now_ms;

    if (!g_sample.has_frame || (dt_s <= 0.0f)) {
        g_sample.position_mm = g_sample.position_raw_mm;
        g_sample.velocity_mm_s = 0.0f;
        g_sample.sample_dt_s = 0.0f;
    } else {
        previous_position = g_sample.position_mm;
        filtered_position = previous_position +
            H3_POSITION_FILTER_ALPHA *
            (g_sample.position_raw_mm - previous_position);
        raw_velocity = (filtered_position - previous_position) / dt_s;
        g_sample.position_mm = filtered_position;
        g_sample.velocity_mm_s += H3_VELOCITY_FILTER_ALPHA *
            (raw_velocity - g_sample.velocity_mm_s);
        g_sample.sample_dt_s = dt_s;
    }

    g_sample.has_frame = true;
    ++g_sample.good_frames;
    g_new_sample = true;
}

void h3_vision_init(void)
{
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_line_length = 0U;
    g_new_sample = false;
    g_sample = (h3_vision_sample_t) {0};
}

void h3_vision_rx_byte_isr(uint8_t byte)
{
    uint16_t head = g_rx_head;
    uint16_t next = (head + 1U) & VISION_RX_BUFFER_MASK;

    if (next == g_rx_tail) {
        ++g_sample.dropped_bytes;
        return;
    }
    g_rx_buffer[head] = byte;
    g_rx_head = next;
}

void h3_vision_process(uint32_t now_ms)
{
    while (g_rx_tail != g_rx_head) {
        uint8_t byte = g_rx_buffer[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1U) & VISION_RX_BUFFER_MASK;

        if ((byte == '\n') || (byte == '\r')) {
            if (g_line_length > 0U) {
                parse_line(now_ms);
                g_line_length = 0U;
            }
        } else if (g_line_length < (VISION_LINE_SIZE - 1U)) {
            g_line[g_line_length++] = (char) byte;
        } else {
            g_line_length = 0U;
            ++g_sample.bad_frames;
        }
    }
}

bool h3_vision_take_new_sample(void)
{
    bool result = g_new_sample;
    g_new_sample = false;
    return result;
}

const h3_vision_sample_t *h3_vision_get(void)
{
    return &g_sample;
}
