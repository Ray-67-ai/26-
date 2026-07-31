#ifndef H3_VISION_H
#define H3_VISION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    bool has_frame;
    uint32_t sequence;
    uint32_t camera_ms;
    uint32_t received_ms;
    uint32_t last_valid_ms;
    float position_raw_mm;
    float position_mm;
    float velocity_mm_s;
    float sample_dt_s;
    uint32_t good_frames;
    uint32_t invalid_frames;
    uint32_t bad_frames;
    uint32_t resync_events;
    volatile uint32_t rx_bytes;
    volatile uint32_t dropped_bytes;
} h3_vision_sample_t;

void h3_vision_init(void);
void h3_vision_rx_byte_isr(uint8_t byte);
void h3_vision_process(uint32_t now_ms);
bool h3_vision_take_new_sample(void);
const h3_vision_sample_t *h3_vision_get(void);

#endif
