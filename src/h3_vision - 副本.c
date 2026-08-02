#include "h3_vision.h"

#include "h3_config.h"

#define VISION_RX_BUFFER_SIZE (128U)
#define VISION_RX_BUFFER_MASK (VISION_RX_BUFFER_SIZE - 1U)
#define VISION_LINE_SIZE      (64U)

static volatile uint8_t g_rx_buffer[VISION_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static char g_line[VISION_LINE_SIZE];
static uint8_t g_line_length;
static bool g_new_sample;
static h3_vision_sample_t g_sample;

static bool inside_legacy_checksum(void)
{
    uint8_t i;
    if ((g_line_length < 5U) ||
        (g_line[0] != 'B') || (g_line[1] != 'A') ||
        (g_line[2] != 'L') || (g_line[3] != 'L') ||
        (g_line[4] != ',')) {
        return false;
    }
    for (i = 5U; i < g_line_length; ++i) {
        if (g_line[i] == '*') {
            return true;
        }
    }
    return false;
}

static bool parse_u32(const char **cursor, char delimiter, uint32_t *value)
{
    const char *p = *cursor;
    uint32_t result = 0U;
    uint8_t digits = 0U;

    while ((*p >= '0') && (*p <= '9')) {
        uint32_t digit = (uint32_t) (*p - '0');
        if (result > ((UINT32_MAX - digit) / 10U)) {
            return false;
        }
        result = result * 10U + digit;
        ++digits;
        ++p;
    }
    if ((digits == 0U) || (*p != delimiter)) {
        return false;
    }
    *value = result;
    *cursor = p + ((delimiter == '\0') ? 0U : 1U);
    return true;
}

static bool parse_i32(const char **cursor, char delimiter, int32_t *value)
{
    const char *p = *cursor;
    bool negative = false;
    uint32_t magnitude;

    if ((*p == '-') || (*p == '+')) {
        negative = (*p == '-');
        ++p;
    }
    if (!parse_u32(&p, delimiter, &magnitude) || (magnitude > 200U)) {
        return false;
    }
    *value = negative ? -(int32_t) magnitude : (int32_t) magnitude;
    *cursor = p;
    return true;
}

static bool parse_decimal_i32(const char **cursor, char delimiter,
                              int32_t *value)
{
    const char *p = *cursor;
    bool negative = false;
    uint32_t magnitude = 0U;
    uint8_t digits = 0U;
    bool round_up = false;

    if ((*p == '-') || (*p == '+')) {
        negative = (*p == '-');
        ++p;
    }
    while ((*p >= '0') && (*p <= '9')) {
        magnitude = magnitude * 10U + (uint32_t) (*p - '0');
        ++digits;
        ++p;
        if (magnitude > 200U) {
            return false;
        }
    }
    if (*p == '.') {
        ++p;
        if ((*p < '0') || (*p > '9')) {
            return false;
        }
        round_up = (*p >= '5');
        while ((*p >= '0') && (*p <= '9')) {
            ++p;
        }
    }
    if ((digits == 0U) || (*p != delimiter)) {
        return false;
    }
    if (round_up) {
        ++magnitude;
    }
    if (magnitude > 200U) {
        return false;
    }
    *value = negative ? -(int32_t) magnitude : (int32_t) magnitude;
    *cursor = p + 1;
    return true;
}

static bool skip_field(const char **cursor)
{
    const char *p = *cursor;
    if ((*p == '\0') || (*p == ',')) {
        return false;
    }
    while ((*p != '\0') && (*p != ',')) {
        ++p;
    }
    if (*p != ',') {
        return false;
    }
    *cursor = p + 1;
    return true;
}

static void commit_frame(uint32_t sequence, uint32_t camera_ms,
                         int32_t position_mm, uint32_t valid,
                         uint32_t now_ms)
{
    float dt_s = 0.0f;

    if (g_sample.has_frame && (sequence == g_sample.sequence)) {
        return;
    }
    if (g_sample.has_frame) {
        uint32_t camera_dt = camera_ms - g_sample.camera_ms;
        uint32_t receive_dt = now_ms - g_sample.received_ms;
        if ((camera_dt >= 5U) && (camera_dt <= 200U)) {
            dt_s = (float) camera_dt * 0.001f;
        } else if ((receive_dt >= 5U) && (receive_dt <= 200U)) {
            dt_s = (float) receive_dt * 0.001f;
        }
    }

    g_sample.sequence = sequence;
    g_sample.camera_ms = camera_ms;
    g_sample.received_ms = now_ms;
    g_sample.position_raw_mm = (float) position_mm;
    g_sample.valid = (valid != 0U);
    g_sample.has_frame = true;

    if (!g_sample.valid) {
        ++g_sample.invalid_frames;
        g_new_sample = true;
        return;
    }

    g_sample.last_valid_ms = now_ms;
    if ((g_sample.good_frames == 0U) || (dt_s <= 0.0f)) {
        g_sample.position_mm = g_sample.position_raw_mm;
        g_sample.velocity_mm_s = 0.0f;
        g_sample.sample_dt_s = 0.0f;
    } else {
        float previous_position = g_sample.position_mm;
        float filtered_position = previous_position +
            H3_POSITION_FILTER_ALPHA *
            (g_sample.position_raw_mm - previous_position);
        float raw_velocity =
            (filtered_position - previous_position) / dt_s;
        g_sample.position_mm = filtered_position;
        g_sample.velocity_mm_s += H3_VELOCITY_FILTER_ALPHA *
            (raw_velocity - g_sample.velocity_mm_s);
        g_sample.sample_dt_s = dt_s;
    }
    ++g_sample.good_frames;
    g_new_sample = true;
}

static bool parse_standard_line(const char *cursor, uint32_t now_ms)
{
    uint32_t sequence;
    uint32_t camera_ms;
    uint32_t valid;
    int32_t position_mm;

    cursor += 2;
    if (!parse_u32(&cursor, ',', &sequence) ||
        !parse_u32(&cursor, ',', &camera_ms) ||
        !parse_i32(&cursor, ',', &position_mm) ||
        !parse_u32(&cursor, '\0', &valid) ||
        (valid > 1U)) {
        return false;
    }
    commit_frame(sequence, camera_ms, position_mm, valid, now_ms);
    return true;
}

static bool parse_legacy_line(const char *cursor, uint32_t now_ms)
{
    uint32_t valid;
    uint32_t camera_ms;
    int32_t position_mm;
    const char *timestamp_start;
    const char *star;

    cursor += 5; /* BALL, */
    if (!parse_u32(&cursor, ',', &valid) || (valid > 1U) ||
        !parse_decimal_i32(&cursor, ',', &position_mm) ||
        !skip_field(&cursor) || !skip_field(&cursor)) {
        return false;
    }
    timestamp_start = cursor;
    star = cursor;
    while ((*star != '\0') && (*star != '*')) {
        ++star;
    }
    if (*star == '*') {
        if (!parse_u32(&timestamp_start, '*', &camera_ms)) {
            return false;
        }
    } else if (!parse_u32(&timestamp_start, '\0', &camera_ms)) {
        return false;
    }
    commit_frame(camera_ms, camera_ms, position_mm, valid, now_ms);
    return true;
}

static void parse_line(uint32_t now_ms)
{
    const char *cursor = g_line;

    g_line[g_line_length] = '\0';
    if ((cursor[0] == 'B') && (cursor[1] == ',')) {
        if (parse_standard_line(cursor, now_ms)) {
            return;
        }
    } else if ((cursor[0] == 'B') && (cursor[1] == 'A') &&
               (cursor[2] == 'L') && (cursor[3] == 'L') &&
               (cursor[4] == ',')) {
        if (parse_legacy_line(cursor, now_ms)) {
            return;
        }
    }
    ++g_sample.bad_frames;
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
    ++g_sample.rx_bytes;
    if (next == g_rx_tail) {
        ++g_sample.dropped_bytes;
        return;
    }
    g_rx_buffer[head] = byte;
    g_rx_head = next;
}

void h3_vision_process(uint32_t now_ms)
{
    uint16_t budget = VISION_RX_BUFFER_SIZE;

    while (g_rx_tail != g_rx_head) {
        if (budget-- == 0U) {
            /* Defensive recovery if an interrupt changes indices unexpectedly. */
            g_rx_tail = g_rx_head;
            ++g_sample.bad_frames;
            break;
        }
        uint8_t byte = g_rx_buffer[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1U) & VISION_RX_BUFFER_MASK;

        /* A fresh B is an unambiguous frame start and repairs half/garbled frames. */
        if ((byte == (uint8_t) 'B') && !inside_legacy_checksum()) {
            if (g_line_length != 0U) {
                ++g_sample.resync_events;
            }
            g_line[0] = 'B';
            g_line_length = 1U;
        } else if ((byte == (uint8_t) '\r') ||
                   (byte == (uint8_t) '\n')) {
            if (g_line_length != 0U) {
                parse_line(now_ms);
                g_line_length = 0U;
            }
        } else if (g_line_length == 0U) {
            /* Ignore boot logs, echoes and noise until a B frame header arrives. */
        } else if ((byte >= 0x20U) && (byte <= 0x7EU)) {
            if (g_line_length < (VISION_LINE_SIZE - 1U)) {
                g_line[g_line_length++] = (char) byte;
            } else {
                g_line_length = 0U;
                ++g_sample.bad_frames;
            }
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
