#include "ssd1306.h"
#include "i2c_bus.h"
#include "vehicle_config.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <string.h>

#define OLED_WIDTH       (128U)
#define OLED_PAGES       (8U)
#define OLED_CHUNK_DATA  (7U)
#define OLED_POWERUP_DELAY_CYCLES (3200000U)
#define OLED_RETRY_DELAY_CYCLES   (320000U)
#define OLED_INIT_RETRIES          (3U)

static uint8_t g_frame[OLED_WIDTH * OLED_PAGES];
static uint8_t g_address;
static uint8_t g_refresh_page;
static bool g_present;

/* 5x7字体：0-9、A-Z及常用标点。只包含本工程界面需要的字符。 */
static const uint8_t g_font[42][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
    {0x00,0x36,0x36,0x00,0x00}, /* : */
    {0x00,0x40,0x60,0x00,0x00}, /* . */
    {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x00,0x00,0x00,0x00,0x00}  /* space */
};

static uint8_t glyph_index(char c)
{
    if ((c >= '0') && (c <= '9')) {
        return (uint8_t) (c - '0');
    }
    if ((c >= 'A') && (c <= 'Z')) {
        return (uint8_t) (10 + c - 'A');
    }
    switch (c) {
        case ':': return 36U;
        case '.': return 37U;
        case '-': return 38U;
        case '/': return 39U;
        case '%': return 40U;
        default:  return 41U;
    }
}

static bool write_commands(const uint8_t *commands, uint8_t count)
{
    uint8_t packet[8];
    uint8_t offset = 0U;
    uint8_t chunk;

    while (offset < count) {
        chunk = (uint8_t) (count - offset);
        if (chunk > OLED_CHUNK_DATA) {
            chunk = OLED_CHUNK_DATA;
        }
        packet[0] = 0x00U;
        memcpy(&packet[1], &commands[offset], chunk);
        if (!i2c_bus_write(I2C_DISPLAY_INST, g_address,
                           packet, (size_t) chunk + 1U)) {
            return false;
        }
        offset = (uint8_t) (offset + chunk);
    }
    return true;
}

static bool try_init_at(uint8_t address)
{
    static const uint8_t init_commands[] = {
        0xAE,       /* display off */
        0xD5,0x80, /* clock */
        0xA8,0x3F, /* 1/64 multiplex */
        0xD3,0x00, /* display offset */
        0x40,       /* start line */
        0x8D,0x14, /* charge pump */
        0x20,0x02, /* page addressing mode */
        0xA1,       /* segment remap */
        0xC8,       /* COM scan direction */
        0xDA,0x12,
        0x81,0x7F,
        0xD9,0xF1,
        0xDB,0x40,
        0xA4,0xA6,0xAF
    };
    g_address = address;
    return write_commands(init_commands, sizeof(init_commands));
}

bool ssd1306_init(void)
{
    uint8_t attempt;
    uint8_t page;

    g_present = false;
    delay_cycles(OLED_POWERUP_DELAY_CYCLES);

    for (attempt = 0U;
         (attempt < OLED_INIT_RETRIES) && !g_present;
         ++attempt) {
        g_present = try_init_at(CFG_OLED_ADDRESS_PRIMARY);
        if (!g_present) {
            delay_cycles(OLED_RETRY_DELAY_CYCLES);
        }
    }
    for (attempt = 0U;
         (attempt < OLED_INIT_RETRIES) && !g_present;
         ++attempt) {
        g_present = try_init_at(CFG_OLED_ADDRESS_SECONDARY);
        if (!g_present) {
            delay_cycles(OLED_RETRY_DELAY_CYCLES);
        }
    }

    if (g_present) {
        ssd1306_clear();
        g_refresh_page = 0U;
        for (page = 0U; (page < OLED_PAGES) && g_present; ++page) {
            (void) ssd1306_refresh_next_page();
        }
    }
    return g_present;
}

bool ssd1306_is_present(void)
{
    return g_present;
}

void ssd1306_clear(void)
{
    memset(g_frame, 0, sizeof(g_frame));
}

void ssd1306_draw_text(uint8_t x, uint8_t page, const char *text)
{
    uint8_t column;
    uint8_t index;

    if ((page >= OLED_PAGES) || (text == NULL)) {
        return;
    }
    while ((*text != '\0') && (x <= (OLED_WIDTH - 6U))) {
        index = glyph_index(*text++);
        for (column = 0U; column < 5U; ++column) {
            g_frame[(uint16_t) page * OLED_WIDTH + x++] =
                g_font[index][column];
        }
        g_frame[(uint16_t) page * OLED_WIDTH + x++] = 0U;
    }
}

void ssd1306_draw_text_2x(uint8_t x, uint8_t page, const char *text)
{
    uint8_t column;
    uint8_t bit;
    uint8_t index;
    uint16_t expanded;

    if ((page >= (OLED_PAGES - 1U)) || (text == NULL)) {
        return;
    }
    while ((*text != '\0') && (x <= (OLED_WIDTH - 12U))) {
        index = glyph_index(*text++);
        for (column = 0U; column < 5U; ++column) {
            expanded = 0U;
            for (bit = 0U; bit < 7U; ++bit) {
                if ((g_font[index][column] & (1U << bit)) != 0U) {
                    expanded |= (uint16_t) (3U << (bit * 2U));
                }
            }
            g_frame[(uint16_t) page * OLED_WIDTH + x] =
                (uint8_t) expanded;
            g_frame[(uint16_t) (page + 1U) * OLED_WIDTH + x] =
                (uint8_t) (expanded >> 8U);
            ++x;
            g_frame[(uint16_t) page * OLED_WIDTH + x] =
                (uint8_t) expanded;
            g_frame[(uint16_t) (page + 1U) * OLED_WIDTH + x] =
                (uint8_t) (expanded >> 8U);
            ++x;
        }
        g_frame[(uint16_t) page * OLED_WIDTH + x] = 0U;
        g_frame[(uint16_t) (page + 1U) * OLED_WIDTH + x++] = 0U;
        g_frame[(uint16_t) page * OLED_WIDTH + x] = 0U;
        g_frame[(uint16_t) (page + 1U) * OLED_WIDTH + x++] = 0U;
    }
}

bool ssd1306_refresh_next_page(void)
{
    uint8_t commands[3];
    uint8_t packet[8];
    uint8_t offset;
    uint8_t chunk;

    if (!g_present) {
        return false;
    }
    commands[0] = (uint8_t) (0xB0U | g_refresh_page);
    commands[1] = 0x00U;
    commands[2] = 0x10U;
    if (!write_commands(commands, sizeof(commands))) {
        g_present = false;
        return false;
    }

    for (offset = 0U; offset < OLED_WIDTH; offset = (uint8_t) (offset + chunk)) {
        chunk = (uint8_t) (OLED_WIDTH - offset);
        if (chunk > OLED_CHUNK_DATA) {
            chunk = OLED_CHUNK_DATA;
        }
        packet[0] = 0x40U;
        memcpy(&packet[1],
               &g_frame[(uint16_t) g_refresh_page * OLED_WIDTH + offset],
               chunk);
        if (!i2c_bus_write(I2C_DISPLAY_INST, g_address,
                           packet, (size_t) chunk + 1U)) {
            g_present = false;
            return false;
        }
    }
    g_refresh_page = (uint8_t) ((g_refresh_page + 1U) % OLED_PAGES);
    return true;
}
