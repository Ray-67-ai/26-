#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stdint.h>

/* 初始化 OLED 显示器，返回是否成功检测到模块。 */
bool ssd1306_init(void);

/* 返回 OLED 是否仍可用。 */
bool ssd1306_is_present(void);

/* 清空屏幕缓冲区但不立即刷新。 */
void ssd1306_clear(void);

/* 在指定页和列位置绘制一行 5x7 字符。 */
void ssd1306_draw_text(uint8_t x, uint8_t page, const char *text);

/* 绘制双倍大小字符，用于关键时间显示。 */
void ssd1306_draw_text_2x(uint8_t x, uint8_t page, const char *text);

/* 刷新 OLED 的下一页内容，避免长时间阻塞。 */
bool ssd1306_refresh_next_page(void);

#endif
