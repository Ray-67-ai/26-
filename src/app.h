#ifndef APP_H
#define APP_H

#include <stdint.h>

/* 初始化应用状态、定时器和外设。 */
void app_init(void);

/* 主循环调用，驱动状态机、速度控制和 OLED 更新。 */
void app_process(void);

/* 1ms 定时中断调用，用于系统运行时间计数。 */
void app_tick_1ms_isr(void);

/* 启动按键中断处理，触发开始/准备切换。 */
void app_start_key_isr(void);

/* 返回系统毫秒计数。 */
uint32_t app_millis(void);

#endif
