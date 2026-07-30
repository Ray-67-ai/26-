#include "line_sensor.h"
#include "vehicle_config.h"
#include "ti_msp_dl_config.h"

/*
 * 从 GPIO 读取光电传感器电平，并根据配置判断是否为黑色。
 */
static bool is_black(GPIO_Regs *port, uint32_t pin)
{
    const bool high = (DL_GPIO_readPins(port, pin) != 0U);
#if CFG_LINE_BLACK_IS_LOW
    return !high;
#else
    return high;
#endif
}

/*
 * 读取 8 路巡线传感器并计算当前线路误差。
 * 当线路不可见时返回上一次误差的方向值，以便继续寻找线路。
 */
line_sample_t line_sensor_read(void)
{
    static int16_t last_error = 0;
    static const int16_t weights[8] = {
        -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
    };
    bool black[8];
    line_sample_t result = {0U, 0U, 0, false, false};
    int32_t weighted_sum = 0;
    uint8_t i;

    black[0] = is_black(LINE_OUT1_PORT, LINE_OUT1_PIN);
    black[1] = is_black(LINE_OUT2_PORT, LINE_OUT2_PIN);
    black[2] = is_black(LINE_OUT3_PORT, LINE_OUT3_PIN);
    black[3] = is_black(LINE_OUT4_PORT, LINE_OUT4_PIN);
    black[4] = is_black(LINE_OUT5_PORT, LINE_OUT5_PIN);
    black[5] = is_black(LINE_OUT6_PORT, LINE_OUT6_PIN);
    black[6] = is_black(LINE_OUT7_PORT, LINE_OUT7_PIN);
    black[7] = is_black(LINE_OUT8_PORT, LINE_OUT8_PIN);

    for (i = 0U; i < 8U; ++i) {
        if (black[i]) {
            result.black_mask |= (uint8_t) (1U << i);
            ++result.black_count;
            weighted_sum += weights[i];
        }
    }

    result.line_visible = (result.black_count > 0U);
    result.wide_line = (result.black_count >= CFG_FINISH_WIDE_CHANNELS);
    if (result.line_visible) {
        result.error = (int16_t) (weighted_sum / result.black_count);
        last_error = result.error;
    } else {
        /* 短时丢线时继续朝最后看见黑线的一侧寻找，超时由上层停车。 */
        result.error = (last_error >= 0) ? 4000 : -4000;
    }
    return result;
}
