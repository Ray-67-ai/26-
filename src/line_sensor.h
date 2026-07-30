#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 光电巡线传感器读取结果。
 */
typedef struct {
    uint8_t black_mask;       /* bit0=X1(车辆左侧)，bit7=X8(车辆右侧) */
    uint8_t black_count;      /* 检测到黑色传感器数量 */
    int16_t error;            /* -3500(线在左) 到 +3500(线在右) */
    bool line_visible;        /* 是否检测到线路 */
    bool wide_line;           /* 是否检测到宽线，用于终点判定 */
} line_sample_t;

/* 读取一帧巡线传感器数据并返回结果。 */
line_sample_t line_sensor_read(void);

#endif
