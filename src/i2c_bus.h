#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * 简化的 I2C 读写接口，使用 MSPM0 的低层 I2C 控制器。
 * 仅支持一次性传输，不支持跨页或分段传输。
 */

/*
 * 向指定 I2C 从机地址写入一段数据。
 * 返回 true 表示传输成功。
 */
bool i2c_bus_write(I2C_Regs *bus, uint8_t address,
                   const uint8_t *data, size_t length);

/*
 * 从指定 I2C 从机地址读取指定长度的数据。
 * 返回 true 表示成功读取到全部字节。
 */
bool i2c_bus_read(I2C_Regs *bus, uint8_t address,
                  uint8_t *data, size_t length);

/*
 * 先写数据再读数据，适用于多数 I2C 传感器或 OLED 命令/数据模式。
 * 使用同一个硬件总线完成连续写读。
 */
bool i2c_bus_write_read(I2C_Regs *bus, uint8_t address,
                       const uint8_t *tx, size_t tx_length,
                       uint8_t *rx, size_t rx_length);

/*
 * 探测 I2C 从机是否存在，通过向其写入一个无害字节。
 */
bool i2c_bus_probe(I2C_Regs *bus, uint8_t address);

#endif
