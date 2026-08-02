#include "i2c_bus.h"

#define I2C_FIFO_SIZE       (8U)        /* I2C 控制器 TX/RX FIFO 的最大字节数 */
#define I2C_TIMEOUT_LOOPS   (160000U)   /* 等待 I2C 状态改变时的超时计数 */

static bool wait_idle(I2C_Regs *bus)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;

    /* 等待 I2C 控制器进入空闲状态，超时则返回 false */
    while (((DL_I2C_getControllerStatus(bus) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) &&
           (timeout > 0U)) {
        --timeout;
    }
    return timeout > 0U;
}

static bool finish_transfer(I2C_Regs *bus)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;

    /* 等待传输完成，即控制器不再忙 */
    while (((DL_I2C_getControllerStatus(bus) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) &&
           (timeout > 0U)) {
        --timeout;
    }

    /* 检查是否超时或发生错误，出现异常时重置并清空 FIFO */
    if ((timeout == 0U) ||
        ((DL_I2C_getControllerStatus(bus) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)) {
        DL_I2C_resetControllerTransfer(bus);
        DL_I2C_flushControllerTXFIFO(bus);
        DL_I2C_flushControllerRXFIFO(bus);
        return false;
    }
    return true;
}

bool i2c_bus_write(I2C_Regs *bus, uint8_t address,
                   const uint8_t *data, size_t length)
{
    uint16_t loaded;

    /* 输入参数检查：数据指针不能为空，长度必须在 FIFO 范围内 */
    if ((data == NULL) || (length == 0U) || (length > I2C_FIFO_SIZE)) {
        return false;
    }

    /* 等待总线空闲后再开始写操作 */
    if (!wait_idle(bus)) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(bus);

    /* 将要写入的数据装载到 TX FIFO */
    loaded = DL_I2C_fillControllerTXFIFO(
        bus, data, (uint16_t) length);
    if (loaded != (uint16_t) length) {
        DL_I2C_flushControllerTXFIFO(bus);
        return false;
    }

    /* 启动 I2C 传输，设置为发送方向 */
    DL_I2C_startControllerTransfer(bus, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, (uint32_t) length);

    /* MSPM0 I2C_ERR_13 workaround：传输启动后等待至少3个I2C功能时钟。 */
    delay_cycles(64U);

    return finish_transfer(bus);
}

bool i2c_bus_read(I2C_Regs *bus, uint8_t address,
                  uint8_t *data, size_t length)
{
    size_t index;
    uint32_t timeout;

    /* 输入参数检查：接收缓冲区不能为空，长度必须在 FIFO 范围内 */
    if ((data == NULL) || (length == 0U) || (length > I2C_FIFO_SIZE)) {
        return false;
    }

    /* 等待总线空闲后再开始读操作 */
    if (!wait_idle(bus)) {
        return false;
    }

    DL_I2C_flushControllerRXFIFO(bus);

    /* 启动 I2C 传输，设置为接收方向 */
    DL_I2C_startControllerTransfer(bus, address,
        DL_I2C_CONTROLLER_DIRECTION_RX, (uint32_t) length);
    delay_cycles(64U);

    /* 从 RX FIFO 中逐字节读取数据，等待每个字节到达 */
    for (index = 0U; index < length; ++index) {
        timeout = I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(bus) && (timeout > 0U)) {
            --timeout;
        }
        if (timeout == 0U) {
            DL_I2C_resetControllerTransfer(bus);
            DL_I2C_flushControllerRXFIFO(bus);
            return false;
        }
        data[index] = DL_I2C_receiveControllerData(bus);
    }
    return finish_transfer(bus);
}

bool i2c_bus_write_read(I2C_Regs *bus, uint8_t address,
                       const uint8_t *tx, size_t tx_length,
                       uint8_t *rx, size_t rx_length)
{
    /* 先写入再读取，复用同一个硬件 I2C 控制器 */
    return i2c_bus_write(bus, address, tx, tx_length) &&
           i2c_bus_read(bus, address, rx, rx_length);
}

bool i2c_bus_probe(I2C_Regs *bus, uint8_t address)
{
    const uint8_t harmless = 0x00U;

    /* 通过发送一个空数据字节探测从设备是否存在 */
    return i2c_bus_write(bus, address, &harmless, 1U);
}
