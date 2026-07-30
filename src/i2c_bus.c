#include "i2c_bus.h"

#define I2C_FIFO_SIZE       (8U)
#define I2C_TIMEOUT_LOOPS   (160000U)

static bool wait_idle(I2C_Regs *bus)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;
    while (((DL_I2C_getControllerStatus(bus) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) &&
           (timeout > 0U)) {
        --timeout;
    }
    return timeout > 0U;
}

static bool finish_transfer(I2C_Regs *bus)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;
    while (((DL_I2C_getControllerStatus(bus) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) &&
           (timeout > 0U)) {
        --timeout;
    }
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

    if ((data == NULL) || (length == 0U) || (length > I2C_FIFO_SIZE)) {
        return false;
    }
    if (!wait_idle(bus)) {
        return false;
    }
    DL_I2C_flushControllerTXFIFO(bus);
    loaded = DL_I2C_fillControllerTXFIFO(
        bus, data, (uint16_t) length);
    if (loaded != (uint16_t) length) {
        DL_I2C_flushControllerTXFIFO(bus);
        return false;
    }
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

    if ((data == NULL) || (length == 0U) || (length > I2C_FIFO_SIZE)) {
        return false;
    }
    if (!wait_idle(bus)) {
        return false;
    }
    DL_I2C_flushControllerRXFIFO(bus);
    DL_I2C_startControllerTransfer(bus, address,
        DL_I2C_CONTROLLER_DIRECTION_RX, (uint32_t) length);
    delay_cycles(64U);

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
    /* 两个阶段均由同一硬件I2C控制器完成，不使用GPIO软件模拟。 */
    return i2c_bus_write(bus, address, tx, tx_length) &&
           i2c_bus_read(bus, address, rx, rx_length);
}

bool i2c_bus_probe(I2C_Regs *bus, uint8_t address)
{
    const uint8_t harmless = 0x00U;
    return i2c_bus_write(bus, address, &harmless, 1U);
}
