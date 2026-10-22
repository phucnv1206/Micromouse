#include "vl53l1x_stm32f4_port.h"
#include "config.h"
#include "main.h"
#include "i2c_bus.h"

#include <limits.h>

/*
 * Port layer VL53L1X cho STM32F4 — dùng I2cBus_MemReadSync / MemWriteSync.
 * VL53L1X ULD dùng địa chỉ 8-bit; I2cBus nhận 7-bit → dịch phải 1 bit.
 * VL53L1X dùng địa chỉ thanh ghi 16-bit (I2C_MEMADD_SIZE_16BIT).
 */

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  if (pdata == NULL || count > UINT16_MAX) {
    return -1;
  }

  return I2cBus_MemWriteSync((uint8_t)(dev >> 1U), index, I2C_MEMADD_SIZE_16BIT,
                             pdata, (uint16_t)count) == HAL_OK
             ? 0
             : -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
  if (pdata == NULL || count > UINT16_MAX) {
    return -1;
  }

  return I2cBus_MemReadSync((uint8_t)(dev >> 1U), index, I2C_MEMADD_SIZE_16BIT,
                            pdata, (uint16_t)count) == HAL_OK
             ? 0
             : -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
  return VL53L1_WriteMulti(dev, index, &data, 1U);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
  uint8_t bytes[2] = {(uint8_t)(data >> 8U), (uint8_t)data};
  return VL53L1_WriteMulti(dev, index, bytes, sizeof(bytes));
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
  uint8_t bytes[4] = {
      (uint8_t)(data >> 24U),
      (uint8_t)(data >> 16U),
      (uint8_t)(data >> 8U),
      (uint8_t)data,
  };
  return VL53L1_WriteMulti(dev, index, bytes, sizeof(bytes));
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata) {
  return VL53L1_ReadMulti(dev, index, pdata, 1U);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata) {
  uint8_t bytes[2];
  if (pdata == NULL || VL53L1_ReadMulti(dev, index, bytes, sizeof(bytes)) != 0) {
    return -1;
  }
  *pdata = (uint16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
  return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata) {
  uint8_t bytes[4];
  if (pdata == NULL || VL53L1_ReadMulti(dev, index, bytes, sizeof(bytes)) != 0) {
    return -1;
  }
  *pdata = ((uint32_t)bytes[0] << 24U) |
           ((uint32_t)bytes[1] << 16U) |
           ((uint32_t)bytes[2] << 8U) |
           (uint32_t)bytes[3];
  return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
  (void)dev;
  if (wait_ms < 0) {
    return -1;
  }
  HAL_Delay((uint32_t)wait_ms);
  return 0;
}
