#include "main.h"
#include "vl53l0x_api.h"

static VL53L0X_Error Vl53l0xPort_MapHalStatus(HAL_StatusTypeDef status) {
  return status == HAL_OK ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV device) {
  return device != NULL ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_INVALID_PARAMS;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV device) {
  return device != NULL ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_INVALID_PARAMS;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV device, uint8_t index, uint8_t *data, uint32_t count) {
  if (device == NULL || device->I2cHandle == NULL || data == NULL || count == 0U || count > UINT16_MAX) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }

  return Vl53l0xPort_MapHalStatus(HAL_I2C_Mem_Write(device->I2cHandle, device->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, data, (uint16_t)count, I2C_TIMEOUT_MS));
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV device, uint8_t index, uint8_t *data, uint32_t count) {
  if (device == NULL || device->I2cHandle == NULL || data == NULL || count == 0U || count > UINT16_MAX) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }

  return Vl53l0xPort_MapHalStatus(HAL_I2C_Mem_Read(device->I2cHandle, device->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, data, (uint16_t)count, I2C_TIMEOUT_MS));
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV device, uint8_t index, uint8_t data) {
  return VL53L0X_WriteMulti(device, index, &data, 1U);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV device, uint8_t index, uint16_t data) {
  uint8_t buffer[2] = {(uint8_t)(data >> 8U), (uint8_t)data};
  return VL53L0X_WriteMulti(device, index, buffer, sizeof(buffer));
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV device, uint8_t index, uint32_t data) {
  uint8_t buffer[4] = {(uint8_t)(data >> 24U), (uint8_t)(data >> 16U), (uint8_t)(data >> 8U), (uint8_t)data};
  return VL53L0X_WriteMulti(device, index, buffer, sizeof(buffer));
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV device, uint8_t index, uint8_t *data) {
  return VL53L0X_ReadMulti(device, index, data, 1U);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV device, uint8_t index, uint16_t *data) {
  uint8_t buffer[2];
  VL53L0X_Error status;

  if (data == NULL) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  status = VL53L0X_ReadMulti(device, index, buffer, sizeof(buffer));
  if (status == VL53L0X_ERROR_NONE) {
    *data = (uint16_t)(((uint16_t)buffer[0] << 8U) | buffer[1]);
  }
  return status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV device, uint8_t index, uint32_t *data) {
  uint8_t buffer[4];
  VL53L0X_Error status;

  if (data == NULL) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  status = VL53L0X_ReadMulti(device, index, buffer, sizeof(buffer));
  if (status == VL53L0X_ERROR_NONE) {
    *data = ((uint32_t)buffer[0] << 24U) | ((uint32_t)buffer[1] << 16U) | ((uint32_t)buffer[2] << 8U) | buffer[3];
  }
  return status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV device, uint8_t index, uint8_t andData, uint8_t orData) {
  uint8_t value;
  VL53L0X_Error status = VL53L0X_RdByte(device, index, &value);

  if (status != VL53L0X_ERROR_NONE) {
    return status;
  }

  return VL53L0X_WrByte(device, index, (uint8_t)((value & andData) | orData));
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV device) {
  if (device == NULL) {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }

  HAL_Delay(1U);
  return VL53L0X_ERROR_NONE;
}
